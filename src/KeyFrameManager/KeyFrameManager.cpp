#include "KeyFrameManager.h"
#include "../Tools/Tools.h"

namespace PKVIO
{
namespace KeyFrameManager
{
    
float RandomData(){
    float rd = (rand()%1000*1.0/1000);
    return rd;
}

Type::TpVecMapPointID MapPointIDManager::getMapPointIDsGeneratedByFrame(const Type::TpFrameID nFrameID) const
{
    TpVecMapPointID nVecMapPointID;
    for(auto Iter = mMapFromMapPointID2KptID.rbegin();Iter!=mMapFromMapPointID2KptID.rend();++Iter){
        const TpMapPoint& nMapPoint =  *Iter;
        const auto& nVecMeasurements = nMapPoint.getVecMeasurments();
        if(nVecMeasurements[0].first != nFrameID)
            break;
        nVecMapPointID.push_back(nMapPoint.getMapPointID());
    }
    return nVecMapPointID;
}

    
    

void KeyFrameManager::solve(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    const TpFrameID nCurFrameID     = fFrame.FrameID();
    
    auto FuncLogDebugKeyFrameGenerationInfo = [&](){
        mDebugKeyFrameGenerationInfo.mFrameID = nCurFrameID;
        mDebugKeyFrameGenerationInfo.log();
    };
    Tools::Timer tTimer(FuncLogDebugKeyFrameGenerationInfo, "KeyFrameManager Whole", false, &mDebugKeyFrameGenerationInfo.mTimeCostWhole);
    mDebugKeyFrameGenerationInfo    = DebugManager::DebugKeyFrameGenerationInfo();
    //AutoLogTimer 
    
    generateOneFrameCameraPose(fFrame);
    
    //int nCountSumTrackKpts        = mFrameKptIDMgr.sizeKeyPointsWithID();
    int nCountSumTrackKpts          = countTrackKptIDsWithMapPointIDAndValid(mFrameKptIDMgr);
    int nCountKptsOnThisFrame       = mFrameKptIDMgr.sizeKeyPoints();
    
    EnSLAMState eSLAMState          = updateTrackState(nCountSumTrackKpts, nCountKptsOnThisFrame);
    
    collectFirstDetectedKeyPointOnNonKeyFrame(fFrame, mFrameMatchResult, mFrameKptIDMgr);
        
    mDebugKeyFrameGenerationInfo.nCountTrackedKptIDs            = mFrameKptIDMgr.sizeKeyPointsWithID();
    mDebugKeyFrameGenerationInfo.nCountNewKptIDs                = mFrameKptIDMgr.sizeFirstKptIDsDetected();
    mDebugKeyFrameGenerationInfo.nCountTrackedMapPoint          = nCountSumTrackKpts;
    //cout << "Track | L+R Match: "  << nCountSumTrackKpts << " | " << mFrameKptIDMgr.str() << endl;
    mDebugKeyFrameGenerationInfo.nCountFirstDetectedKptIDs      = mLstFirstDetectedKptIDBeforKF.size();
    mDebugKeyFrameGenerationInfo.mLastKeyFrameID                = mKeyFrameIDGenerator.getLastID();
    
    switch(eSLAMState){
        case EnNeedKF: 
            createOneKeyFrame(fFrame, mFrameMatchResult, mFrameKptIDMgr);
            break;
        case EnLost:
            throw;
        case EnNonKF: 
            break;
        default: ;
    }
    return;
}


EnSLAMState KeyFrameManager::updateTrackState(int nCountSumTrackKpts, int nCountKptsOnThisFrame) {
    //
    cout << "TrackState: WithMapID|Kpts -" << nCountSumTrackKpts <<"|"<<nCountKptsOnThisFrame<<endl;
    EnSLAMState eSLAMState = EnUnKnown;
    std::string sSLAMState;
    if(nCountSumTrackKpts< DebugManager::getMinimumKptNumberToKeepTrackingWell()) {
        if(nCountKptsOnThisFrame >= DebugManager::getMinimumKptNumberToCreateKFOtherwiseLost()) {
            // new key frame
            sSLAMState = "#### need keyframe";
            eSLAMState = EnNeedKF;
        } else {
            // lost
            sSLAMState = "@@@@ lost";
            eSLAMState = EnLost;
        }
    } else {
        // normal non-keyframe
        sSLAMState = "      non-keyframe";
        eSLAMState = EnNonKF;
    }
    mDebugKeyFrameGenerationInfo.mStrKeyFrameGeneration = sSLAMState;
    return eSLAMState;
}

void KeyFrameManager::createOneKeyFrame(Type::Frame& fFrame,const KeyPointManager::FrameMatchResult& mFrameMatchResult,
                        KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) 
{
    //Tools::Timer tTimer("createOneKeyFrame", true);
    
    //TpVecKeyPointID nVecAllKptIDs = mFrameKptIDMgr.getAllKeyPointIDs();
    TpVecKeyPointID nVecAllKptIDs;
    
    int nCountFirstDetectedKptIDToMapIDs = 0;
    {
        int nCountFirstDetectedKptIDs = 0;
        auto FuncAddFirstDetectedKptIDAndMapIDInfo = [&](){
            mDebugKeyFrameGenerationInfo.nCountFirstDetectedKptIDToMapIDs   = nCountFirstDetectedKptIDToMapIDs;
        };
        Tools::Timer tTimer(FuncAddFirstDetectedKptIDAndMapIDInfo,"createOneKeyFrame - generate mappoint", 
                            false, &mDebugKeyFrameGenerationInfo.mTimeCostGenerateMapPoint);
        
        nVecAllKptIDs = mFrameKptIDMgr.getAllKeyPointIDs(); 
        TpSetKeyPointID nSetFirstDetectedKptID(mLstFirstDetectedKptIDBeforKF.begin(), mLstFirstDetectedKptIDBeforKF.end());
        nCountFirstDetectedKptIDs = nSetFirstDetectedKptID.size();
        mLstFirstDetectedKptIDBeforKF.clear();
        
        for(auto Iter=nVecAllKptIDs.begin(), EndIter = nVecAllKptIDs.end();Iter!=EndIter;++Iter){
        TpKeyPointID nKptID = *Iter;
        
        // Filter 1: has been solved as above.
        if(!nSetFirstDetectedKptID.count(nKptID))
            continue;
        
        // Just speed optimized.
        // TODO: Filter 2: has been co-vis by other nearest KFs.
        
        // Filter 3: 
        //if(mMapPointIDManager.isExistingMapPointID(nKptID))
        //    continue;
        
        ++nCountFirstDetectedKptIDToMapIDs;
        TpMapPointID nMapPointID    = mMapPointIDManager.generateOneMapPointID(nKptID);
        
        // only this KF observe this kpt, other observe is non-KF.
        // so treat this new Kpts as other Kpt below, add their obersevation once.
        }
        mMapPointIDManager.log();
    }
   
    {
        Tools::Timer tTimer("createOneKeyFrame - add measuremnt", false , &mDebugKeyFrameGenerationInfo.mTimeCostAddMeasurement);
        // copy the measuremnt from this frame to MapPoint.
        TpVecKeyPointIndex nVecAllKptIndex = mFrameKptIDMgr.getAllKeyPointIndexsWithID();
        for(int nIdxKptID =0,nSzKptID=nVecAllKptIDs.size();nIdxKptID<nSzKptID;++nIdxKptID){
            TpKeyPointID nKptID         = nVecAllKptIDs[nIdxKptID];   
            
            TpMapPoint& mMapPoint = mMapPointIDManager.MapPointByKeyPointID(nKptID);
            mMapPoint.addMeasurement(fFrame.FrameID(), nVecAllKptIndex[nIdxKptID]);
        }
        assert(nCountFirstDetectedKptIDToMapIDs == (int)mMapPointIDManager.getMapPointIDsGeneratedByFrame(fFrame.FrameID()).size());
    }
    
    // 
    
    TpKeyFrameID nNewKFID = generateOneKeyFrame(fFrame);
    
    
}


void KeyFrameManager::collectFirstDetectedKeyPointOnNonKeyFrame(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) 
{
    Tools::Timer tTimer("collectDetectedKptID", false, &mDebugKeyFrameGenerationInfo.mTimeCostAccFirstDetectedKptIDs);
    TpVecKeyPointID nLstFirstDetectedKptID = mFrameKptIDMgr.getFirstDetectedKptIDs();
    mLstFirstDetectedKptIDBeforKF.insert(mLstFirstDetectedKptIDBeforKF.end(), nLstFirstDetectedKptID.begin(),nLstFirstDetectedKptID.end());
}

int KeyFrameManager::countTrackKptIDsWithMapPointIDAndValid(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    Tools::Timer tTimer("countTrackKptIDsWithMapPointIDAndValid", false , &mDebugKeyFrameGenerationInfo.mTimeCostCountTrackedMapPoint);
    TpVecKeyPointID nKptIDs = mFrameKptIDMgr.getAllKeyPointIDs();
    int nCount = std::count_if(nKptIDs.begin(),nKptIDs.end(),[&](const TpKeyPointID nKptID)->bool {
        //return mMapPointIDManager.isExistingMapPointID(nKptID);
        //TpMapPointID nMpID = INVALIDMAPPOINTID;
        //bool b = mMapPointIDManager.isExistingMapPointID(nKptID, nMpID);
        //cout << "count Track one:KptID|MpID-"<< nKptID<<"|"<<nMpID<< "|" <<b <<  endl;
        //return mMapPointIDManager.isExistingMapPointID(nKptID);
        return mMapPointIDManager.isExistingMapPointID(nKptID) && mMapPointIDManager.MapPointByKeyPointID(nKptID).getMapPoint3DValid();
    });
    //cout << "Count Track count:" << nCount<<endl;
    //for(int nIdx=0;nIdx<nKptIDs.size();++nIdx)cout << "count-kptID:" << nIdx<< "|" <<nKptIDs[nIdx]<<endl;
    //cout << "count-MpID:" << endl;
    //mMapPointIDManager.log();
    //cout << "count-MpID:end" << endl;
    return nCount;
}


int KeyFrameManager::countTrackKptIDsWithMapPointID(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    Tools::Timer tTimer("countTrackKptIDsWithMapPointID", false , &mDebugKeyFrameGenerationInfo.mTimeCostCountTrackedMapPoint);
    TpVecKeyPointID nKptIDs = mFrameKptIDMgr.getAllKeyPointIDs();
    int nCount = std::count_if(nKptIDs.begin(),nKptIDs.end(),[&](const TpKeyPointID nKptID)->bool {
        //return mMapPointIDManager.isExistingMapPointID(nKptID);
        //TpMapPointID nMpID = INVALIDMAPPOINTID;
        //bool b = mMapPointIDManager.isExistingMapPointID(nKptID, nMpID);
        //cout << "count Track one:KptID|MpID-"<< nKptID<<"|"<<nMpID<< "|" <<b <<  endl;
        return mMapPointIDManager.isExistingMapPointID(nKptID);
    });
    //cout << "Count Track count:" << nCount<<endl;
    //for(int nIdx=0;nIdx<nKptIDs.size();++nIdx)cout << "count-kptID:" << nIdx<< "|" <<nKptIDs[nIdx]<<endl;
    //cout << "count-MpID:" << endl;
    //mMapPointIDManager.log();
    //cout << "count-MpID:end" << endl;
    return nCount;
}


void KeyFrameManager::getKptIDsWithMapPointID(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr,
                                                    TpFrameKptIDMapPointPair& nFrameKptIDMapPointPair) 
{
    TpVecKeyPointID nVecKptIDs; TpVecKeyPointIndex nVecKptIndexs;
    mFrameKptIDMgr.getAllKptIDsAndIdexs(nVecKptIDs, nVecKptIndexs);
    
    for(int nIdxKptId=0,nSzKptID=nVecKptIDs.size();nIdxKptId<nSzKptID;++nIdxKptId){
        const TpKeyPointID      nKptID      = nVecKptIDs[nIdxKptId];
        const TpKeyPointIndex   nKptIndex   = nVecKptIndexs[nIdxKptId];
        TpMapPointID            nMapPointID = INVALIDMAPPOINTID;
        if(mMapPointIDManager.isExistingMapPointID(nKptID, nMapPointID)){
            nFrameKptIDMapPointPair.addKptIDMapPointPair(TpKptIDMapPointPair(nKptID,nKptIndex,nMapPointID));
        }
    }
}

}
}

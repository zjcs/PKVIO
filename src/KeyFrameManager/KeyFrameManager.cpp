#include "KeyFrameManager.h"
#include "../Tools/Tools.h"

namespace PKVIO
{
namespace KeyFrameManager
{
    
    
    

void KeyFrameManager::solve(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    const TpFrameID nCurFrameID     = fFrame.FrameID();
    
    auto FuncLogDebugKeyFrameGenerationInfo = [&](){
        mDebugKeyFrameGenerationInfo.mFrameID = nCurFrameID;
        mDebugKeyFrameGenerationInfo.log();
    };
    Tools::Timer tTimer(FuncLogDebugKeyFrameGenerationInfo, "KeyFrameManager Whole", false, &mDebugKeyFrameGenerationInfo.mTimeCostWhole);
    mDebugKeyFrameGenerationInfo    = DebugManager::DebugKeyFrameGenerationInfo();
    //AutoLogTimer 
    
    //int nCountSumTrackKpts        = mFrameKptIDMgr.sizeKeyPointsWithID();
    int nCountSumTrackKpts          = countTrackKptIDsWithMapPointID(mFrameKptIDMgr);
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
    EnSLAMState eSLAMState = EnUnKnown;
    std::string sSLAMState;
    if(nCountSumTrackKpts<30) {
        if(nCountKptsOnThisFrame > 50) {
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
    
    {
        int nCountFirstDetectedKptIDs = 0, nCountFirstDetectedKptIDToMapIDs = 0;
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
    }
   
    {
        Tools::Timer tTimer("createOneKeyFrame - add measuremnt", false , &mDebugKeyFrameGenerationInfo.mTimeCostAddMeasurement);
        // copy the measuremnt from this frame to MapPoint.
        TpVecKeyPointIndex nVecAllKptIndex = mFrameKptIDMgr.getAllKeyPointIndexsWithID();
        for(int nIdxKptID =0,nSzKptID=nVecAllKptIDs.size();nIdxKptID<nSzKptID;++nIdxKptID){
            TpKeyPointID nKptID         = nVecAllKptIDs[nIdxKptID];   
            
            TpMapPoint& mMapPoint = mMapPointIDManager.MapPointByKeyPointID(nKptID);
            mMapPoint.addMeasurement(fFrame.FrameIndex(), nVecAllKptIndex[nIdxKptID]);
        }
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


int KeyFrameManager::countTrackKptIDsWithMapPointID(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    Tools::Timer tTimer("countTrackKptIDsWithMapPointID", false , &mDebugKeyFrameGenerationInfo.mTimeCostCountTrackedMapPoint);
    TpVecKeyPointID nKptIDs = mFrameKptIDMgr.getAllKeyPointIDs();
    return std::count_if(nKptIDs.begin(),nKptIDs.end(),[&](const TpKeyPointID nKptID) {
        return mMapPointIDManager.isExistingMapPointID(nKptID);
    });
}


void KeyFrameManager::getKptIDsWithMapPointID(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr,
                                                    TpFrameKptIDMapPointPair& nFrameKptIDMapPointPair) 
{
    TpVecKeyPointID nVecKptIDs; TpVecKeyPointIndex nVecKptIndexs;
    mFrameKptIDMgr.getAllKptIDsAndIdexs(nVecKptIDs, nVecKptIndexs);
    
    TpMapPointID                nMapPointID = INVALIDMAPPOINTID;
    for(int nIdxKptId=0,nSzKptID=nVecKptIDs.size();nIdxKptId<nSzKptID;++nIdxKptId){
        const TpKeyPointID      nKptID      = nVecKptIDs[nIdxKptId];
        const TpKeyPointIndex   nKptIndex   = nVecKptIndexs[nIdxKptId];
        if(mMapPointIDManager.isExistingMapPointID(nKptID)){
            nFrameKptIDMapPointPair.addKptIDMapPointPair(TpKptIDMapPointPair(nKptID,nKptIndex,nMapPointID));
        }
    }
}

}
}

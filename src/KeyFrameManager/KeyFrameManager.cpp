#include "KeyFrameManager.h"
#include "../Tools/Tools.h"

namespace PKVIO
{
namespace KeyFrameManager
{
    
    
    

void KeyFrameManager::solve(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    const TpFrameID nCurFrameID = fFrame.FrameID();
   Tools::Timer tTimer("KeyFrameManager Whole");
   //AutoLogTimer 
    
    //int nCountSumTrackKpts = mFrameKptIDMgr.sizeKeyPointsWithID();
    int nCountSumTrackKpts = countTrackKptIDsWithMapPointID(mFrameKptIDMgr);
    int nCountKptsOnThisFrame = mFrameKptIDMgr.sizeKeyPoints();
    
    cout << "Track | L+R Match: "  << nCountSumTrackKpts << " | " << mFrameKptIDMgr.str() << endl;
    
    EnSLAMState eSLAMState = updateTrackState(nCountSumTrackKpts, nCountKptsOnThisFrame);
    
    collectFirstDetectedKeyPointOnNonKeyFrame(fFrame, mFrameMatchResult, mFrameKptIDMgr);
    
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
}


EnSLAMState KeyFrameManager::updateTrackState(int nCountSumTrackKpts, int nCountKptsOnThisFrame) {
    //
    if(nCountSumTrackKpts<30) {
        if(nCountKptsOnThisFrame > 50) {
            // new key frame
            cout << "#### need new key frame" <<endl;
            return EnNeedKF;
        } else {
            // lost
            cout << "lost" << endl;
            return EnLost;
        }
    } else {
        // normal non-keyframe
        cout << "normal non-keyframe" <<endl;
        return EnNonKF;
    }
}

void KeyFrameManager::createOneKeyFrame(Type::Frame& fFrame,const KeyPointManager::FrameMatchResult& mFrameMatchResult,
                        KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) 
{
    Tools::Timer tTimer("createOneKeyFrame");
    {
        
        Tools::Timer tTimer("createOneKeyFrame - set");
        TpSetKeyPointID nSetFirstDetectedKptID(mLstFirstDetectedKptIDBeforKF.begin(), mLstFirstDetectedKptIDBeforKF.end());
    }
    TpSetKeyPointID nSetFirstDetectedKptID(mLstFirstDetectedKptIDBeforKF.begin(), mLstFirstDetectedKptIDBeforKF.end());
     cout << "First Detected KptIDs: " << nSetFirstDetectedKptID.size() << endl;
     mLstFirstDetectedKptIDBeforKF.clear();
     
    TpVecKeyPointID nVecAllKptIDs = mFrameKptIDMgr.getAllKeyPointIDs();
    
    {
        Tools::Timer tTimer("createOneKeyFrame - generate mappoint");
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
        
        
        TpMapPointID nMapPointID    = mMapPointIDManager.generateOneMapPointID(nKptID);
        
        // only this KF observe this kpt, other observe is non-KF.
        // so treat this new Kpts as other Kpt below, add their obersevation once.
        }
    }
   
    {
        Tools::Timer tTimer("createOneKeyFrame - add measuremnt");
        // copy the measuremnt from this frame to MapPoint.
        TpVecKeyPointIndex nVecAllKptIndex = mFrameKptIDMgr.getAllKeyPointIndexsWithID();
        for(int nIdxKptID =0,nSzKptID=nVecAllKptIDs.size();nIdxKptID<nSzKptID;++nIdxKptID){
            TpKeyPointID nKptID         = nVecAllKptIDs[nIdxKptID];   
            
            TpMapPoint& mMapPoint = mMapPointIDManager.MapPointByKeyPointID(nKptID);
            mMapPoint.addMeasurement(fFrame.FrameIndex(), nVecAllKptIndex[nIdxKptID]);
        }
    }
    
    // 
    
    generateOneKeyFrame(fFrame);
    
    
}


void KeyFrameManager::collectFirstDetectedKeyPointOnNonKeyFrame(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) 
{
    Tools::Timer tTimer("collectFirstDetectedKeyPointOnNonKeyFrame");
    TpVecKeyPointID nLstFirstDetectedKptID = mFrameKptIDMgr.getFirstDetectedKptIDs();
    mLstFirstDetectedKptIDBeforKF.insert(mLstFirstDetectedKptIDBeforKF.end(), nLstFirstDetectedKptID.begin(),nLstFirstDetectedKptID.end());
}


int KeyFrameManager::countTrackKptIDsWithMapPointID(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    Tools::Timer tTimer("countTrackKptIDsWithMapPointID");
    TpVecKeyPointID nKptIDs = mFrameKptIDMgr.getAllKeyPointIDs();
    return std::count_if(nKptIDs.begin(),nKptIDs.end(),[&](const TpKeyPointID nKptID) {
        return mMapPointIDManager.isExistingMapPointID(nKptID);
    });
}

}
}

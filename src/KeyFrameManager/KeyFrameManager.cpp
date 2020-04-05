#include "KeyFrameManager.h"

namespace PKVIO
{
namespace KeyFrameManager
{
    
    
    

void KeyFrameManager::solve(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    const TpFrameID nCurFrameID = fFrame.FrameID();
    
    
    //int nCountSumTrackKpts = mFrameKptIDMgr.sizeKeyPointsWithID();
    int nCountSumTrackKpts = countTrackKptIDsWithMapPointID(mFrameKptIDMgr);
    int nCountKptsOnThisFrame = mFrameKptIDMgr.sizeKeyPoints();
    
    cout << "Track | L+R Match: "  << nCountSumTrackKpts << " | " << mFrameKptIDMgr.str() << endl;
    
    EnSLAMState eSLAMState = updateTrackState(nCountSumTrackKpts, nCountKptsOnThisFrame);
    
    switch(eSLAMState){
        case EnNeedKF: createOneKeyFrame(fFrame, mFrameMatchResult, mFrameKptIDMgr); break;
        case EnLost: throw;
        case EnNonKF: break;
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
    
    //TpVecKeyPointID nVecFirstDetectedKptIDs; TpVecKeyPointIndex nVecFirstDetectedKptIndex;
    //mFrameKptIDMgr.getFirstDetectedKptIDsAndIdexs(nVecFirstDetectedKptIDs, nVecFirstDetectedKptIndex);
    //
    //// State 1: initialized of this new kf. maybe small
    //for(int nIdxKptID =0,nSzKptID=nVecFirstDetectedKptIDs.size();nIdxKptID<nSzKptID;++nIdxKptID){
    //    
    //    TpKeyPointID nKptID         = nVecFirstDetectedKptIDs[nIdxKptID];
    //    TpMapPointID nMapPointID    = mMapPointIDManager.generateOneMapPointID(nKptID);
    //    // all measurement to MapPointID, except the measurement from this frame which will be solved at bellow.
    //}
    
    //// State 2: the kpt is created of non-kf, but tracked in this kf. maybe many.
    TpVecKeyPointID nVecAllKptIDs = mFrameKptIDMgr.getAllKeyPointIDs();
    //TpSetKeyPointID nSetFirstDetectedKptID (nVecFirstDetectedKptIDs.begin(), nVecFirstDetectedKptIDs.end());
    for(auto Iter=nVecAllKptIDs.begin(), EndIter = nVecAllKptIDs.end();Iter!=EndIter;++Iter){
        TpKeyPointID nKptID = *Iter;
        
        // Filter 1: has been solved as above.
        //if(nSetFirstDetectedKptID.count(nKptID))
        //    continue;
        
        // Just speed optimized.
        // TODO: Filter 2: has been co-vis by other nearest KFs.
        
        // Filter 3: 
        if(mMapPointIDManager.isExistingMapPointID(nKptID))
            continue;
        
        
        TpMapPointID nMapPointID    = mMapPointIDManager.generateOneMapPointID(nKptID);
        
        // only this KF observe this kpt, other observe is non-KF.
        // so treat this new Kpts as other Kpt below, add their obersevation once.
    }
    
    // copy the measuremnt from this frame to MapPoint.
    TpVecKeyPointIndex nVecAllKptIndex = mFrameKptIDMgr.getAllKeyPointIndexsWithID();
    for(int nIdxKptID =0,nSzKptID=nVecAllKptIDs.size();nIdxKptID<nSzKptID;++nIdxKptID){
        TpKeyPointID nKptID         = nVecAllKptIDs[nIdxKptID];   
        
        TpMapPoint& mMapPoint = mMapPointIDManager.MapPointByKeyPointID(nKptID);
        mMapPoint.addMeasurement(fFrame.FrameIndex(), nVecAllKptIndex[nIdxKptID]);
    }
    
    // 
    
    generateOneKeyFrame(fFrame);
    
    
}

}
}

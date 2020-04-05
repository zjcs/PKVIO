#ifndef __KEYFRAMEMANAGER_H__
#define __KEYFRAMEMANAGER_H__

#include <memory>
#include "../Type/type.h"
#include "../KeyPointManager/KeyPointManager.h"
#include "../CoVisManager/KeyPointIDManager.h"
#include "KFMapPointManager.h"


namespace PKVIO
{
namespace KeyFrameManager{
    
//class MapPointID2KeyPointIDHistory:public DataHistoryTemplate<TpMapPointID, TpKeyPointID>{
//public:
//    MapPointID2KeyPointIDHistory():DataHistoryTemplate([&](TpMapPointID& pMapPtID, const TpKeyPointID pKptID)->bool{
//        if(mHistory.size()<=pMapPtID)
//            return false;
//        auto Iter = mHistory.begin();
//        std::advance(Iter, pMapPtID-1);
//        return *Iter == pKptID;
//    }){}
//};
    
class    MapPointIDManager {
public:
    bool            isExistingMapPointID(const TpKeyPointID& nKptID){ 
                        return std::count_if(mMapFromMapPointID2KptID.begin(), mMapFromMapPointID2KptID.end(), [=](TpMapPoint& mMapPoint){
                            return mMapPoint.getKeyPointID()==nKptID;
                        }); 
                    }
                    
    TpMapPointID    generateOneMapPointID(const TpKeyPointID& nKptID){ 
                        TpMapPointID nMapPointID = mMapPointIDGenerator.create();
                        if(nMapPointID!=(TpMapPointID)mMapFromMapPointID2KptID.size()) 
                            throw;
                        mMapFromMapPointID2KptID.push_back(TpMapPoint(nKptID));
                        return nMapPointID;
                    }
                    
    TpMapPoint&     MapPoint(const TpMapPointID nMapPointID){
                        return mMapFromMapPointID2KptID[nMapPointID];
                    }
                    
    TpMapPoint&     MapPointByKeyPointID(const TpKeyPointID& nKptID){
                        auto Iter = std::find_if(mMapFromMapPointID2KptID.begin(), mMapFromMapPointID2KptID.end(), [=](TpMapPoint& mMapPoint){
                            return mMapPoint.getKeyPointID()==nKptID;
                        });                
                        if(Iter == mMapFromMapPointID2KptID.end())
                            throw;
                        return *Iter;
                    }
    
    
    TpKeyPointID    getKptID(const TpMapPointID nMapPointID){return mMapFromMapPointID2KptID[nMapPointID].getKeyPointID();}
    int             sizeMapPointID(void){ return (int)mMapFromMapPointID2KptID.size(); }
private:
    Type::IDGenerator<TpMapPointID>     mMapPointIDGenerator;
    vector<TpMapPoint>                  mMapFromMapPointID2KptID;
   //vector<> 
};
   
class KeyFrameManager
{
public:
    
    void                    solve(Type::Frame& fFrame,const KeyPointManager::FrameMatchResult& mFrameMatchResult,
                                    KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
    
    void                    createOneKeyFrame(Type::Frame& fFrame,const KeyPointManager::FrameMatchResult& mFrameMatchResult,
                                    KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
    bool                    isKeyFrame(const TpFrameID nFrameID){
                                return std::count(mMapKFID2FrameID.begin(), mMapKFID2FrameID.end(), nFrameID);
                            }
    
    inline int              countTrackKptIDsWithMapPointID(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr){
                                TpVecKeyPointID nKptIDs = mFrameKptIDMgr.getAllKeyPointIDs();
                                return std::count_if(nKptIDs.begin(),nKptIDs.end(),[&](const TpKeyPointID nKptID){return mMapPointIDManager.isExistingMapPointID(nKptID);});
                            }
    
protected:
    EnSLAMState             updateTrackState(int nCountSumTrackKpts, int nCountKptsOnThisFrame);
    
    void                    generateOneKeyFrame(Type::Frame& fFrame){
                                TpKeyFrameID nKFID = mKeyFrameIDGenerator.create();
                                if(nKFID!=(int)mMapKFID2FrameID.size())
                                    throw;
                                mMapKFID2FrameID.push_back(fFrame.FrameID());
                            }
private:
    MapPointIDManager       mMapPointIDManager;
    Type::IDGenerator<TpKeyFrameID>     mKeyFrameIDGenerator;
    vector<TpFrameID>                   mMapKFID2FrameID;
};

typedef shared_ptr<KeyFrameManager> PtrKeyFrameManager;
    
}
}

#endif

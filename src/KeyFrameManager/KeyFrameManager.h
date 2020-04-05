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
                        //return std::count_if(mMapFromMapPointID2KptID.begin(), mMapFromMapPointID2KptID.end(), [=](TpMapPoint& mMapPoint){ return mMapPoint.getKeyPointID()==nKptID; }); 
                        return (int)mMapFromKptID2MapPointID.size()>nKptID&&mMapFromKptID2MapPointID[nKptID]!=INVALIDMAPPOINTID;
                    }
                    
    TpMapPointID    generateOneMapPointID(const TpKeyPointID& nKptID){ 
                        TpMapPointID nMapPointID = mMapPointIDGenerator.create();
                        if(nMapPointID!=(TpMapPointID)mMapFromMapPointID2KptID.size()) 
                            throw;
                        mMapFromMapPointID2KptID.push_back(TpMapPoint(nKptID));
                        
                        if((int)mMapFromKptID2MapPointID.capacity() <= nKptID){
                            mMapFromKptID2MapPointID.reserve(2*mMapFromKptID2MapPointID.capacity());
                        }
                        if((int)mMapFromKptID2MapPointID.size()<= nKptID){
                            mMapFromKptID2MapPointID.insert(mMapFromKptID2MapPointID.end(), int(nKptID-mMapFromKptID2MapPointID.size()+1), INVALIDMAPPOINTID);
                        }
                        mMapFromKptID2MapPointID[nKptID] = nMapPointID;
                        
                        return nMapPointID;
                    }
                    
    TpMapPoint&     MapPoint(const TpMapPointID nMapPointID){
                        return mMapFromMapPointID2KptID[nMapPointID];
                    }
                    
    TpMapPoint&     MapPointByKeyPointID(const TpKeyPointID& nKptID){
                        if(!isExistingMapPointID(nKptID)){
                            cout << "**** Error: MapPointByKeyPointID" << nKptID <<endl;
                            throw;
                        }
                        return mMapFromMapPointID2KptID[mMapFromKptID2MapPointID[nKptID]];
                        
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
    vector<TpMapPointID>                mMapFromKptID2MapPointID;
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
    
    int                     countTrackKptIDsWithMapPointID(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
    
protected:
    EnSLAMState             updateTrackState(int nCountSumTrackKpts, int nCountKptsOnThisFrame);
    
    inline void             generateOneKeyFrame(Type::Frame& fFrame){
                                TpKeyFrameID nKFID = mKeyFrameIDGenerator.create();
                                if(nKFID!=(int)mMapKFID2FrameID.size())
                                    throw;
                                mMapKFID2FrameID.push_back(fFrame.FrameID());
                            }
                            
    void                    collectFirstDetectedKeyPointOnNonKeyFrame(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
    
private:
    MapPointIDManager                   mMapPointIDManager;
    Type::IDGenerator<TpKeyFrameID>     mKeyFrameIDGenerator;
    vector<TpFrameID>                   mMapKFID2FrameID;
    
    list<TpKeyPointID>                  mLstFirstDetectedKptIDBeforKF;
};

typedef shared_ptr<KeyFrameManager> PtrKeyFrameManager;
    
}
}

#endif

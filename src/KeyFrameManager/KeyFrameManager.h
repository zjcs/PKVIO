#ifndef __KEYFRAMEMANAGER_H__
#define __KEYFRAMEMANAGER_H__

#include <memory>
#include "../Type/type.h"
#include "../KeyPointManager/KeyPointManager.h"
#include "../CoVisManager/KeyPointIDManager.h"
#include "KFMapPointManager.h"
#include "../DebugManager/DebugManager.h"


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
    inline bool     isExistingMapPointID(const TpKeyPointID& nKptID, TpMapPointID& nMapPointID){ 
                        if((int)mMapFromKptID2MapPointID.size()>nKptID){
                            nMapPointID = mMapFromKptID2MapPointID[nKptID] ;
                            return nMapPointID !=INVALIDMAPPOINTID;
                        }
                        return false;
                    }
    bool            isExistingMapPointID(const TpKeyPointID& nKptID){ 
                        //return std::count_if(mMapFromMapPointID2KptID.begin(), mMapFromMapPointID2KptID.end(), [=](TpMapPoint& mMapPoint){ return mMapPoint.getKeyPointID()==nKptID; }); 
                            //log();
                        return (int)mMapFromKptID2MapPointID.size()>nKptID && mMapFromKptID2MapPointID[nKptID]!=INVALIDMAPPOINTID;
                    }
                    
    TpMapPointID    generateOneMapPointID(const TpKeyPointID& nKptID){ 
                        TpMapPointID nMapPointID = mMapPointIDGenerator.create();
                        if(nMapPointID!=(TpMapPointID)mMapFromMapPointID2KptID.size()) 
                            throw;
                        mMapFromMapPointID2KptID.push_back(TpMapPoint(nMapPointID, nKptID));
                        
                        if((int)mMapFromKptID2MapPointID.capacity() <= nKptID){
                            mMapFromKptID2MapPointID.reserve(2*mMapFromKptID2MapPointID.capacity());
                        }
                        if((int)mMapFromKptID2MapPointID.size()<= nKptID){
                            mMapFromKptID2MapPointID.insert(mMapFromKptID2MapPointID.end(), int(nKptID-mMapFromKptID2MapPointID.size()+1), INVALIDMAPPOINTID);
                        }
                        mMapFromKptID2MapPointID[nKptID] = nMapPointID;
                        
                        return nMapPointID;
                    }
    void log(void)const{
       //for(int nIdx=0;nIdx<mMapFromKptID2MapPointID.size();++nIdx){cout << "Check KptID|MpID:"<<nIdx<<"|"<<mMapFromKptID2MapPointID[nIdx]<<endl;} 
    }
                    
    TpMapPoint&     MapPoint(const TpMapPointID nMapPointID){
                        assert(nMapPointID < (int)mMapFromMapPointID2KptID.size());
                        return mMapFromMapPointID2KptID[nMapPointID];
                    }
                    
    TpMapPoint&     MapPointByKeyPointID(const TpKeyPointID& nKptID){
                        if(!isExistingMapPointID(nKptID)){
                            cout << "**** Error: MapPointByKeyPointID" << nKptID <<endl;
                            throw;
                        }
                        return mMapFromMapPointID2KptID[mMapFromKptID2MapPointID[nKptID]];
                        
                        //auto Iter = std::find_if(mMapFromMapPointID2KptID.begin(), mMapFromMapPointID2KptID.end(), [=](TpMapPoint& mMapPoint){
                        //    return mMapPoint.getKeyPointID()==nKptID;
                        //});                
                        //if(Iter == mMapFromMapPointID2KptID.end())
                        //    throw;
                        //return *Iter;
                    }
    
    
    TpKeyPointID    getKptID(const TpMapPointID nMapPointID)const{return mMapFromMapPointID2KptID[nMapPointID].getKeyPointID();}
    int             sizeMapPointID(void)const{ return (int)mMapFromMapPointID2KptID.size(); }
    TpVecMapPointID getMapPointIDsGeneratedByFrame(const TpFrameID nFrameID) const;
private:
    Type::IDGenerator<TpMapPointID>     mMapPointIDGenerator;
    vector<TpMapPoint>                  mMapFromMapPointID2KptID;
    vector<TpMapPointID>                mMapFromKptID2MapPointID;
   //vector<> 
};

class TpKptIDMapPointPair{
public:
    TpKptIDMapPointPair(const TpKeyPointID nKptID, const TpKeyPointIndex nKptIndex,const TpMapPointID nMapPointID)
    : mKptID(nKptID), mKptIndex(nKptIndex), mMapPointID(nMapPointID){}
    TpKeyPointID    mKptID;
    TpKeyPointIndex mKptIndex;
    TpMapPointID    mMapPointID;
};

class TpKptIDMapPointPairWithFrameID: public TpKptIDMapPointPair {
public:
    TpKptIDMapPointPairWithFrameID(const TpFrameID nFrameID, const TpKeyPointID nKptID, const TpKeyPointIndex nKptIndex,const TpMapPointID nMapPointID):TpKptIDMapPointPair(nKptID, nKptIndex, nMapPointID), mFrameID(nFrameID){}
    
    TpFrameID       mFrameID;
};

class TpFrameKptIDMapPointPair{
public:
    TpFrameKptIDMapPointPair(const TpFrameID nFrameID):mFrameID(nFrameID), mVecFrameKptIDMapPointPair(){}
    
    inline void                         addKptIDMapPointPair(const TpKptIDMapPointPair& nKptIDMapPointPair){
                                            mVecFrameKptIDMapPointPair.push_back(nKptIDMapPointPair);
                                        }
                                
    inline vector<TpKeyPointID>         getKptIDs(void){
                                            TpVecKeyPointID nVecKeyPointID;
                                            nVecKeyPointID.reserve(mVecFrameKptIDMapPointPair.size());
                                            for(int nIdxPair=0,nSzPairs = mVecFrameKptIDMapPointPair.size();nIdxPair<nSzPairs;++nIdxPair){
                                                nVecKeyPointID.push_back(mVecFrameKptIDMapPointPair[nIdxPair].mKptID);
                                            }
                                            return nVecKeyPointID;
                                        }
    inline int                          size(void)const{return mVecFrameKptIDMapPointPair.size();}
    inline const TpKptIDMapPointPair&   getKptIDMapPointPair(const int nIndexPair)const{return mVecFrameKptIDMapPointPair[nIndexPair];}
    
    map<TpKeyPointID, TpMapPointID>     getMapKptID2MapPointID(void){
                                            map<TpKeyPointID, TpMapPointID> nMapKeyPointID2MapPointID;
                                            for(int nIdxPair=0,nSzPairs=size();nIdxPair<nSzPairs;++nIdxPair){
                                                const KeyFrameManager::TpKptIDMapPointPair& nPair = getKptIDMapPointPair(nIdxPair);
                                                nMapKeyPointID2MapPointID[nPair.mKptID] = nPair.mMapPointID;
                                            }
                                            return nMapKeyPointID2MapPointID;
                                        }
protected:
    TpFrameID                   mFrameID;
    vector<TpKptIDMapPointPair> mVecFrameKptIDMapPointPair;
};

   
class KeyFrameManager
{
public:
    
    void                    solve(Type::Frame& fFrame,const KeyPointManager::FrameMatchResult& mFrameMatchResult,
                                    KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
    
    void                    createOneKeyFrame(Type::Frame& fFrame,const KeyPointManager::FrameMatchResult& mFrameMatchResult,
                                    KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
    bool                    isKeyFrame(const TpFrameID nFrameID){
                                //if(mMapKFID2FrameID.empty())cout<<"empty kf"<<endl;else cout << "find kf"<<mMapKFID2FrameID[0]<<endl;
                                return std::count(mMapKFID2FrameID.begin(), mMapKFID2FrameID.end(), nFrameID);
                            }
    
    int                     countTrackKptIDsWithMapPointID(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
    
    void                    getKptIDsWithMapPointID(KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr,
                                                          TpFrameKptIDMapPointPair& nFrameKptIDMapPointPair);
    
    MapPointIDManager&      getMapPointIDManager(void){return mMapPointIDManager;}
    
    inline const TpFrameID  getFrameID(const TpKeyFrameID nKeyFrameID)const{return mMapKFID2FrameID[nKeyFrameID];}
    
    TpPtrCameraPose         getKeyFrameCameraPose(const TpKeyFrameID nKeyFrameID){
                                return mMapFrameID2CameraPose[getFrameID(nKeyFrameID)];
                            }
                            
    TpPtrCameraPose         getFrameCameraPose(const TpFrameID nFrameID){
                                if(nFrameID >= mMapFrameID2CameraPose.size()){
                                    cout << "Error: no Camera Pose is found for FrameID:" << nFrameID <<endl;
                                }
                                return mMapFrameID2CameraPose[nFrameID];
                            }
                            
    inline TpPtrCameraPose  generateOneFrameCameraPose(Type::Frame& fFrame){
                                assert(fFrame.FrameID() == (int)mMapFrameID2CameraPose.size());
                                TpPtrCameraPose nPtrCameraPose;
                                if(mMapFrameID2CameraPose.size()==0){
                                    nPtrCameraPose = std::make_shared<TpCameraPose>(cv::Matx44f::eye());
                                }else{
                                    nPtrCameraPose = std::make_shared<TpCameraPose>(*mMapFrameID2CameraPose.back());
                                }
                                // cout << "Genera one Camera Pose for FrameID:" << fFrame.FrameID() <<endl;
                                mMapFrameID2CameraPose.push_back(nPtrCameraPose);
                                return nPtrCameraPose;
                            }
protected:
    EnSLAMState             updateTrackState(int nCountSumTrackKpts, int nCountKptsOnThisFrame);
    
    inline TpKeyFrameID     generateOneKeyFrame(Type::Frame& fFrame){
                                TpKeyFrameID nKFID = mKeyFrameIDGenerator.create();
                                if(nKFID!=(int)mMapKFID2FrameID.size())
                                    throw;
                                mMapKFID2FrameID.push_back(fFrame.FrameID());
                                //cout << "add Kf" << nKFID << "|" << fFrame.FrameID() <<endl;
                                return nKFID;
                            }
                            
    void                    collectFirstDetectedKeyPointOnNonKeyFrame(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
    
private:
    MapPointIDManager                   mMapPointIDManager;
    Type::IDGenerator<TpKeyFrameID>     mKeyFrameIDGenerator;
    vector<TpFrameID>                   mMapKFID2FrameID;
    vector<TpPtrCameraPose>             mMapFrameID2CameraPose;
    
    list<TpKeyPointID>                  mLstFirstDetectedKptIDBeforKF;
private:
    DebugManager::DebugKeyFrameGenerationInfo   mDebugKeyFrameGenerationInfo;
    
};

typedef shared_ptr<KeyFrameManager> PtrKeyFrameManager;
    
}
}

#endif

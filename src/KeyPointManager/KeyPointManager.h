#ifndef __KEYPOINTMANAGER_H__
#define __KEYPOINTMANAGER_H__

#include "../Type/type.h"
#include "ORBextractor.h"
#include <memory>
#include <queue>

namespace PKVIO{
namespace KeyPointManager{
    
typedef enum{
    TpStillExisting,
    TpNotHere,
    TpRemoved,
    TpIsComing
} TpHistoryStorageState;

typedef struct{
   TpVecKeyPoints  mKeyPointsLeft; 
   TpVecDescriptor mDescriptorsLeft;
   TpVecKeyPoints  mKeyPointsRight; 
   TpVecDescriptor mDescriptorsRight;
   Type::TpFrameID mFrameID;
} TpOneFrameKptDescriptor;
    
class DescriptorHistory{
private:
   Type::FixLengthQueue<TpOneFrameKptDescriptor> mHistory;
public:
    DescriptorHistory():mHistory(20+5){}
    //operator= (DescriptorHistory& op){}

void push(TpOneFrameKptDescriptor& One){
    mHistory.push(One);
}

bool isExisting(const TpFrameID nFrmID){
    for(auto Iter = mHistory.begin(), EndIter = mHistory.end();Iter!=EndIter;++Iter){
        const TpOneFrameKptDescriptor& h = *Iter;
        if(h.mFrameID == nFrmID)
            return true;
    }
    return false;
    //for(const TpOneFrameKptDescriptor& h in mHistory){
    //    if(h.mFrameID == nFrmID)
    //        return true;
    //}
    //return false;
}
TpOneFrameKptDescriptor& get(const TpFrameID nFrmID){
    for(auto Iter = mHistory.begin(), EndIter = mHistory.end();Iter!=EndIter;++Iter){
        TpOneFrameKptDescriptor& h = *Iter;
        if(h.mFrameID == nFrmID)
            return h;
    }
    
    throw;
}
};
    
class KeyPointManager{
public:
    KeyPointManager(void);
    ~KeyPointManager();
    
    typedef std::shared_ptr<ORB_SLAM2::ORBextractor> TpFeatureExtractor;
    
    void                    extract(const Frame& f, TpOneFrameKptDescriptor& mKptsDescriptors);
    void                    solve(const Frame& f);
    
    inline bool             queryDescriptorExisting(const TpFrameID nFrmID){ return mDescriptorHistoryRecord.isExisting(nFrmID); }
    TpOneFrameKptDescriptor& getDescriptor(const TpFrameID nFrmID) { return mDescriptorHistoryRecord.get(nFrmID); }
private:
    inline void             initialize(void);
    
    void                    initializeFeatureExtroctor(void);
    void                    initializeFeatureMatcher(void);
    inline void             initializeHistoryRecord(void);
    
    
    TpFeatureExtractor      mPtrORBExtractorLeft;
    TpFeatureExtractor      mPtrORBExtractorRight;
    
    DescriptorHistory       mDescriptorHistoryRecord;
}; 

}
}

#endif

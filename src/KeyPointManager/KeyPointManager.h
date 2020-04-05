#ifndef __KEYPOINTMANAGER_H__
#define __KEYPOINTMANAGER_H__

#include <memory>
#include "../Type/type.h"
#include "ORBextractor.h"
#include "FrameKptsDescriptor.h"
#include "DescriptorMatch.h"
#include <iostream>

using namespace std;

namespace PKVIO{
namespace KeyPointManager{
    
class FrameMatchResult{
public:
    FrameMatchResult():mbInnerFrameDescriptorMatchResult(false){}
    void                                clear(void){mbInnerFrameDescriptorMatchResult=false;mVecDescriptorMatchResult.clear();}
    bool                                isExistInnerFrameDescriptorMatchResult(void) const { return mbInnerFrameDescriptorMatchResult;}
    void                                pushInnerFrameDescriptorMatchResult(TpDescriptorMatchResult& mInnerFrameDescMatchResult){
        mbInnerFrameDescriptorMatchResult = true;
        mVecDescriptorMatchResult.insert(mVecDescriptorMatchResult.begin(), mInnerFrameDescMatchResult);
    }
    const TpDescriptorMatchResult&      getInnerFrameDescriptorMatchResult(void) const {
        if(!mbInnerFrameDescriptorMatchResult)
            throw;
        return mVecDescriptorMatchResult[0];
    }
    
    inline int                              size(void) const {return (int)mVecDescriptorMatchResult.size();}
    inline const TpDescriptorMatchResult&   getFrameDescriptoreMatchResult(const int nIndex) const { return mVecDescriptorMatchResult[nIndex]; }
    inline const int                        sizeOuterFrameDescriptorMatchResult(void) const {return isExistInnerFrameDescriptorMatchResult()?size()-1:size();}
    inline void                             pushOuterFrameDescriptorMatchResult(TpDescriptorMatchResult& mOuterFrameDescMatchResult){mVecDescriptorMatchResult.push_back(mOuterFrameDescMatchResult);}
    inline const TpDescriptorMatchResult&   getOuterFrameDescriptorMatchResult(const int nOuterIndex) const {
        return mVecDescriptorMatchResult[mbInnerFrameDescriptorMatchResult?nOuterIndex+1:nOuterIndex];
    }
    
    inline const int                        getCountKptsOnThisFrame(void) const {
        if(isExistInnerFrameDescriptorMatchResult()){
            return getInnerFrameDescriptorMatchResult().getCountNonDuplicateKpts();
        }else{
            cout << "Mono. Not Support Now. exit" <<endl;
            throw;
        }
        
    }
private:
    bool                                    mbInnerFrameDescriptorMatchResult;
    std::vector<TpDescriptorMatchResult>    mVecDescriptorMatchResult;
};
    
class KeyPointManager{
public:
    KeyPointManager(void);
    ~KeyPointManager();
    
    typedef std::shared_ptr<ORB_SLAM2::ORBextractor> TpFeatureExtractor;
    
    void                        extract(const Frame& f, TpOneFrameKptDescriptor& mKptsDescriptors);
    FrameMatchResult&           solve(const Frame& f);
    
    inline bool                 queryDescriptorExisting(const TpFrameID nFrmID){ return mFrameKptsDescriptorHistoryRecord.isExisting(nFrmID); }
    TpOneFrameKptDescriptor&    getDescriptor(const TpFrameID nFrmID) { return mFrameKptsDescriptorHistoryRecord.get(nFrmID); }
    
    inline FrameMatchResult&    getFrameMatchResult(void){return mFrameMatchResult;}
    
    inline StereoFrameHistory&  getStereoFrameHistory(void){return mFrameHistoryRecord;}
    
    inline FrameKptsDescriptorHistory& getFrameKptsDescriptorHistory(void){return mFrameKptsDescriptorHistoryRecord;}
protected:
    void                        track(const Frame& fCurFrame, const TpOneFrameKptDescriptor& fCurFrameKptDescriptor);
    
    // for what?
    const Frame&                getLastFrame(void){ return mFrameHistoryRecord.back(); }
private:
    inline void                 initialize(void);
    
    void                        initializeFeatureExtroctor(void);
    void                        initializeFeatureMatcher(void);
    inline void                 initializeHistoryRecord(void);
    
    
    TpFeatureExtractor          mPtrORBExtractorLeft;
    TpFeatureExtractor          mPtrORBExtractorRight;
    
    PtrDescriptorMatch          mPtrDesciptorMatcher;
    
    FrameKptsDescriptorHistory  mFrameKptsDescriptorHistoryRecord;
    StereoFrameHistory          mFrameHistoryRecord;
    
    FrameMatchResult            mFrameMatchResult;
}; 

}
}

#endif

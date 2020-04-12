#ifndef __KEYPOINTMANAGER_H__
#define __KEYPOINTMANAGER_H__

#include <memory>
#include "../Type/type.h"
#include "ORBextractor.h"
#include "FrameKptsDescriptor.h"
#include "DescriptorMatch.h"
#include <iostream>
#include "../DebugManager/DebugManager.h"

using namespace std;

namespace PKVIO{
namespace KeyPointManager{
    
   
class KeyPointManager{
public:
    KeyPointManager(void);
    ~KeyPointManager();
    
    void                        setSimulator(bool bUseSimulator, TpPtrCameraStereo nPtrCameraStereo);
    
    typedef std::shared_ptr<ORB_SLAM2::ORBextractor> TpFeatureExtractor;
    
    void                        extract(const Frame& f, TpOneFrameKptDescriptor& mKptsDescriptors);
    FrameMatchResult&           solve(const Frame& f);
    
    inline bool                 queryDescriptorExisting(const TpFrameID nFrmID){ return mFrameKptsDescriptorHistoryRecord.isExisting(nFrmID); }
    TpOneFrameKptDescriptor&    getDescriptor(const TpFrameID nFrmID) { return mFrameKptsDescriptorHistoryRecord.get(nFrmID); }
    
    inline FrameMatchResult&    getFrameMatchResult(void){return mFrameMatchResult;}
    
    inline StereoFrameHistory&  getStereoFrameHistory(void){return mFrameHistoryRecord;}
    
    inline FrameKptsDescriptorHistory& getFrameKptsDescriptorHistory(void){return mFrameKptsDescriptorHistoryRecord;}

    cv::Mat                     showMatchResult(const TpOneFrameKptDescriptor& fKptsDesc, const TpDescriptorMatchResult& mBestVecMatchResult, const string sWindowTitle = "mBestMatch");
    
    
    cv::Mat                     showMatchResult(const TpFrameID nFrameIDStereo);
    cv::Mat                     showMatchResult(const TpFrameID nFrameIDMaster,const TpFrameID nFrameIDSlaver, bool bShow);
    
    bool                        getTrackingKptDescriptorMatchResult(const TpFrameID nFrameIDMaster,const TpFrameID nFrameIDSlaver, TpOneFrameKptDescriptor& fKptsDesc, TpDescriptorMatchResult& nMatchResult);
protected:
    void                        track(const Frame& fCurFrame, TpOneFrameKptDescriptor& nKptsDescriptors,
                                      TpDescriptorMatchResult& nInnerMatchResult, vector<TpDescriptorMatchResult>& nOuterMatchResult);
    
    void                        trackBySimulator(const Frame& fCurFrame, TpOneFrameKptDescriptor& nKptsDescriptors,
                                      TpDescriptorMatchResult& nInnerMatchResult, vector<TpDescriptorMatchResult>& nOuterMatchResult);
    
    void                        track(const Frame& fCurFrame, const TpOneFrameKptDescriptor& fCurFrameKptDescriptor,
                                vector<TpDescriptorMatchResult>& nOuterMatchResult);
    
    // for what?
    const Frame&                getLastFrame(void){ return mFrameHistoryRecord.back(); }
    
    StereoFrame                 constructTrackFrame(const Frame& fFramePrev, const Frame& fFrameCur);
    TpOneFrameKptDescriptor     constructTrackingKptDescriptor(const TpOneFrameKptDescriptor& fFrameKptDescriptorPrev, const TpOneFrameKptDescriptor& fFrameKptDescriptorCur);
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
    FrameMatchResultHistory     mFrameMatchResultHistoryRecord;
    
    DebugManager::DebugKeyPointTrackingInfo mDebugKeyPointTrackingInfo;
    
    bool                        mBoolUseSimulator;
    TpPtrCameraStereo           mPtrCameraStereo;
}; 

}
}

#endif

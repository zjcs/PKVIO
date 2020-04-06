#include "KeyPointManager.h"
#include <iostream>
#include "../Tools/Tools.h"


namespace PKVIO{
namespace KeyPointManager{

KeyPointManager::KeyPointManager() {
    initialize();
}

KeyPointManager::~KeyPointManager() {}


void KeyPointManager::initialize(void) {
    initializeFeatureExtroctor();
    initializeFeatureMatcher();
    initializeHistoryRecord();
}

FrameMatchResult& KeyPointManager::solve ( const Type::Frame& f )
{
    auto FuncLogDebugKeyPointTrackingInfo = [&](){
        mDebugKeyPointTrackingInfo.mFrameID = f.FrameID();
        mDebugKeyPointTrackingInfo.log();
    };
    
    mDebugKeyPointTrackingInfo = DebugManager::DebugKeyPointTrackingInfo();
    Tools::Timer tTimer(FuncLogDebugKeyPointTrackingInfo, "KeyPoint extract and match", false, &mDebugKeyPointTrackingInfo.mTimeCostWhole);
    mFrameMatchResult.clear();
        
    // extract;
    TpOneFrameKptDescriptor mKptsDescriptors;
    extract(f, mKptsDescriptors);
    
    // match;
    TpDescriptorMatchResult mMatchResult = mPtrDesciptorMatcher->match(mKptsDescriptors);
    
    mPtrDesciptorMatcher->showMatchResult(f, mKptsDescriptors, mMatchResult, "Left | Right - Match Result");
    //assert(mPtrDesciptorMatcher->debugDuplicatedMatch(f, mKptsDescriptors, mMatchResult));
    mFrameMatchResult.pushInnerFrameDescriptorMatchResult(mMatchResult);
    
    // track from previous frames.
    track(f, mKptsDescriptors);
    
    // TODO : Implement EXCORP function, and move this operation before track.
    // History Record;
    mFrameKptsDescriptorHistoryRecord.push(mKptsDescriptors);
    const StereoFrame& fStereoFrame = dynamic_cast<const StereoFrame&>(f);
    mFrameHistoryRecord.push(fStereoFrame);
    
    mFrameMatchResultHistoryRecord.push(getFrameMatchResult());
    return getFrameMatchResult();
}

void KeyPointManager::extract ( const Type::Frame& f , TpOneFrameKptDescriptor& mKptsDescriptors) 
{    
    //Tools::Timer tTimer("KeyPoint extract");
    if(f.type()!= Type::TpFrame::TpStereo){
        cout << "Error: Frame type donot match, exit."<<endl;
        throw;
    }
    
    const Type::StereoFrame& sf = dynamic_cast<const Type::StereoFrame&>(f);
    //auto& mImgLeft = sf.
    auto& mImgLeft  = sf.getImageLeft();
    auto& mImgRight = sf.getImageRight();
    
    //int nRows = mImgLeft.rows, nCols = mImgLeft.cols;
    
    mKptsDescriptors.mFrameIDLeft   = f.FrameID();
    mKptsDescriptors.mFrameIDRight  = f.FrameID();
    (*mPtrORBExtractorLeft)(mImgLeft, cv::Mat(), mKptsDescriptors.mKeyPointsLeft, mKptsDescriptors.mDescriptorsLeft);
    (*mPtrORBExtractorRight)(mImgRight, cv::Mat(), mKptsDescriptors.mKeyPointsRight, mKptsDescriptors.mDescriptorsRight);
}



void KeyPointManager::initializeFeatureExtroctor()
{
//    int nFeatures = fSettings["ORBextractor.nFeatures"];
//    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
//    int nLevels = fSettings["ORBextractor.nLevels"];
//    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
//    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    //int nFeatures = 1200;
    int nFeatures = 500;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = 20;
    int fMinThFAST = 7;

    mPtrORBExtractorLeft = std::make_shared<ORB_SLAM2::ORBextractor>(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    mPtrORBExtractorRight = std::make_shared<ORB_SLAM2::ORBextractor>(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}


void KeyPointManager::initializeFeatureMatcher()
{
    // true: show the match result.
    mPtrDesciptorMatcher = std::make_shared<DescriptorMatch>();
}


void KeyPointManager::initializeHistoryRecord()
{
    //mFrameKptsDescriptorHistoryRecord = DescriptorHistory();
}



void KeyPointManager::track(const Type::Frame& fCurFrame, const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fCurFrameKptDescriptor) {
    //Tools::Timer tTimer("KeyPoint track");
    
    if(mFrameKptsDescriptorHistoryRecord.empty() || mFrameHistoryRecord.empty())
        return;
    
    TpOneFrameKptDescriptor&    fLastFrameKptDescriptor = mFrameKptsDescriptorHistoryRecord.back();
    Frame&                      fLastFrame              = mFrameHistoryRecord.back();
    
    const StereoFrame& fCurStereoFrame              = dynamic_cast<const StereoFrame&>(fCurFrame);
    const StereoFrame& fLastStereoFrame             = dynamic_cast<const StereoFrame&>(fLastFrame);
    
    StereoFrame                 mPrevLeftAndCurLeftFrame = constructTrackFrame(fLastStereoFrame, fCurStereoFrame);
    
    TpOneFrameKptDescriptor     mPrevAndCurLeftKptsDescriptor;
    mPrevAndCurLeftKptsDescriptor.mFrameIDLeft      = fLastStereoFrame.FrameID();
    mPrevAndCurLeftKptsDescriptor.mFrameIDRight     = fCurStereoFrame.FrameID();
    mPrevAndCurLeftKptsDescriptor.mDescriptorsLeft  = fLastFrameKptDescriptor.mDescriptorsLeft;
    mPrevAndCurLeftKptsDescriptor.mKeyPointsLeft    = fLastFrameKptDescriptor.mKeyPointsLeft;
    mPrevAndCurLeftKptsDescriptor.mDescriptorsRight = fCurFrameKptDescriptor.mDescriptorsLeft;
    mPrevAndCurLeftKptsDescriptor.mKeyPointsRight   = fCurFrameKptDescriptor.mKeyPointsLeft;
    
    // Pre-Cur is about 60%, 300/500 match and a little better than left-right's 50%, and all is right match.
    TpDescriptorMatchResult mPrevAndCurMatchResult = mPtrDesciptorMatcher->match(mPrevAndCurLeftKptsDescriptor);
        
    mFrameMatchResult.pushOuterFrameDescriptorMatchResult(mPrevAndCurMatchResult);
    
    
    //AutoLogTimer("KeyPoint show match");
    mPtrDesciptorMatcher->showMatchResult(mPrevLeftAndCurLeftFrame, mPrevAndCurLeftKptsDescriptor, mPrevAndCurMatchResult, "PrevLeft | CurLeft - Match Result");
    //assert(mPtrDesciptorMatcher->debugDuplicatedMatch(f, mKptsDescriptors, mMatchResult));
}


Type::StereoFrame KeyPointManager::constructTrackFrame(const Type::Frame& fFramePrev, const Type::Frame& fFrameCur) {
    StereoFrame                 mPrevLeftAndCurLeftFrame;
    mPrevLeftAndCurLeftFrame.initFrameID(fFramePrev.FrameID());
    mPrevLeftAndCurLeftFrame.ImageLeft()         = fFramePrev.getImage();    // Prev-Mono or Prev-Stereo-Left;
    mPrevLeftAndCurLeftFrame.ImageRight()        = fFrameCur.getImage();     // Prev-Mono or Cur-Stereo-Left;
    return mPrevLeftAndCurLeftFrame;
}


cv::Mat KeyPointManager::showMatchResult(const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc, const TpDescriptorMatchResult& mBestVecMatchResult, const string sWindowTitle)
{
    const TpFrameID nFrameIDMaster = fKptsDesc.FrameIDLeft(),nFrameIDSlaver = fKptsDesc.FrameIDRight();
    if(nFrameIDMaster == nFrameIDSlaver) {
        //
        const TpFrameID nStereoFrameID =  nFrameIDMaster;
        if(mFrameHistoryRecord.isExisting(nStereoFrameID)) {
            StereoFrame& pStereoFrame = dynamic_cast<StereoFrame&>(mFrameHistoryRecord.get(nStereoFrameID));
            return mPtrDesciptorMatcher->showMatchResult(pStereoFrame, fKptsDesc, mBestVecMatchResult, sWindowTitle);
        } else {
            cout << "Not implemention to read frame by id."<<endl;
            throw;
            return cv::Mat();
        }
    } else {
        if(mFrameHistoryRecord.isExisting(nFrameIDMaster)&&mFrameHistoryRecord.isExisting(nFrameIDSlaver)) {
            StereoFrame fStereoFrame = constructTrackFrame(mFrameHistoryRecord.get(nFrameIDMaster), mFrameHistoryRecord.get(nFrameIDSlaver));
            return mPtrDesciptorMatcher->showMatchResult(fStereoFrame, fKptsDesc, mBestVecMatchResult, sWindowTitle);
        }
    }
    return cv::Mat();
}

cv::Mat KeyPointManager::showMatchResult(const Type::TpFrameID nFrameIDStereo) {
    // show left-right match result;
    
    TpOneFrameKptDescriptor nFrameKptDescriptorStereo;
    TpDescriptorMatchResult nDescriptorsMatchResult;
    if(!getTrackingKptDescriptorMatchResult(nFrameIDStereo,nFrameIDStereo, nFrameKptDescriptorStereo, nDescriptorsMatchResult))
        return cv::Mat();
    return showMatchResult(nFrameKptDescriptorStereo, nDescriptorsMatchResult, "Match:L|R-"+Type::toString(nFrameIDStereo));
}

cv::Mat KeyPointManager::showMatchResult(const Type::TpFrameID nFrameIDMaster, const Type::TpFrameID nFrameIDSlaver, bool bShow) {
    TpOneFrameKptDescriptor nFrameKptDescriptorPrevCur;
    TpDescriptorMatchResult nDescriptorsMatchResult;
    getTrackingKptDescriptorMatchResult(nFrameIDMaster, nFrameIDSlaver,nFrameKptDescriptorPrevCur, nDescriptorsMatchResult);
    return showMatchResult(nFrameKptDescriptorPrevCur, nDescriptorsMatchResult, "Match:M|S-"+Type::toString(nFrameIDMaster)+"|"+toString(nFrameIDSlaver));
}

bool KeyPointManager::getTrackingKptDescriptorMatchResult(const Type::TpFrameID nFrameIDMaster, const Type::TpFrameID nFrameIDSlaver, PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc, TpDescriptorMatchResult& nMatchResult) 
{
     // draw the match and debug.
    if(nFrameIDMaster == nFrameIDSlaver){
        const TpFrameID nFrameIDStereo = nFrameIDMaster;
        if(!mFrameKptsDescriptorHistoryRecord.isExisting(nFrameIDStereo))
            return false;
    
        if(!mFrameMatchResultHistoryRecord.isExisting(nFrameIDStereo))
            return false; 
        FrameMatchResult& nFrameMatchResultStereo = mFrameMatchResultHistoryRecord.get(nFrameIDStereo);
        if(!nFrameMatchResultStereo.isExistInnerFrameDescriptorMatchResult())
            return false;
        
        // kptdesc
        fKptsDesc = mFrameKptsDescriptorHistoryRecord.get(nFrameIDStereo);
        
        // match result
        nMatchResult = nFrameMatchResultStereo.getInnerFrameDescriptorMatchResult();
    }else{
        if(!mFrameKptsDescriptorHistoryRecord.isExisting(nFrameIDMaster)||!mFrameHistoryRecord.isExisting(nFrameIDSlaver))
            return false;
        
        if(!mFrameMatchResultHistoryRecord.isExisting(nFrameIDSlaver))
            return false;
        FrameMatchResult& mFrameMatchResultSlaver = mFrameMatchResultHistoryRecord.get(nFrameIDSlaver);
        int nOuterIndex = mFrameMatchResultSlaver.getOuterIndex(nFrameIDMaster);
        if(nOuterIndex == -1)
            return false;
        
        TpOneFrameKptDescriptor& nFrameKptDescriptorMaster = mFrameKptsDescriptorHistoryRecord.get(nFrameIDMaster);
        TpOneFrameKptDescriptor& nFrameKptDescriptorSlaver = mFrameKptsDescriptorHistoryRecord.get(nFrameIDSlaver);
        
        // kptdesc
        fKptsDesc = constructTrackingKptDescriptor(nFrameKptDescriptorMaster, nFrameKptDescriptorSlaver);
        
        // match
        nMatchResult = mFrameMatchResultSlaver.getOuterFrameDescriptorMatchResult(nOuterIndex);        
    }
    return true;
}

PKVIO::KeyPointManager::TpOneFrameKptDescriptor KeyPointManager::constructTrackingKptDescriptor(const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fFrameKptDescriptorPrev, const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fFrameKptDescriptorCur) 
{
    TpOneFrameKptDescriptor nPrevCurKptDescriptor;
    
    nPrevCurKptDescriptor.mFrameIDLeft      = fFrameKptDescriptorPrev.FrameIDLeft();
    nPrevCurKptDescriptor.mFrameIDRight     = fFrameKptDescriptorCur.FrameIDLeft();
    nPrevCurKptDescriptor.mKeyPointsLeft    = fFrameKptDescriptorPrev.mKeyPointsLeft;
    nPrevCurKptDescriptor.mKeyPointsRight   = fFrameKptDescriptorCur.mKeyPointsLeft;
    
    nPrevCurKptDescriptor.mDescriptorsLeft  = fFrameKptDescriptorPrev.mDescriptorsLeft;
    nPrevCurKptDescriptor.mDescriptorsRight = fFrameKptDescriptorCur.mDescriptorsLeft;
    
    return nPrevCurKptDescriptor;
}

}
}

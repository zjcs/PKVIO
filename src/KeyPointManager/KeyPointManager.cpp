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
   //AutoLogTimer("KeyPoint extract and match");
   mFrameMatchResult.clear();
    
    // extract;
    TpOneFrameKptDescriptor mKptsDescriptors;
    extract(f, mKptsDescriptors);
    
    // match;
    TpDescriptorMatchResult mMatchResult = mPtrDesciptorMatcher->match(mKptsDescriptors);
    mPtrDesciptorMatcher->showMatchResult(f, mKptsDescriptors, mMatchResult, "Left | Right - Match Result");
    mFrameMatchResult.pushInnerFrameDescriptorMatchResult(mMatchResult);
    
    // track from previous frames.
    track(f, mKptsDescriptors);
    
    // TODO : Implement EXCORP function, and move this operation before track.
    // History Record;
    mFrameKptsDescriptorHistoryRecord.push(mKptsDescriptors);
    const StereoFrame& fStereoFrame = dynamic_cast<const StereoFrame&>(f);
    mFrameHistoryRecord.push(fStereoFrame);
    
    return getFrameMatchResult();
}

void KeyPointManager::extract ( const Type::Frame& f , TpOneFrameKptDescriptor& mKptsDescriptors) 
{    
    //AutoLogTimer("KeyPoint extract");
    if(f.type()!= Type::TpFrame::TpStereo){
        cout << "Error: Frame type donot match, exit."<<endl;
        throw;
    }
    
    const Type::StereoFrame& sf = dynamic_cast<const Type::StereoFrame&>(f);
    //auto& mImgLeft = sf.
    auto& mImgLeft  = sf.ImageLeft();
    auto& mImgRight = sf.ImageRight();
    
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
    //AutoLogTimer("KeyPoint track");
    
    if(mFrameKptsDescriptorHistoryRecord.empty() || mFrameHistoryRecord.empty())
        return;
    
    TpOneFrameKptDescriptor&    fLastFrameKptDescriptor = mFrameKptsDescriptorHistoryRecord.back();
    Frame&                      fLastFrame              = mFrameHistoryRecord.back();
    
    const StereoFrame& fCurStereoFrame              = dynamic_cast<const StereoFrame&>(fCurFrame);
    const StereoFrame& fLastStereoFrame             = dynamic_cast<const StereoFrame&>(fLastFrame);
    
    StereoFrame                 mPrevLeftAndCurLeftFrame;
    mPrevLeftAndCurLeftFrame.initFrameID(fLastStereoFrame.FrameID());
    mPrevLeftAndCurLeftFrame.getImageLeft()         = fLastStereoFrame.ImageLeft();
    mPrevLeftAndCurLeftFrame.getImageRight()        = fCurStereoFrame.ImageLeft();
    
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
}

}
}

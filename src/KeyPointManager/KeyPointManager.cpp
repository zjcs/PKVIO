#include "KeyPointManager.h"
#include <iostream>

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

void KeyPointManager::solve ( const Type::Frame& f )
{
    // extract;
    TpOneFrameKptDescriptor mKptsDescriptors;
    extract(f, mKptsDescriptors);
    
    // match;
    TpDescriptorMatchResult mMatchResult = mPtrDesciptorMatcher->match(f, mKptsDescriptors);
    
    // History Record;
    mDescriptorHistoryRecord.push(mKptsDescriptors);
}

void KeyPointManager::extract ( const Type::Frame& f , TpOneFrameKptDescriptor& mKptsDescriptors) 
{    
    if(f.type()!= Type::TpFrame::TpStereo){
        cout << "Error: Frame type donot match, exit."<<endl;
        throw;
    }
    
    const Type::StereoFrame& sf = dynamic_cast<const Type::StereoFrame&>(f);
    //auto& mImgLeft = sf.
    auto& mImgLeft = sf.ImageLeft();
    auto& mImgRight = sf.ImageRight();
    
    //int nRows = mImgLeft.rows, nCols = mImgLeft.cols;
    
    mKptsDescriptors.mFrameID = f.FrameID();
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
    mPtrDesciptorMatcher = std::make_shared<DescriptorMatch>(true);
}


void KeyPointManager::initializeHistoryRecord()
{
    //mDescriptorHistoryRecord = DescriptorHistory();
}


}
}

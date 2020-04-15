#include "KeyPointManager.h"
#include <iostream>
#include "../Tools/Tools.h"


namespace PKVIO{
namespace KeyPointManager{

KeyPointManager::KeyPointManager() 
: mBoolUseSimulator(false)
{
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
    
    TpOneFrameKptDescriptor nKptsDescriptors;
    TpDescriptorMatchResult mInnerMatchResult;
    vector<TpDescriptorMatchResult> nOuterMatchResult;
    if(mBoolUseSimulator){
        trackBySimulator(f, nKptsDescriptors, mInnerMatchResult, nOuterMatchResult);
    }else{
        track(f, nKptsDescriptors, mInnerMatchResult, nOuterMatchResult);
    }
    
    //mPtrDesciptorMatcher->showMatchResult(f, mKptsDescriptors, mMatchResult, "Left | Right - Match Result");
    //assert(mPtrDesciptorMatcher->debugDuplicatedMatch(f, mKptsDescriptors, mMatchResult));
    mFrameMatchResult.pushInnerFrameDescriptorMatchResult(mInnerMatchResult);
    for(size_t nIdxOuterMatch=0,nSzOuterMatch=nOuterMatchResult.size();nIdxOuterMatch<nSzOuterMatch;++nIdxOuterMatch){
        mFrameMatchResult.pushOuterFrameDescriptorMatchResult(nOuterMatchResult[nIdxOuterMatch]);
    }
    
    // TODO : Implement EXCORP function, and move this operation before track.
    // History Record;
    mFrameKptsDescriptorHistoryRecord.push(nKptsDescriptors);
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



void KeyPointManager::track(const Type::Frame& fCurFrame, const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fCurFrameKptDescriptor, 
    vector<TpDescriptorMatchResult>& nOuterMatchResult) 
{
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
        
    //mFrameMatchResult.pushOuterFrameDescriptorMatchResult(mPrevAndCurMatchResult);
    nOuterMatchResult.push_back(mPrevAndCurMatchResult);
    
    
    //AutoLogTimer("KeyPoint show match");
    //mPtrDesciptorMatcher->showMatchResult(mPrevLeftAndCurLeftFrame, mPrevAndCurLeftKptsDescriptor, mPrevAndCurMatchResult, "PrevLeft | CurLeft - Match Result");
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
    return showMatchResult(nFrameKptDescriptorStereo, nDescriptorsMatchResult, "Match:L|R-"+Type::cvtToString(nFrameIDStereo));
}

cv::Mat KeyPointManager::showMatchResult(const Type::TpFrameID nFrameIDMaster, const Type::TpFrameID nFrameIDSlaver, bool bShow) {
    TpOneFrameKptDescriptor nFrameKptDescriptorPrevCur;
    TpDescriptorMatchResult nDescriptorsMatchResult;
    getTrackingKptDescriptorMatchResult(nFrameIDMaster, nFrameIDSlaver,nFrameKptDescriptorPrevCur, nDescriptorsMatchResult);
    return showMatchResult(nFrameKptDescriptorPrevCur, nDescriptorsMatchResult, "Match:M|S-"+Type::cvtToString(nFrameIDMaster)+"|"+cvtToString(nFrameIDSlaver));
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


void KeyPointManager::track(const Type::Frame& f,TpOneFrameKptDescriptor& nKptsDescriptors,
                            TpDescriptorMatchResult& nInnerMatchResult
, std::vector< PKVIO::KeyPointManager::TpDescriptorMatchResult >& nOuterMatchResult) 
{
    // extract;
    extract(f, nKptsDescriptors);
    
    // match;
    nInnerMatchResult = mPtrDesciptorMatcher->match(nKptsDescriptors);
    
    // track from previous frames.
    track(f, nKptsDescriptors, nOuterMatchResult);
}


void KeyPointManager::trackBySimulator(const Type::Frame& fCurFrame, PKVIO::KeyPointManager::TpOneFrameKptDescriptor& nKptsDescriptors,
    TpDescriptorMatchResult& nInnerMatchResult, std::vector< PKVIO::KeyPointManager::TpDescriptorMatchResult >& nOuterMatchResult) 
{
    std::map< Type::TpFrameID, TpPtrCameraPose> nMapFrameID2CameraPose;
    std::map<Type::TpMapPointID, Type::TpPtrMapPoint3D > nMapMapPointID2MapPoint3D;
    //TpVecVisualMeasurement nVecVisualMeasurement;
    //TpPtrCameraStereo nPtrCameraStereo;
    
    {
        typedef cv::Vec3d TpDataPt;
        ++nSzIteration;
        std::vector<cv::Vec3d> nTranslation = {TpDataPt(0,0,0), TpDataPt(100,0,0), TpDataPt(100, 100, 0), TpDataPt(0, 100, 0)};
        std::vector<TpPtrCameraPose> nTMatrix;
        for(int nIdx=0,nSz=nTranslation.size();nIdx<nSz;++nIdx){
            for(int nIdxSplit=0,nSzSplit=DebugManager::DebugControl().mCountSegment;nIdxSplit<nSzSplit;++nIdxSplit){
                auto nTrans = nTranslation[nIdx] + (nTranslation[(nIdx+1)%nSz]-nTranslation[nIdx])*(nIdxSplit*1.0/nSzSplit);
                //nTranslation[nIdx];
                cv::Matx44f nCamera = Type::cvtR33T31ToMatx44f(cv::Mat(cv::Matx33f::eye()), cv::Mat(cv::Matx31f(nTrans)));
                nTMatrix.push_back(std::make_shared<TpCameraPose>(TpCameraPose(nCamera)));
            }
        }
        
        std::vector<cv::Vec3d> nVecMapPt3D = DebugManager::getVirtualPointInSimulator();
        int nSzMp=nVecMapPt3D.size();
        
        nMapMapPointID2MapPoint3D.clear();
        //nVecVisualMeasurement.clear();
        nMapFrameID2CameraPose.clear();
        for(int nIdxMp=0;nIdxMp<nSzMp;++nIdxMp)
            nMapMapPointID2MapPoint3D[nIdxMp] = std::make_shared<cv::Point3f>(cv::Point3f(nVecMapPt3D[nIdxMp]));
        
        auto FuncGenerateMeasure = [&](int nIdxIter, const Camera& nCamera, std::vector<cv::KeyPoint>& nVecKpt){
            cv::Mat nInnerMat = nCamera.getInnerParam().getInnerMat64F();
            cv::Mat nDistorMat = nCamera.getInnerParam().getDistorParam64F();
            
            cv::Matx44f nTpt = nTMatrix[nIdxIter%(int(nTMatrix.size()))]->getMatx44f().inv();
            cv::Matx44f ncTw = nCamera.getCameraOuterParam().get()*nTpt;
            //cv::Mat rvec, tvec;
            //cout << "nT:" <<endl << ncTw<<endl;
            //cv::Rodrigues(cv::Mat(ncTw), rvec, tvec);
            //std::vector<cv::Vec2d> nVecVm; cv::Mat nMatVm;
            //cv::projectPoints(cv::Mat(nVecMapPt3D), (rvec), (tvec), nInnerMat, nDistorMat, nMatVm);
            for(int nIdx=0,nSz = nVecMapPt3D.size();nIdx<nSz;++nIdx){
                cv::Vec3f nPtCam3D  = Type::project(ncTw, cv::Vec3f(nVecMapPt3D[nIdx]));
                cv::Vec3f nPtCam = nPtCam3D/nPtCam3D(2);
                cv::Vec3f nPtProj = cv::Matx33f(nInnerMat)*nPtCam;
                nVecKpt.push_back(cv::KeyPoint(cv::Point2f(nPtProj(0),nPtProj(1)), 1));
                
                //cout << "Kpt: World|Came|Pixel-"<<nVecMapPt3D[nIdx]<<"|"<<nPtCam3D<<"|"<<nPtProj<<endl;
            }
        };
        
        auto FuncGenerateMeasureStereo = [&](int nIdxIter, std::vector<cv::KeyPoint>& nVecVmLeft, std::vector<cv::KeyPoint>& nVecVmRight){
            //cout << "Simulate - Left View:"<<endl;
            FuncGenerateMeasure(nIdxIter, mPtrCameraStereo->CameraLeft(), nVecVmLeft);
            //cout << "Simulate - RightView:"<<endl;
            FuncGenerateMeasure(nIdxIter, mPtrCameraStereo->CameraRight(), nVecVmRight);
            
        };
        
        nKptsDescriptors.mFrameIDLeft = nKptsDescriptors.mFrameIDRight = fCurFrame.FrameID();
        FuncGenerateMeasureStereo(fCurFrame.FrameID(), nKptsDescriptors.mKeyPointsLeft, nKptsDescriptors.mKeyPointsRight);
        nKptsDescriptors.mDescriptorsLeft.resize(nKptsDescriptors.mKeyPointsLeft.size());
        nKptsDescriptors.mDescriptorsRight.resize(nKptsDescriptors.mKeyPointsRight.size());
        
        TpVecMatchResult nMatch;
        for(int nIdxMp=0;nIdxMp<nSzMp;++nIdxMp){
            nMatch.push_back(TpOneMatchResult(nIdxMp,nIdxMp, 0));
        }
        nInnerMatchResult = TpDescriptorMatchResult(fCurFrame.FrameID(),fCurFrame.FrameID(), nMatch, nSzMp, nSzMp);
        if(fCurFrame.FrameID()>0){
            nOuterMatchResult.push_back(TpDescriptorMatchResult(fCurFrame.FrameID()-1,fCurFrame.FrameID(), nMatch, nSzMp, nSzMp));
        }
    }   
  
}


void KeyPointManager::setSimulator(bool bUseSimulator, Type::TpPtrCameraStereo nPtrCameraStereo) {
    mBoolUseSimulator = bUseSimulator;
    if(mBoolUseSimulator){
        mPtrCameraStereo = nPtrCameraStereo;
        cout << endl << "Info: Using bUseSimulator ..." << endl << endl;
    }
}

}
}

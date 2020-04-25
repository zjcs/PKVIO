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
    
    TpOneFrameKptDescriptor nKptsDescriptors;
    TpDescriptorMatchResult mInnerMatchResult;
    vector<TpDescriptorMatchResult> nOuterMatchResult;
    if(mBoolUseSimulator){
        trackBySimulator(f, nKptsDescriptors, mInnerMatchResult, nOuterMatchResult);
    }else{
        if(DebugManager::DebugControl().mBoolTrackByOpticalFlow){
            trackByOpticalFlow(f, nKptsDescriptors, mInnerMatchResult, nOuterMatchResult, mFrameMatchResult);
        }else{
            track(f, nKptsDescriptors, mInnerMatchResult, nOuterMatchResult);
        }
    }
    mFrameKptsDescriptorHistoryRecord.push(nKptsDescriptors);
    const StereoFrame& fStereoFrame = dynamic_cast<const StereoFrame&>(f);
    mFrameHistoryRecord.push(fStereoFrame);
    
    if(DebugManager::DebugControl().mBoolShowMatchResult)
        mPtrDesciptorMatcher->showMatchResult(f, nKptsDescriptors, mInnerMatchResult, "Left | Right - Match Result");
    //assert(mPtrDesciptorMatcher->debugDuplicatedMatch(f, mKptsDescriptors, mMatchResult));
    mFrameMatchResult.clear();
    mFrameMatchResult.pushInnerFrameDescriptorMatchResult(mInnerMatchResult);
    for(size_t nIdxOuterMatch=0,nSzOuterMatch=nOuterMatchResult.size();nIdxOuterMatch<nSzOuterMatch;++nIdxOuterMatch){
        auto& nMatchResult = nOuterMatchResult[nIdxOuterMatch];
        mFrameMatchResult.pushOuterFrameDescriptorMatchResult(nOuterMatchResult[nIdxOuterMatch]);
        
        if(DebugManager::DebugControl().mBoolShowMatchResult){
           //mPtrDesciptorMatcher->showMatchResult(f, nKptsDescriptors, mInnerMatchResult, "Left | Right - Match Result");
            const StereoFrame& fCurStereoFrame      = dynamic_cast<const StereoFrame&>(f);
            const StereoFrame& fLastStereoFrame     = dynamic_cast<const StereoFrame&>(mFrameHistoryRecord.get(nMatchResult.getFrameIDLeft()));
            StereoFrame mPrevLeftAndCurLeftFrame    = constructTrackFrame(fLastStereoFrame, fCurStereoFrame);
            TpOneFrameKptDescriptor     mPrevAndCurLeftKptsDescriptor;
            mPrevAndCurLeftKptsDescriptor.mFrameIDLeft      = nMatchResult.getFrameIDLeft();
            mPrevAndCurLeftKptsDescriptor.mFrameIDRight     = nMatchResult.getFrameIDRight();
            mPrevAndCurLeftKptsDescriptor.mDescriptorsLeft  = mFrameKptsDescriptorHistoryRecord.get(nMatchResult.getFrameIDLeft()).mDescriptorsLeft;
            mPrevAndCurLeftKptsDescriptor.mKeyPointsLeft    = mFrameKptsDescriptorHistoryRecord.get(nMatchResult.getFrameIDLeft()).mKeyPointsLeft;
            mPrevAndCurLeftKptsDescriptor.mDescriptorsRight = mFrameKptsDescriptorHistoryRecord.get(nMatchResult.getFrameIDRight()).mDescriptorsLeft;
            mPrevAndCurLeftKptsDescriptor.mKeyPointsRight   = mFrameKptsDescriptorHistoryRecord.get(nMatchResult.getFrameIDRight()).mKeyPointsLeft;
            mPtrDesciptorMatcher->showMatchResult(mPrevLeftAndCurLeftFrame, mPrevAndCurLeftKptsDescriptor, nMatchResult, "PrevLeft | CurLeft - Match Result");
        }
    }
    
    if(DebugManager::DebugControl().mBoolShowMatchResult && DebugManager::DebugControl().mBoolShowMatchResultWaitKey){
        cv::waitKey();
    }
    
    // TODO : Implement EXCORP function, and move this operation before track.
    // History Record;
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
    
    auto FuncTrack = [&](const TpOneFrameKptDescriptor& fFrmKptDescLast, const TpOneFrameKptDescriptor& fFrmKptDescCur){
        TpOneFrameKptDescriptor     mPrevAndCurLeftKptsDescriptor;
        mPrevAndCurLeftKptsDescriptor.mFrameIDLeft      = fFrmKptDescLast.FrameIDLeft();
        mPrevAndCurLeftKptsDescriptor.mFrameIDRight     = fFrmKptDescCur.FrameIDLeft();
        mPrevAndCurLeftKptsDescriptor.mDescriptorsLeft  = fFrmKptDescLast.mDescriptorsLeft;
        mPrevAndCurLeftKptsDescriptor.mKeyPointsLeft    = fFrmKptDescLast.mKeyPointsLeft;
        mPrevAndCurLeftKptsDescriptor.mDescriptorsRight = fFrmKptDescCur.mDescriptorsLeft;
        mPrevAndCurLeftKptsDescriptor.mKeyPointsRight   = fFrmKptDescCur.mKeyPointsLeft;
        //return mPtrDesciptorMatcher->match(mPrevAndCurLeftKptsDescriptor);
        
        auto m = mPtrDesciptorMatcher->match(mPrevAndCurLeftKptsDescriptor);
        return m;
        
        if(DebugManager::DebugControl().mBoolShowMatchResult){
            auto m = mPtrDesciptorMatcher->match(mPrevAndCurLeftKptsDescriptor);
            
            const StereoFrame& fCurStereoFrame      = dynamic_cast<const StereoFrame&>(fCurFrame);
            const StereoFrame& fLastStereoFrame     = dynamic_cast<const StereoFrame&>(mFrameHistoryRecord.get(fFrmKptDescLast.FrameID()));
            StereoFrame mPrevLeftAndCurLeftFrame    = constructTrackFrame(fLastStereoFrame, fCurStereoFrame);
            mPtrDesciptorMatcher->showMatchResult(mPrevLeftAndCurLeftFrame, mPrevAndCurLeftKptsDescriptor, m, "PrevLeft | CurLeft - Match Result");
            
            return m;
        }else{
            return mPtrDesciptorMatcher->match(mPrevAndCurLeftKptsDescriptor);
        }
    };
    
    // Step 1. track from last frame.
    Frame&                      fLastFrame              = mFrameHistoryRecord.back();
    TpOneFrameKptDescriptor&    fLastFrameKptDescriptor = mFrameKptsDescriptorHistoryRecord.back();
    // Pre-Cur is about 60%, 300/500 match and a little better than left-right's 50%, and all is right match.
    TpDescriptorMatchResult mPrevAndCurMatchResult      = FuncTrack(fLastFrameKptDescriptor, fCurFrameKptDescriptor);
    nOuterMatchResult.push_back(mPrevAndCurMatchResult);
    
    const StereoFrame& fCurStereoFrame      = dynamic_cast<const StereoFrame&>(fCurFrame);
    const StereoFrame& fLastStereoFrame     = dynamic_cast<const StereoFrame&>(fLastFrame);
    StereoFrame mPrevLeftAndCurLeftFrame    = constructTrackFrame(fLastStereoFrame, fCurStereoFrame);
    //mPtrDesciptorMatcher->showMatchResult(mPrevLeftAndCurLeftFrame, mPrevAndCurLeftKptsDescriptor, mPrevAndCurMatchResult, "PrevLeft | CurLeft - Match Result");
    //assert(mPtrDesciptorMatcher->debugDuplicatedMatch(f, mKptsDescriptors, mMatchResult));
    
    // Step 2. track from last kf frame.
    int nSzKFToTrack    = DebugManager::DebugControl().mMaxKeyFramesToMatchInTrack;
    if(nSzKFToTrack>0){
        //throw; // not test now.
        std::set<pair<TpFrameID,TpFrameID>> nSetFrameIDTracked;
        for(size_t nIdx=0;nIdx<nOuterMatchResult.size();++nIdx){
            TpDescriptorMatchResult& fKptDesc = nOuterMatchResult[nIdx];
            nSetFrameIDTracked.insert(pair<TpFrameID,TpFrameID>(fKptDesc.getFrameIDLeft(), fKptDesc.getFrameIDRight()));
        }
        
        auto nVecKFKptDesc  = mFrameKptsDescriptorHistoryRecord.getLastKeyFrames(nSzKFToTrack);
        for(size_t nIdxKFTOTrack=0;nIdxKFTOTrack<nVecKFKptDesc.size();++nIdxKFTOTrack){
            TpOneFrameKptDescriptor&    fKFKptDesc = nVecKFKptDesc[nIdxKFTOTrack];
            if(nSetFrameIDTracked.count(pair<TpFrameID,TpFrameID>(fKFKptDesc.FrameIDLeft(), fCurFrameKptDescriptor.FrameIDLeft())))
                continue;
            cout << "Track KF - KFID:" << fKFKptDesc.FrameID() << endl;
            TpDescriptorMatchResult mPrevKFAndCurMatchResult = FuncTrack(fKFKptDesc, fCurFrameKptDescriptor);
            nOuterMatchResult.push_back(mPrevKFAndCurMatchResult);
        }
    }
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
                
                cout << "Kpt: World|Came|Pixel-"<<nVecMapPt3D[nIdx]<<"|"<<nPtCam3D<<"|"<<nPtProj<<endl;
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


void KeyPointManager::trackByOpticalFlow(const Frame& fCurFrame, TpOneFrameKptDescriptor& nKptsDescriptors, TpDescriptorMatchResult& nInnerMatchResult, vector<TpDescriptorMatchResult>& nOuterMatchResult, FrameMatchResult& nPrevFrameMatchResult) {
    
    if(mFrameKptsDescriptorHistoryRecord.size() == 0){
        
        extract(fCurFrame, nKptsDescriptors);
        
        // match;
        nInnerMatchResult = mPtrDesciptorMatcher->match(nKptsDescriptors);
        
        return;
    }
    
    const StereoFrame&       nStereoFrameCur      = dynamic_cast<const StereoFrame&>(fCurFrame);
    StereoFrame&             nStereoFramePrev     = mFrameHistoryRecord.getLastOne();
    TpOneFrameKptDescriptor& nKptsDescriptorsPrev = mFrameKptsDescriptorHistoryRecord.getLastOne();
    
    // track prev and cur.
    TpVecPoint2f        nPtCur(nKptsDescriptorsPrev.mKeyPointsLeft.size());
    std::vector<uchar>  nBoolInlier(nKptsDescriptorsPrev.mKeyPointsLeft.size());
    cv::calcOpticalFlowPyrLK(nStereoFramePrev.getImageLeft(), fCurFrame.getImage(), 
                             Type::cvt(nKptsDescriptorsPrev.mKeyPointsLeft), nPtCur, 
                             nBoolInlier, cv::noArray());
    int nRectBorder = 20; cv::Size nSzImage = nStereoFrameCur.getImage().size();
    cv::Rect nRectInner(nRectBorder,nRectBorder, nSzImage.width-2*nRectBorder, nSzImage.height-2*nRectBorder);
    for(int nIdxPt=0,nSzPt=nPtCur.size();nIdxPt<nSzPt;++nIdxPt){
        if(nBoolInlier[nIdxPt] == 1){
            const cv::Point2f& p = nPtCur[nIdxPt];
            if(!Tools::isInWindow(p, nRectInner)){
                nBoolInlier[nIdxPt] = 0;
            }
        }
    }
    Tools::filter(nPtCur, nBoolInlier);
    
    TpVecMatchResult    nMatchTrack;
    for(int nIdx=0,nSz=nBoolInlier.size();nIdx<nSz;++nIdx){
        if(nBoolInlier[nIdx] == 1)
            nMatchTrack.push_back(TpOneMatchResult(nIdx, nMatchTrack.size(), 1));
    }
    
    // extract more Features
    int nSubRegionWidth = 100, nSubRegionHeight = 100;
    cv::Mat nSubRegionPtCount(std::ceil(nSzImage.height/nSubRegionHeight), std::ceil(nSzImage.width/nSubRegionWidth), CV_8UC1, cv::Scalar(0));
    for(int nIdxPt=0,nSzPt=nPtCur.size();nIdxPt<nSzPt;++nIdxPt){
        const auto& p = nPtCur[nIdxPt];
        int nIdxSubRegionWidth  = p.x / nSubRegionWidth;
        int nIdxSubRegionHeight = p.y / nSubRegionHeight;
        ++nSubRegionPtCount.at<uchar>(nIdxSubRegionHeight, nIdxSubRegionWidth);
    }
    int nSubRegionEmpty = std::count_if(nSubRegionPtCount.begin<uchar>(), nSubRegionPtCount.end<uchar>(), [](uchar nPtCount){return nPtCount==0;});
    if(nSubRegionEmpty*1.0/nSubRegionPtCount.size().area() >= 0.4){
        cv::Mat mask(nSzImage, CV_8UC1, cv::Scalar(255));
        for(int nIdxRow=0,nSzRow=nSubRegionPtCount.rows;nIdxRow<nSzRow;++nIdxRow){
            for(int nIdxCol=0,nSzCol=nSubRegionPtCount.cols;nIdxCol<nSzCol;++nIdxCol){
                if(nSubRegionPtCount.at<uchar>(nIdxRow, nIdxCol)>0) {
                    cv::Rect nRectSubRegion = cv::Rect(nIdxCol*nSubRegionWidth,nIdxRow*nSubRegionHeight,nSubRegionWidth,nSubRegionHeight);
                    nRectSubRegion.width  = std::min(nSzImage.width,  nRectSubRegion.br().x) - nRectSubRegion.x;
                    nRectSubRegion.height = std::min(nSzImage.height, nRectSubRegion.br().y) - nRectSubRegion.y;
                    cv::Mat(mask, nRectSubRegion).setTo(0);
                }
            }
        }
        
        //cv::imshow("mask", mask); cv::waitKey();
        
        TpVecPoint2f    nPtsNew;
        if(false){
            cv::goodFeaturesToTrack(fCurFrame.getImage(), nPtsNew, 200, 0.4, 5, mask);
        }else{
            TpVecKeyPoints nKptsNew; cv::Mat nDescsNew;
            (*mPtrORBExtractorLeft)(nStereoFrameCur.getImageLeft(), cv::Mat(), nKptsNew, nDescsNew);
            std::vector<bool> nVecToSave(nKptsNew.size(), false);
            for(int nIdxPt=0,nSzPt=nKptsNew.size();nIdxPt<nSzPt;++nIdxPt){
                const auto& p = nKptsNew[nIdxPt];
                int nIdxSubRegionWidth  = p.pt.x / nSubRegionWidth;
                int nIdxSubRegionHeight = p.pt.y / nSubRegionHeight;
                nVecToSave[nIdxPt] = nSubRegionPtCount.at<uchar>(nIdxSubRegionHeight, nIdxSubRegionWidth) == 0;
            }
            Tools::filter(nKptsNew, nVecToSave);
            nPtsNew = Type::cvt(nKptsNew);
        }
        
        //cv::cornerSubPix(fCurFrame.getImage(), nKptsNew, cv::Size(3,3), );
        //TpVecKeyPoints nKptsNew = Type::cvt(nPtsNew);
        nPtCur.insert(nPtCur.end(), nPtsNew.begin(), nPtsNew.end());
        cout << "Detect New Kpt by Region:" << nPtsNew.size() << endl;
    }
    
    /*
    // stereo matching.
    TpVecPoint2f        nPtCurRight(nPtCur.size());
    std::vector<uchar>  nBoolInlierRight(nPtCur.size());
    cv::calcOpticalFlowPyrLK(fCurFrame.getImage(), nStereoFrameCur.getImageRight(), nPtCur, nPtCurRight, nBoolInlierRight, cv::noArray());
    Tools::filter(nPtCurRight, nBoolInlierRight);
    TpVecMatchResult    nMatchStereo;
    for(int nIdx=0,nSz=nBoolInlierRight.size();nIdx<nSz;++nIdx){
        nMatchStereo.push_back(TpOneMatchResult(nIdx, nMatchStereo.size(), 1));
    }
    TpVecKeyPoints      nKptCur         = Type::cvt(nPtCur);
    TpVecKeyPoints      nKptCurRight    = Type::cvt(nPtCurRight);
    
    //
    nKptsDescriptors.mFrameIDLeft       = fCurFrame.FrameID();
    nKptsDescriptors.mFrameIDRight      = fCurFrame.FrameID();
    nKptsDescriptors.mKeyPointsLeft     = nKptCur;
    nKptsDescriptors.mKeyPointsRight    = nKptCurRight;
    nKptsDescriptors.mDescriptorsLeft.resize(nKptsDescriptors.mKeyPointsLeft.size());
    nKptsDescriptors.mDescriptorsRight.resize(nKptsDescriptors.mKeyPointsRight.size());
    nInnerMatchResult = TpDescriptorMatchResult(fCurFrame.FrameID(),fCurFrame.FrameID(), nMatchStereo, nKptCur.size(), nKptCurRight.size());
    */
    
    TpVecKeyPoints nKptCur = Type::cvt(nPtCur);
    TpVecKeyPoints nKptCurRight;
    
    cv::Mat nDescriptorsLeftCur,nDescriptorsRightCur;
    (*mPtrORBExtractorLeft).computeKptDescriptors(nStereoFrameCur.getImageLeft(), nKptCur, nDescriptorsLeftCur);
    (*mPtrORBExtractorRight)(nStereoFrameCur.getImageRight(), cv::Mat(), nKptCurRight, nDescriptorsRightCur);
    // match;
    nKptsDescriptors.mFrameIDLeft       = fCurFrame.FrameID();
    nKptsDescriptors.mFrameIDRight      = fCurFrame.FrameID();
    nKptsDescriptors.mKeyPointsLeft     = nKptCur;
    nKptsDescriptors.mKeyPointsRight    = nKptCurRight;
    nKptsDescriptors.mDescriptorsLeft   = nDescriptorsLeftCur;
    nKptsDescriptors.mDescriptorsRight  = nDescriptorsRightCur;
    nInnerMatchResult = mPtrDesciptorMatcher->match(nKptsDescriptors);
    
    TpDescriptorMatchResult nPrevCurMatch(nPrevFrameMatchResult.FrameID(), fCurFrame.FrameID(), 
                                          nMatchTrack, nKptsDescriptorsPrev.mKeyPointsLeft.size() , nKptCur.size());
    nOuterMatchResult.push_back(nPrevCurMatch);
}

}
}

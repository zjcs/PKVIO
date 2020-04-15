#include <iostream>
#include <opencv2/highgui.hpp>
#include "System.h"
#include "../DatasetManager/DatasetEuRoc.h"
#include "../DatasetManager/DatasetSimulator.h"

#include "../Tools/Tools.h"
#include "../Solver/Solver.h"

namespace PKVIO {
namespace System {

TpPtrVIOSystem generateVIOSystem(void){
  return std::make_shared<System>();
};
    
System::~System(){
    exit();
}

void System::exec(void) {
    doexec();
}

void System::initialize(const DebugManager::TpDebugControl& nDbgCtrl) 
{
    //mPtrDataset = std::make_shared<DatasetEuRoc>("E:/DataSet/MH_01_easy/");
    string sDatasetPath = std::string("/home/ubuntu/work/dataset/DataSet/MH_01_easy/");
    cout << nDbgCtrl.str();
    DebugManager::DebugControl() = nDbgCtrl;
    mPtrDbgCtrl = &DebugManager::DebugControl();
    
    DebugManager::setUseSimulator(mPtrDbgCtrl->mBoolUseSimulator);
    if(!DebugManager::getUseSimulator()){
        mPtrDataset = std::make_shared<DatasetEuRoc>(sDatasetPath);   
        mPtrDataset->initialize();
    } else {
        mPtrDataset = std::make_shared<DatasetSimulator>(sDatasetPath);
        mPtrDataset->initialize();
        mKeyPointMgr.setSimulator(true, mPtrDataset->getPtrCamera());
    }
    
    mPtrKeyFrameMgr = std::make_shared<KeyFrameManager::KeyFrameManager>();
    
    cout << "PkVio System intialization Finish." << endl;
}

void System::exit(void) {
    cout << "PkVio System Exit Now." << endl;
}

void System::doexec(void) {
    mPtrFuncDoExec();
}


void System::showVideoOnly() {
    auto FuncDoShowVideoOnly = [&](){
        for (int nIndex = 0; !mPtrDataset->isFinished(); ++nIndex) {
            Frame& mCurFrame = mPtrDataset->read();
            cv::imshow("Viewer", mCurFrame.getImage());
            cv::waitKey(40);
        }
    };
    
    mPtrFuncDoExec = FuncDoShowVideoOnly;
    exec();
}


void System::runVIO() {
    setRunVIO();
    exec();
}


void System::debugCountTrackingKptIDWihtMapPointID(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    int nCurCountKptIDsWithMapPoint = mPtrKeyFrameMgr->countTrackKptIDsWithMapPointID(mFrameKptIDMgr);
    static int nPrevCountKptIDsWithMapPoint = 0;
    
    if(mPtrKeyFrameMgr->isKeyFrame(fFrame.FrameID())) {
        // do nothing.
    } else {
        if(nCurCountKptIDsWithMapPoint>nPrevCountKptIDsWithMapPoint) {
            
            cout << "**** Error: Match Point number should decreas... Prev|Cur - " << nPrevCountKptIDsWithMapPoint 
                 << " | " << nCurCountKptIDsWithMapPoint << endl;
            
            TpFrameID nFrameIDPre2 = fFrame.FrameID()-2, nFrameIDPrev = fFrame.FrameID()-1, nFrameIDCur = fFrame.FrameID();
            
            KeyPointManager::TpOneFrameKptDescriptor nFrameKptDescriptorPre2Prev;
            KeyPointManager::TpDescriptorMatchResult nDescriptorsMatchResultPre2Prev;
            mKeyPointMgr.getTrackingKptDescriptorMatchResult(nFrameIDPre2, nFrameIDPrev, nFrameKptDescriptorPre2Prev, nDescriptorsMatchResultPre2Prev);
            
            KeyPointManager::TpOneFrameKptDescriptor nFrameKptDescriptorPrevCur;
            KeyPointManager::TpDescriptorMatchResult nDescriptorsMatchResultPrevCur;
            mKeyPointMgr.getTrackingKptDescriptorMatchResult(nFrameIDPrev, nFrameIDCur, nFrameKptDescriptorPrevCur, nDescriptorsMatchResultPrevCur);
            
            cv::Mat mPr2PrevMatch = mKeyPointMgr.showMatchResult(nFrameIDPre2, nFrameIDPrev , true);
            cv::Mat mPrevCurMatch = mKeyPointMgr.showMatchResult(nFrameIDPrev, nFrameIDCur, true);
            cv::waitKey();
            
        }
    }
    nPrevCountKptIDsWithMapPoint = nCurCountKptIDsWithMapPoint;
}

void System::setRunVIOSimple(bool bRunAllFrame)
{
    auto FuncDorunVIO = [&](){
        for (int nIndex = 0; !mPtrDataset->isFinished(); ++nIndex) {
            Frame& mCurFrame = mPtrDataset->read();
            TpFrameID nFrameIDCur = mCurFrame.FrameID();
            
            
            if(mCurFrame.getImage().empty() && DatasetManager::isOfflineDatasetType(mPtrDataset->type())){
                FrameInfo mFrmInfo = dynamic_cast<DatasetOfflineImp*>(mPtrDataset.get())->getFrameInfor(mCurFrame.FrameID());
                cout<< "Empyt:" << mFrmInfo.mFrameIndex << " - " << mFrmInfo.mStrFileName <<endl;
            }
            StereoFrame& mCurFrameStereo = dynamic_cast<StereoFrame&>(mCurFrame);
            auto mPtrDatasetEuRoc    = dynamic_cast<DatasetManager::DatasetEuRoc*>(mPtrDataset.get());
            cv::Matx44f nPrTPl       = mPtrDataset->getPtrCamera()->getTranslateCvtPtLViewToRView().get();
            const auto& nCameraLeft  = mPtrDataset->getPtrCamera()->CameraLeft();
            const auto& nCameraRight = mPtrDataset->getPtrCamera()->CameraRight();
            const TpCameraInnerParam& nCameraInnerLeft  = mPtrDataset->getPtrCamera()->CameraLeft().getInnerParam();
            const TpCameraInnerParam& nCameraInnerRight = mPtrDataset->getPtrCamera()->CameraRight().getInnerParam();
            
            //cout << "Feature Matching ..." <<endl;
            const KeyPointManager::FrameMatchResult& mFrameMatchResult = mKeyPointMgr.solve(mCurFrame);
            //cout << "Feature Matching Finish." <<endl;
            
            mPtrCameraPoseCurFrame = mPtrKeyFrameMgr->generateOneFrameCameraPose(mCurFrame);
            if(mFrameMatchResult.sizeOuterFrameDescriptorMatchResult()==0)
                continue;
            static map<TpFrameID, map<TpKeyPointIndex, cv::Point3f>> nMapMap3D;
            auto& nStereoFramePrev = mKeyPointMgr.getStereoFrameHistory().get(nFrameIDCur-1);
            
            // track;
            bool bStateTrackSuccess = false;
            {
                auto IterMap3DPrev = nMapMap3D.find(nFrameIDCur-1);
                if(IterMap3DPrev!=nMapMap3D.end()){
                    auto& nMap3DPrev = IterMap3DPrev->second;
                    auto& nMap3DCur =  nMapMap3D[nFrameIDCur];
                    
                    
                    auto& mMatchResultTrack = mFrameMatchResult.getOuterFrameDescriptorMatchResult(0);
                    auto& mKptsDescriptors = mKeyPointMgr.getDescriptor(nFrameIDCur);
                    std::vector<cv::Point3f> nPt3DPrev;
                    std::vector<cv::Point2f> nPt2DCur;
                    for(int nIdxMatch=0,nSz=mMatchResultTrack.getMatch().size();nIdxMatch<nSz;++nIdxMatch){
                        const TpOneMatchResult& nMatch = mMatchResultTrack.getMatch()[nIdxMatch];
                        auto IterPt3D = nMap3DPrev.find(nMatch.queryIdx);
                        if(IterPt3D!=nMap3DPrev.end()){
                            nPt3DPrev.push_back(IterPt3D->second);
                            nPt2DCur.push_back(mKptsDescriptors.mKeyPointsLeft[nMatch.trainIdx].pt);
                            nMap3DCur[nMatch.trainIdx] = IterPt3D->second;
                        }
                    }
                    //cout << "track P3P:" << nFrameIDCur << " - " << nPt3DPrev.size()<<endl;
                    
                    cv::Mat RMat,rvec, tvec; cv::Mat mask;
                    bool bPnpState = cv::solvePnPRansac(nPt3DPrev, nPt2DCur,nCameraInnerLeft.getInnerMat(),
                                       nCameraInnerLeft.getDistorParam(), rvec, tvec, false, 100,  8, 0.99, mask, cv::SOLVEPNP_P3P);
                    bStateTrackSuccess = bPnpState;
                    if(bPnpState){
                        //cout << mask.size()<<endl;
                        //cout << std::count(mask.begin<uchar>(),mask.end<uchar>(),1)<<endl;
                        cv::Mat T = cv::Mat::eye(4,4, CV_64FC1);
                        cv::Rodrigues(rvec, RMat);
                        RMat.copyTo(T(cv::Rect(0,0,3,3)));
                        tvec.copyTo(T(cv::Rect(3,0,1,3)));
                        mPtrCameraPoseCurFrame->setMatx44f(cv::Matx44f(T));
                    }else{
                        cout << "@@@ track fail" <<endl;
                    }
                }
            }
            
            // triangular;
            {
                auto& mMatchResultTrack = mFrameMatchResult.getOuterFrameDescriptorMatchResult(0);
                auto& mKptsDescriptors = mKeyPointMgr.getDescriptor(nFrameIDCur);
                auto& mKptsDescriptorsPrev = mKeyPointMgr.getDescriptor(nFrameIDCur-1);
                
                auto mPtrCameraPosePrevFrame = mPtrKeyFrameMgr->getFrameCameraPose(nFrameIDCur-1);
                
                cv::Mat nKptPrev, nKptCur, mask;    
                mMatchResultTrack.getMatchKpts(mKptsDescriptorsPrev.mKeyPointsLeft, mKptsDescriptors.mKeyPointsLeft, nKptPrev, nKptCur);
                size_t  nSzMatchTrack = nKptPrev.rows;
                
                if(!bStateTrackSuccess)
                {
                    cv::Mat E = cv::findEssentialMat(nKptPrev, nKptCur, nCameraInnerLeft.getInnerMat(), CV_RANSAC, 0.999, 10.0, mask);
                    
                    TpVecKeyPoints nVecKptPrev, nVecKptCur;
                    mMatchResultTrack.getMatchKpts(mKptsDescriptorsPrev.mKeyPointsLeft, mKptsDescriptors.mKeyPointsLeft, nVecKptPrev, nVecKptCur);
                    //Tools::drawMatch(nStereoFramePrev.getImageLeft(),nVecKptPrev, mCurFrameStereo.getImageLeft(), nVecKptCur, true, "Track-Init");
                    
                    cv::Mat RMat, tvec;
                    int nInlier = cv::recoverPose(E, nKptPrev, nKptCur, nCameraInnerLeft.getInnerMat(), RMat, tvec, mask);
                    cv::Mat T = cv::Mat::eye(4,4, CV_32FC1); RMat.copyTo(T(cv::Rect(0,0,3,3))); tvec.copyTo(T(cv::Rect(3,0,1,3)));
                    //cout << "Relative RT: " << endl <<T<<endl;
                    mPtrCameraPosePrevFrame->setMatx44f(cv::Matx44f::eye());
                    mPtrCameraPoseCurFrame->setMatx44f(mPtrCameraPosePrevFrame->getMatx44f()*cv::Matx44f(T));
                }
                
                cv::Mat nProjMtxPrev  = nCameraInnerLeft.getInnerMat() * cv::Mat(cv::Mat(mPtrCameraPosePrevFrame->getMatx44f()), cv::Rect(0,0,4,3));
                cv::Mat nProjMtxCur = nCameraInnerLeft.getInnerMat() * cv::Mat(cv::Mat(mPtrCameraPoseCurFrame->getMatx44f()), cv::Rect(0,0,4,3));
                
                TpVecKeyPoints nVecKptsPrev, nVecKptsCur; 
                std::vector<cv::Point2f> nKptsPrev, nKptsCur; 
                std::vector<int> nFilteredMatchIdx2InitMatchIdx;
                nFilteredMatchIdx2InitMatchIdx.reserve(mask.rows);
                auto& nMap3DCur =  nMapMap3D[nFrameIDCur];
                for(int nIdx=0,nSz=nSzMatchTrack;nIdx<nSz;++nIdx){
                    if(nMap3DCur.find(mMatchResultTrack.getMatch()[nIdx].trainIdx)!=nMap3DCur.end())
                        continue;
                    if(!mask.empty() && mask.at<uchar>(0,nIdx)==0)
                        continue;
                    nVecKptsPrev.push_back(cv::KeyPoint(nKptPrev.at<cv::Point2f>(0,nIdx), 1));
                    nVecKptsCur.push_back(cv::KeyPoint(nKptCur.at<cv::Point2f>(0,nIdx), 1));
                    nKptsPrev.push_back(nKptPrev.at<cv::Point2f>(0,nIdx));
                    nKptsCur.push_back(nKptCur.at<cv::Point2f>(0,nIdx));
                    nFilteredMatchIdx2InitMatchIdx.push_back(nIdx);
                    //cout << "parallax:" << nFilteredMatchIdx2InitMatchIdx.size() << "-"<< nKptsCur.back() - nKptsPrev.back() << endl;
                }
                
                //Tools::drawMatch(mKeyPointMgr.getStereoFrameHistory().get(nFrameIDCur-1).getImageLeft(),nVecKptsPrev, mCurFrameStereo.getImageLeft(), nVecKptsCur, true, "Track-Essen"); cv::waitKey();
                int nInner = mask.empty() ? -1 : std::count(mask.begin<uchar>(),mask.end<uchar>(), 1);
                //cout << "triangular: FrameID|Inner|AllMatch " << nFrameIDCur << " - " << nInner << "-" << nSzMatchTrack << endl;
                //cout << nKptsPrev.size() << endl;
                
                //cout << "nInner: "<< nInner <<endl; cout << mask.type() << mask.size()<<endl;
                //cout << nProjMtxPrev << endl << nProjMtxCur <<endl;
                if(nKptsPrev.size()>0){
                    cv::Mat vPt4D;
                    cv::triangulatePoints(nProjMtxPrev, nProjMtxCur, nKptsPrev, nKptsCur, vPt4D);
                    for (int nIdx=0,nSz=vPt4D.cols;nIdx<nSz;++nIdx){
                        cv::Vec4f nPt4D = vPt4D.col(nIdx);
                        cv::Vec3f nPt3DNorm = cv::Vec3f(nPt4D(0), nPt4D(1), nPt4D(2))/nPt4D(3);
                        //cout << nPt4D << nPt3DNorm <<endl;
                        int nKptIdx = mMatchResultTrack.getMatch()[nFilteredMatchIdx2InitMatchIdx[nIdx]].trainIdx;
                        nMap3DCur[nKptIdx] = cv::Point3f(nPt3DNorm);
                    }
                    //cv::waitKey();
                }
            }
            
            cv::Mat mImgToShow3 = mCurFrame.Image().clone();
            
            cout << "Pose: " << nFrameIDCur << endl << mPtrCameraPoseCurFrame->getMatx44f() <<endl;
            
            if(!mImgToShow3.empty()){
                mTrackingImageCurFrame = mImgToShow3;
                //cv::imshow("Viewer", mImgToShow);
                //cv::waitKey(mbRunAllFrame?40:1);
            }
            //cv::waitKey();
            if(!mbRunAllFrame){
                break;
            }
       }
    };
    
    mPtrFuncDoExec = FuncDorunVIO;
    mbRunAllFrame = bRunAllFrame;
}

void System::setRunVIOSimpleStereo(bool bRunAllFrame)
{
    auto FuncDorunVIO = [&](){
        for (int nIndex = 0; !mPtrDataset->isFinished(); ++nIndex) {
            Frame& mCurFrame = mPtrDataset->read();
            TpFrameID nFrameIDCur = mCurFrame.FrameID();
            
            
            if(mCurFrame.getImage().empty() && DatasetManager::isOfflineDatasetType(mPtrDataset->type())){
                FrameInfo mFrmInfo = dynamic_cast<DatasetOfflineImp*>(mPtrDataset.get())->getFrameInfor(mCurFrame.FrameID());
                cout<< "Empyt:" << mFrmInfo.mFrameIndex << " - " << mFrmInfo.mStrFileName <<endl;
            }
            StereoFrame& mCurFrameStereo = dynamic_cast<StereoFrame&>(mCurFrame);
            auto mPtrDatasetEuRoc    = dynamic_cast<DatasetManager::DatasetEuRoc*>(mPtrDataset.get());
            cv::Matx44f nPrTPl       = mPtrDataset->getPtrCamera()->getTranslateCvtPtLViewToRView().get();
            const auto& nCameraLeft  = mPtrDataset->getPtrCamera()->CameraLeft();
            const auto& nCameraRight = mPtrDataset->getPtrCamera()->CameraRight();
            const TpCameraInnerParam& nCameraInnerLeft  = mPtrDataset->getPtrCamera()->CameraLeft().getInnerParam();
            const TpCameraInnerParam& nCameraInnerRight = mPtrDataset->getPtrCamera()->CameraRight().getInnerParam();
            
            //cout << "Feature Matching ..." <<endl;
            const KeyPointManager::FrameMatchResult& mFrameMatchResult = mKeyPointMgr.solve(mCurFrame);
            //cout << "Feature Matching Finish." <<endl;
            
            mPtrCameraPoseCurFrame = mPtrKeyFrameMgr->generateOneFrameCameraPose(mCurFrame);
            if(mFrameMatchResult.sizeOuterFrameDescriptorMatchResult()==0)
                continue;
            static map<TpFrameID, map<TpKeyPointIndex, TpPtrMapPoint3D>> nMapMap3D;
            auto& nStereoFramePrev = mKeyPointMgr.getStereoFrameHistory().get(nFrameIDCur-1);
            
            // track;
            bool bStateTrackSuccess = false;
            {
                auto IterMap3DPrev = nMapMap3D.find(nFrameIDCur-1);
                if(IterMap3DPrev!=nMapMap3D.end()){
                    auto& nMap3DPrev = IterMap3DPrev->second;
                    auto& nMap3DCur =  nMapMap3D[nFrameIDCur];
                    
                    
                    auto& mMatchResultTrack = mFrameMatchResult.getOuterFrameDescriptorMatchResult(0);
                    auto& mKptsDescriptors = mKeyPointMgr.getDescriptor(nFrameIDCur);
                    std::vector<TpPtrMapPoint3D> nPt3DPrev;
                    std::vector<cv::Point2f> nPt2DCur;
                    for(int nIdxMatch=0,nSz=mMatchResultTrack.getMatch().size();nIdxMatch<nSz;++nIdxMatch){
                        const TpOneMatchResult& nMatch = mMatchResultTrack.getMatch()[nIdxMatch];
                        auto IterPt3D = nMap3DPrev.find(nMatch.queryIdx);
                        if(IterPt3D!=nMap3DPrev.end()){
                            nPt3DPrev.push_back(IterPt3D->second);
                            nPt2DCur.push_back(mKptsDescriptors.mKeyPointsLeft[nMatch.trainIdx].pt);
                            nMap3DCur[nMatch.trainIdx] = IterPt3D->second;
                        }
                    }
                    //cout << "track P3P:" << nFrameIDCur << " - " << nPt3DPrev.size()<<endl;
                    
                    cout << "FrmID:" << nFrameIDCur << (mPtrDbgCtrl->mBoolUseG2OSolver?"G2O":"PnP") <<endl;
                    
                    if(mPtrDbgCtrl->mBoolUseG2OSolver){
                        std::map< Type::TpFrameID, TpPtrCameraPose> nMapFrameID2CameraPose;
                        std::map<Type::TpMapPointID, Type::TpPtrMapPoint3D > nMapMapPointID2MapPoint3D;
                        TpPtrCameraStereo nPtrCameraStereo = mPtrDataset->getPtrCamera();
                        Solver::TpVecVisualMeasurement nVecVisualMeasurement; 
                        nMapFrameID2CameraPose[nFrameIDCur] = mPtrCameraPoseCurFrame;
                        for(int nIdx=0,nSz = nPt3DPrev.size();nIdx<nSz;++nIdx){
                            
                            bool bShallowCopyToOptimizeMapPoint = true;
                            
                            auto pMp3D = (bShallowCopyToOptimizeMapPoint)? 
                                // Shallow Copy, both the mappoint and camera pose will be optimized in the Solver.
                                nPt3DPrev[nIdx]:
                                // Deep Clone, donot update the mappoint,which is not same as fix-mappoint. The result is also different.
                                std::make_shared<TpMapPoint3D>(*nPt3DPrev[nIdx]);
                                
                            Solver::TpVisualMeasurement nVm;
                            nVm.mFrameID = nFrameIDCur;
                            nVm.mKeyPoint = cv::KeyPoint(nPt2DCur[nIdx], 1);
                            nVm.mMapPointID = nIdx;
                            nVm.mMapPoint3D = pMp3D;
                            nMapMapPointID2MapPoint3D[nIdx] = pMp3D;
                            nVecVisualMeasurement.push_back(nVm);
                        }
                        
                        Solver::Solver nSolverCurFramePose;
                        //nSolverCurFramePose.initCamerPoses(nMapFrameID2CameraPose);
                        //nSolverCurFramePose.initMapPoints(nMapMapPointID2MapPoint3D);
                        nSolverCurFramePose.solve(nMapFrameID2CameraPose, nMapMapPointID2MapPoint3D, nVecVisualMeasurement, nPtrCameraStereo);
                        auto nPtrCameraPoseCur = nMapFrameID2CameraPose[nFrameIDCur];
                        
                        bStateTrackSuccess = true;
                    }else{
                        for(int nIdx=0,nSz = nPt3DPrev.size();nIdx<nSz;++nIdx){
                            cout << "Mp3D:" << nPt3DPrev[nIdx]<<endl;
                        }
                        for(int nIdx=0,nSz = nPt3DPrev.size();nIdx<nSz;++nIdx){
                            cout << "Pt2D:" << nPt2DCur[nIdx]<<endl;
                        }
                        cv::Mat RMat,rvec, tvec; cv::Mat mask;
                        bool bPnpState = cv::solvePnPRansac(nPt3DPrev, nPt2DCur,nCameraInnerLeft.getInnerMat(),
                                        nCameraInnerLeft.getDistorParam(), rvec, tvec, false, 100,  8, 0.9, mask, cv::SOLVEPNP_P3P);
                        bStateTrackSuccess = bPnpState;
                        if(bPnpState){
                            //cout << mask.size()<<endl;
                            //cout << std::count(mask.begin<uchar>(),mask.end<uchar>(),1)<<endl;
                            cv::Mat T = cv::Mat::eye(4,4, CV_64FC1);
                            cv::Rodrigues(rvec, RMat);
                            RMat.copyTo(T(cv::Rect(0,0,3,3)));
                            tvec.copyTo(T(cv::Rect(3,0,1,3)));
                            mPtrCameraPoseCurFrame->setMatx44f(cv::Matx44f(T));
                        }else{
                            cout << "@@@ track fail" <<endl;
                        }
                            
                    }

                }
            }
            
            // triangular;
            {
                auto& mMatchResultTrack = mFrameMatchResult.getOuterFrameDescriptorMatchResult(0);
                auto& mKptsDescriptors = mKeyPointMgr.getDescriptor(nFrameIDCur);
                auto& mKptsDescriptorsPrev = mKeyPointMgr.getDescriptor(nFrameIDCur-1);
                
                auto mPtrCameraPosePrevFrame = mPtrKeyFrameMgr->getFrameCameraPose(nFrameIDCur-1);
                
                cv::Mat nKptPrev, nKptCur, mask;    
                mMatchResultTrack.getMatchKpts(mKptsDescriptorsPrev.mKeyPointsLeft, mKptsDescriptors.mKeyPointsLeft, nKptPrev, nKptCur);
                size_t  nSzMatchTrack = nKptPrev.rows;
                
                if(!bStateTrackSuccess)
                {
                    // The initialization is very importan, try modify 6st 1.0 to 10.
                    cv::Mat E = cv::findEssentialMat(nKptPrev, nKptCur, nCameraInnerLeft.getInnerMat(), CV_RANSAC, 0.999, 1.00, mask);
                    
                    TpVecKeyPoints nVecKptPrev, nVecKptCur;
                    mMatchResultTrack.getMatchKpts(mKptsDescriptorsPrev.mKeyPointsLeft, mKptsDescriptors.mKeyPointsLeft, nVecKptPrev, nVecKptCur);
                    //Tools::drawMatch(nStereoFramePrev.getImageLeft(),nVecKptPrev, mCurFrameStereo.getImageLeft(), nVecKptCur, true, "Track-Init");
                    
                    cv::Mat RMat, tvec;
                    int nInlier = cv::recoverPose(E, nKptPrev, nKptCur, nCameraInnerLeft.getInnerMat(), RMat, tvec, mask);
                    cv::Mat T = cv::Mat::eye(4,4, CV_32FC1); RMat.copyTo(T(cv::Rect(0,0,3,3))); tvec.copyTo(T(cv::Rect(3,0,1,3)));
                    cout << "ReInitialize: " << nFrameIDCur << "-Inliner:" << nInlier <<endl
                         <<  "Relative RT: " << endl <<T<<endl;;
                    mPtrCameraPosePrevFrame->setMatx44f(cv::Matx44f::eye());
                    mPtrCameraPoseCurFrame->setMatx44f(mPtrCameraPosePrevFrame->getMatx44f()*cv::Matx44f(T));
                }
                
                cv::Mat nProjMtxPrev  = nCameraInnerLeft.getInnerMat() * cv::Mat(cv::Mat(mPtrCameraPosePrevFrame->getMatx44f()), cv::Rect(0,0,4,3));
                cv::Mat nProjMtxCur = nCameraInnerLeft.getInnerMat() * cv::Mat(cv::Mat(mPtrCameraPoseCurFrame->getMatx44f()), cv::Rect(0,0,4,3));
                
                TpVecKeyPoints nVecKptsPrev, nVecKptsCur; 
                std::vector<cv::Point2f> nKptsPrev, nKptsCur; 
                std::vector<int> nFilteredMatchIdx2InitMatchIdx;
                nFilteredMatchIdx2InitMatchIdx.reserve(mask.rows);
                auto& nMap3DCur =  nMapMap3D[nFrameIDCur];
                for(int nIdx=0,nSz=nSzMatchTrack;nIdx<nSz;++nIdx){
                    if(nMap3DCur.find(mMatchResultTrack.getMatch()[nIdx].trainIdx)!=nMap3DCur.end())
                        continue;
                    if(!mask.empty() && mask.at<uchar>(0,nIdx)==0)
                        continue;
                    nVecKptsPrev.push_back(cv::KeyPoint(nKptPrev.at<cv::Point2f>(0,nIdx), 1));
                    nVecKptsCur.push_back(cv::KeyPoint(nKptCur.at<cv::Point2f>(0,nIdx), 1));
                    nKptsPrev.push_back(nKptPrev.at<cv::Point2f>(0,nIdx));
                    nKptsCur.push_back(nKptCur.at<cv::Point2f>(0,nIdx));
                    nFilteredMatchIdx2InitMatchIdx.push_back(nIdx);
                    //cout << "parallax:" << nFilteredMatchIdx2InitMatchIdx.size() << "-"<< nKptsCur.back() - nKptsPrev.back() << endl;
                }
                
                //Tools::drawMatch(mKeyPointMgr.getStereoFrameHistory().get(nFrameIDCur-1).getImageLeft(),nVecKptsPrev, mCurFrameStereo.getImageLeft(), nVecKptsCur, true, "Track-Essen"); cv::waitKey();
                int nInner = mask.empty() ? -1 : std::count(mask.begin<uchar>(),mask.end<uchar>(), 1);
                cout << "triangular: FrameID|Inner|AllMatch " << nFrameIDCur << " - " << nInner << "-" << nSzMatchTrack << endl;
                //cout << nKptsPrev.size() << endl;
                
                //cout << "nInner: "<< nInner <<endl; cout << mask.type() << mask.size()<<endl;
                //cout << nProjMtxPrev << endl << nProjMtxCur <<endl;
                if(nKptsPrev.size()>0){
                    cv::Mat vPt4D;
                    cv::triangulatePoints(nProjMtxPrev, nProjMtxCur, nKptsPrev, nKptsCur, vPt4D);
                    for (int nIdx=0,nSz=vPt4D.cols;nIdx<nSz;++nIdx){
                        cv::Vec4f nPt4D = vPt4D.col(nIdx);
                        cv::Vec3f nPt3DNorm = cv::Vec3f(nPt4D(0), nPt4D(1), nPt4D(2))/nPt4D(3);
                        //cout << nPt4D << nPt3DNorm <<endl;
                        int nKptIdx = mMatchResultTrack.getMatch()[nFilteredMatchIdx2InitMatchIdx[nIdx]].trainIdx;
                        nMap3DCur[nKptIdx] = std::make_shared<TpMapPoint3D>(cv::Point3f(nPt3DNorm));
                    }
                    //cv::waitKey();
                }
            }
            
            cv::Mat mImgToShow3 = mCurFrame.Image().clone();
            
            cout << "Pose: " << nFrameIDCur << endl << mPtrCameraPoseCurFrame->getMatx44f() <<endl;
            
            if(!mImgToShow3.empty()){
                mTrackingImageCurFrame = mImgToShow3;
                //cv::imshow("Viewer", mImgToShow);
                //cv::waitKey(mbRunAllFrame?40:1);
            }
            //cv::waitKey();
            if(!mbRunAllFrame){
                break;
            }
       }
    };
    
    mPtrFuncDoExec = FuncDorunVIO;
    mbRunAllFrame = bRunAllFrame;
}

void System::setRunVIO(bool bRunAllFrame /*= true*/) {
    //return setRunVIOSimple(bRunAllFrame);
    if(mPtrDbgCtrl->mBoolUseCoVisMgr == false)
        return setRunVIOSimpleStereo(bRunAllFrame);
    
    auto FuncDorunVIO = [&](){
        for (int nIndex = 0; !mPtrDataset->isFinished(); ++nIndex) {
            Frame& mCurFrame = mPtrDataset->read();
            TpFrameID nFrameIDCur = mCurFrame.FrameID();
            
            
            if(mCurFrame.getImage().empty() && DatasetManager::isOfflineDatasetType(mPtrDataset->type())){
                FrameInfo mFrmInfo = dynamic_cast<DatasetOfflineImp*>(mPtrDataset.get())->getFrameInfor(mCurFrame.FrameID());
                cout<< "Empyt:" << mFrmInfo.mFrameIndex << " - " << mFrmInfo.mStrFileName <<endl;
            }
            StereoFrame& mCurFrameStereo = dynamic_cast<StereoFrame&>(mCurFrame);
            auto mPtrDatasetEuRoc = dynamic_cast<DatasetManager::DatasetEuRoc*>(mPtrDataset.get());
            cv::Matx44f nPrTPl = mPtrDataset->getPtrCamera()->getTranslateCvtPtLViewToRView().get();
            const auto& nCameraLeft  = mPtrDataset->getPtrCamera()->CameraLeft();
            const auto& nCameraRight = mPtrDataset->getPtrCamera()->CameraRight();
            const TpCameraInnerParam& nCameraInnerLeft  = mPtrDataset->getPtrCamera()->CameraLeft().getInnerParam();
            const TpCameraInnerParam& nCameraInnerRight = mPtrDataset->getPtrCamera()->CameraRight().getInnerParam();
            
            //cout << "Feature Matching ..." <<endl;
            const KeyPointManager::FrameMatchResult& mFrameMatchResult = mKeyPointMgr.solve(mCurFrame);
            //cout << "Feature Matching Finish." <<endl;
            
            mCoVisMgr.solve(mCurFrame, mFrameMatchResult);
            
            auto& mMatchResult      = mFrameMatchResult.getInnerFrameDescriptorMatchResult();
            auto& mKptsDescriptors  = mKeyPointMgr.getDescriptor(mCurFrame.FrameID());
            
            
            TpVecMatchResult nVecMatchResultWithParall;
            TpVecMatchResult nVecMatchResultWithoutParall;
            
#if 1
            cout << "Parallax on Stereo Frame :" <<endl;
            for(int nIdxMatch=0,nSzMatch=mMatchResult.get().size();nIdxMatch<nSzMatch;++nIdxMatch){
                TpKeyPointIndex nKptIdxLeft, nKptIdxRight;
                mMatchResult.getMatchKptIndex(nIdxMatch, nKptIdxLeft, nKptIdxRight);
                TpKeyPoint nKptLeft  = mKptsDescriptors.mKeyPointsLeft[nKptIdxLeft];
                TpKeyPoint nKptRight = mKptsDescriptors.mKeyPointsRight[nKptIdxRight];
                TpKeyPoint nKptLeftUndistor = nCameraInnerLeft.undistor(nKptLeft);
                TpKeyPoint nKptRightUndistor = nCameraInnerRight.undistor(nKptRight);
                auto nPtNormLeft  = nCameraInnerLeft.cvtPixelToNorm(nKptLeftUndistor.pt);
                auto nPtNormRight = nCameraInnerRight.cvtPixelToNorm(nKptRightUndistor.pt);
                
                cv::Vec3f nMapPointLeft;
                bool bTriangular; 
                bTriangular = (nKptRightUndistor.pt.x - nKptLeftUndistor.pt.x) < -1e-3;
                //bool bTriangular2 = Tools::triangulation(nPtNormLeft, nPtNormRight, nPrTPl, nMapPointLeft);
                //cout << nKptIdxLeft<<"-" << nKptRightUndistor.pt-nKptLeftUndistor.pt;
                if(bTriangular){
                    nVecMatchResultWithParall.push_back(mMatchResult.get()[nIdxMatch]);
                }else{
                    nVecMatchResultWithoutParall.push_back(mMatchResult.get()[nIdxMatch]);
                }
            }
            //cout <<endl;
            //Tools::drawMatch(mCurFrame.getImage(), mKptsDescriptors.mKeyPointsLeft, dynamic_cast<StereoFrame&>(mCurFrame).getImageRight(), mKptsDescriptors.mKeyPointsRight, nVecMatchResultWithParall, true, "Parall" );
            //Tools::drawMatch(mCurFrame.getImage(), mKptsDescriptors.mKeyPointsLeft, dynamic_cast<StereoFrame&>(mCurFrame).getImageRight(), mKptsDescriptors.mKeyPointsRight, nVecMatchResultWithoutParall, true, "NoParall" );
            //cv::waitKey();
#endif
            
            
            KeyPointManager::TpOneFrameIDManager& mOneFrameIDMgr = mCoVisMgr.OneFrameKptIDMgrByFrameID(mCurFrame.FrameID());
            mPtrKeyFrameMgr->solve(mCurFrame, mFrameMatchResult, mOneFrameIDMgr);
            
            debugCountTrackingKptIDWihtMapPointID(mCurFrame, mFrameMatchResult, mOneFrameIDMgr);
            
            if(mPtrKeyFrameMgr->isKeyFrame(mCurFrame.FrameID())){
                auto& nMapPointIDMgr = mPtrKeyFrameMgr->getMapPointIDManager();
                TpVecMapPointID nVecMapPointID = nMapPointIDMgr.getMapPointIDsGeneratedByFrame(mCurFrame.FrameID());
                map<TpKeyPointIndex, TpMapPointID> nMapKptIdx2MpID;
                for(int nIdxMapPointID=0;nIdxMapPointID<nVecMapPointID.size();++nIdxMapPointID){
                    TpMapPointID nMapPointIDNew = nVecMapPointID[nIdxMapPointID];
                    // triangulation;
                    auto nFrmIDAndKptIdx = nMapPointIDMgr.MapPoint(nMapPointIDNew).getVecMeasurments()[0];
                    TpKeyPointIndex nKptIdxLeft = nFrmIDAndKptIdx.second;
                    nMapKptIdx2MpID[nKptIdxLeft] = nMapPointIDNew;
                }
                
                const TpVecKeyPoints& nVecKptLeft = mKeyPointMgr.getDescriptor(mCurFrame.FrameID()).mKeyPointsLeft;
                const TpVecKeyPoints& nVecKptRight = mKeyPointMgr.getDescriptor(mCurFrame.FrameID()).mKeyPointsRight;
                const TpVecMatchPairs& nInnerMatch = mKeyPointMgr.getFrameMatchResult().getInnerFrameDescriptorMatchResult().getMatchPairs();
                cout << "For Key Point triangular on KF" <<endl;
                for(int nIdxMatch=0,nSzMatch = nInnerMatch.size();nIdxMatch<nSzMatch;++nIdxMatch){
                    const TpMatchPair& nMatchPair = nInnerMatch[nIdxMatch];
                    const TpKeyPointIndex& nMatchKptIdxLeft  = nMatchPair.first;
                    const TpKeyPointIndex& nMatchKptIdxRight = nMatchPair.second;
                    auto Iter = nMapKptIdx2MpID.find(nMatchKptIdxLeft);
                    if(Iter==nMapKptIdx2MpID.end())
                        continue;
                    const TpMapPointID& nMpID = Iter->second;
                    KeyFrameManager::TpMapPoint& nMp =  nMapPointIDMgr.MapPoint(nMpID);
                    TpKeyPoint nKptLeftUndistor  = nCameraLeft.getInnerParam().undistor(nVecKptLeft[nMatchKptIdxLeft]);
                    TpKeyPoint nKptRightUndistor = nCameraRight.getInnerParam().undistor(nVecKptRight[nMatchKptIdxRight]);
                    
                    float nDepthInLeftView = 0;
                    if(Tools::triangulation(nKptLeftUndistor.pt, nKptRightUndistor.pt, 435.262, 47.912663, nDepthInLeftView)){
                        TpKeyPoint nKptLeftUndistorNorm  = nCameraLeft.getInnerParam().cvtPixelToNorm(nKptLeftUndistor);
                        cv::Vec3f  nMp3DLeftView(nKptLeftUndistorNorm.pt.x*nDepthInLeftView,
                                                 nKptLeftUndistorNorm.pt.y*nDepthInLeftView, nDepthInLeftView);
                        auto nMp3DWorld = mPtrKeyFrameMgr->getFrameCameraPose(mCurFrame.FrameID())->cvtToWorld(nMp3DLeftView);
                        nMp.initMapPoint3D(cv::Point3f(nMp3DWorld));
                        cout << "Pixel Left|Right - PtInWorld(LeftView): " << nKptLeftUndistor.pt << "|" << nKptRightUndistor.pt << "  - "<< nMp3DWorld <<endl;
                    }
                }
                //cout <<endl;
            }else{
                // non-kf observe should insert to the mappoint?
            }
            
            cv::Matx44f nFramePoseCur = solverCurrentFramePose(mCurFrame.FrameID());
            mPtrCameraPoseCurFrame = mPtrKeyFrameMgr->getFrameCameraPose(mCurFrame.FrameID());
            
            cv::Mat mImgToShow = mCurFrame.Image().clone();
            
            if(!mImgToShow.empty()){
                mTrackingImageCurFrame = mImgToShow;
                //cv::imshow("Viewer", mImgToShow);
                //cv::waitKey(mbRunAllFrame?40:1);
            }
            
            if(!mbRunAllFrame){
                break;
            }
        }
    };
    
    mPtrFuncDoExec = FuncDorunVIO;
    mbRunAllFrame = bRunAllFrame;
}

cv::Matx44f System::solverCurrentFramePose(const TpFrameID nFrameIDCur) {
    cv::Matx44f nFramePoseCur = cv::Matx44f::eye();
    if(nFrameIDCur == 0)    // TODO Fist Frame, need try other way.
        return nFramePoseCur;
    
    TpVecKeyPointID nVecKptIDs; TpVecKeyPointIndex nVecKptIndexs;
    KeyPointManager::TpOneFrameIDManager& nFrameIDManagerCur = mCoVisMgr.OneFrameKptIDMgrByFrameID(nFrameIDCur);
    nFrameIDManagerCur.getAllKptIDsAndIdexs(nVecKptIDs, nVecKptIndexs);
    
    KeyFrameManager::TpFrameKptIDMapPointPair nFrameKptIDMapPointPair(nFrameIDCur);
    mPtrKeyFrameMgr->getKptIDsWithMapPointID(nFrameIDManagerCur, nFrameKptIDMapPointPair);
    
    map<TpKeyPointID, TpMapPointID> nMapKeyPointID2MapPointID = nFrameKptIDMapPointPair.getMapKptID2MapPointID();
    vector<KeyFrameManager::TpKptIDMapPointPairWithFrameID> nVecKptIDMapPointPairWithFrameID;
    auto FuncGetCoVisWithCurrentFrame = [&](const TpFrameID nCosVisFrmID) {
            if(nFrameIDCur - nCosVisFrmID>DebugManager::getMaxCoVisLength()) return;
        
            TpVecKeyPointID nVecKptIDs; TpVecKeyPointIndex nVecKptIndexs;
            mCoVisMgr.OneFrameKptIDMgrByFrameID(nCosVisFrmID).getAllKptIDsAndIdexs(nVecKptIDs, nVecKptIndexs);
            for(int nIdxKptID=0,nSzKptIDs=nVecKptIDs.size();nIdxKptID<nSzKptIDs;++nIdxKptID){
                const TpKeyPointID nKptID       = nVecKptIDs[nIdxKptID];
                auto IterFind = nMapKeyPointID2MapPointID.find(nKptID);
                if(IterFind == nMapKeyPointID2MapPointID.end())
                    continue;
                
                // get one co-vis
                const TpKeyPointIndex nKptIdex  = nVecKptIndexs[nIdxKptID];
                KeyFrameManager::TpKptIDMapPointPairWithFrameID nOneMesurement(nCosVisFrmID, nKptID, nKptIdex, IterFind->second);
                nVecKptIDMapPointPairWithFrameID.push_back(nOneMesurement);
            }
        };
    mCoVisMgr.collectCoVisInfo(nFrameIDCur, FuncGetCoVisWithCurrentFrame, DebugManager::DebugControl().mCountCoVis);
    
    // Another way is (1)get all MapPointIDs in Current Frame, (2) get all measurments stored in TpMapPoint
    //      how to solve the measurment from non-keyframe, directly added in TpMapPoint, which still is not done in runVIO?
    
    // Through nVecKptIDMapPointPairWithFrameID get measurment Info: camera pose 6D, keypoint pixel 2D, mapoint 3D.
    // TODO
    map<TpFrameID, cv::Mat>             nMapFrameID2FrameImage;
    map<TpFrameID, TpPtrCameraPose>     nMapFrameID2CameraPose;
    map<TpMapPointID, TpPtrMapPoint3D>  nMapMapPointID2MapPoint3D;
    Solver::TpVecVisualMeasurement      nVecVisualMeasurement;
    
    for(int nIdxMeasurement=0,nSzMeasurements = nVecKptIDMapPointPairWithFrameID.size();nIdxMeasurement<nSzMeasurements;++nIdxMeasurement){
        auto& nMeasurement = nVecKptIDMapPointPairWithFrameID[nIdxMeasurement];
        if(nMapFrameID2CameraPose.find(nMeasurement.mFrameID) == nMapFrameID2CameraPose.end()){
            nMapFrameID2CameraPose[nMeasurement.mFrameID] = mPtrKeyFrameMgr->getFrameCameraPose(nMeasurement.mFrameID);
            nMapFrameID2FrameImage[nMeasurement.mFrameID] = mKeyPointMgr.getStereoFrameHistory().get(nMeasurement.mFrameID).getImageLeft().clone();
        }
        if(nMapMapPointID2MapPoint3D.find(nMeasurement.mMapPointID) == nMapMapPointID2MapPoint3D.end()){
            //nMapMapPointID2MapPoint3D[nMeasurement.mMapPointID] = TpMapPoint3D();
            const KeyFrameManager::TpMapPoint& nMp = mPtrKeyFrameMgr->getMapPointIDManager().MapPoint(nMeasurement.mMapPointID);
            if(!nMp.getMapPoint3DValid())
                continue;
            nMapMapPointID2MapPoint3D[nMeasurement.mMapPointID] = nMp.getMapPoint3D();
        }
    }
    
    Type::TpPtrCameraStereo nPtrCameraStereo = mPtrDataset->getPtrCamera();
    
    
    for(int nIdxMeasurement=0,nSzMeasurements = nVecKptIDMapPointPairWithFrameID.size();nIdxMeasurement<nSzMeasurements;++nIdxMeasurement){
        auto& nMeasurement = nVecKptIDMapPointPairWithFrameID[nIdxMeasurement];
        TpFrameID nFrameID = nMeasurement.mFrameID;
        if(mKeyPointMgr.getFrameKptsDescriptorHistory().isExisting(nFrameID)){
            
            cv::KeyPoint nKeyPoint = mKeyPointMgr.getFrameKptsDescriptorHistory().get(nFrameID).mKeyPointsLeft[nMeasurement.mKptIndex];
            //TpMapPoint3D nMapPoint = mPtrKeyFrameMgr->getMapPointIDManager().MapPoint(nMeasurement.mMapPointID).MapPoint3D();
            cv::KeyPoint nKeyPointUnDistor = mPtrDataset->getPtrCamera()->CameraLeft().getInnerParam().undistor(nKeyPoint);
            if(nMapMapPointID2MapPoint3D.find(nMeasurement.mMapPointID)==nMapMapPointID2MapPoint3D.end())
                continue;
            
            Solver::TpVisualMeasurement nVisualMeasurement;
            nVisualMeasurement.mFrameID     = nFrameID;
            nVisualMeasurement.mMapPointID  = nMeasurement.mMapPointID;
            //nVisualMeasurement.mKeyPoint  = nKeyPoint;
            nVisualMeasurement.mKeyPoint    =  nKeyPointUnDistor;
            
            //nSolverCurFramePose.addMeasurement(nVisualMeasurement);
            nVecVisualMeasurement.push_back(nVisualMeasurement);
        }else{
            cout << "Error: FrameKptsDescriptorHistory has been freed, FrameID:" << nFrameID<<endl;
            throw;
        }
    }
#if 0
    
    {
        
    bool bShowAll = false;
    for(auto Iter = nMapFrameID2CameraPose.begin();Iter!=nMapFrameID2CameraPose.end() && bShowAll;++Iter){
        auto nFrameIDCoVis = Iter->first;
        Tools::drawMatch(
            mKeyPointMgr.getStereoFrameHistory().get(nFrameIDCur).getImageLeft(),
            mKeyPointMgr.getDescriptor(nFrameIDCur).mKeyPointsLeft,
            mKeyPointMgr.getStereoFrameHistory().get(nFrameIDCoVis).getImageLeft(),
            mKeyPointMgr.getDescriptor(nFrameIDCoVis).mKeyPointsLeft,
            mCoVisMgr.getCoVis(nFrameIDCur, nFrameIDCoVis), 
            true, "CoVis"+cvtToString(nFrameIDCur)+"|"+cvtToString(nFrameIDCoVis));
        
        bShowAll = cv::waitKey() != 27;
    }
    cv::destroyAllWindows();
    }
    
    for(int nIdxVm=0,nSzVm=nVecVisualMeasurement.size();nIdxVm<nSzVm;++nIdxVm){
        const Solver::TpVisualMeasurement& nVm = nVecVisualMeasurement[nIdxVm];
        if(nMapFrameID2FrameImage.find(nVm.mFrameID) == nMapFrameID2FrameImage.end()){
            cout << "Error: " << nVm.mFrameID<<endl;
        }
        Tools::drawKeyPoints(nMapFrameID2FrameImage[nVm.mFrameID], TpVecKeyPoints(1, nVm.mKeyPoint));
        cv::putText(nMapFrameID2FrameImage[nVm.mFrameID], cvtToString(nVm.mMapPointID), nVm.mKeyPoint.pt, 1, 1, cv::Scalar(255));
    }
    {
    bool bShowAll = false;
    for(auto Iter = nMapFrameID2CameraPose.begin();Iter!=nMapFrameID2CameraPose.end() && bShowAll;++Iter){
        auto nFrameIDCoVis = Iter->first;
        cout << "Disp: " << nFrameIDCoVis << " - " << nFrameIDCur <<endl;
        Tools::showImageIfTitleNotEmpty(nMapFrameID2FrameImage[nFrameIDCoVis], "Vm"+cvtToString(nFrameIDCur)+"|"+cvtToString(nFrameIDCoVis));
        bShowAll = cv::waitKey() != 27;
    }
    cv::destroyAllWindows();
    }
#endif
    
    
    Solver::Solver nSolverCurFramePose;
    //nSolverCurFramePose.initCamerPoses(nMapFrameID2CameraPose);
    //nSolverCurFramePose.initMapPoints(nMapMapPointID2MapPoint3D);
    nSolverCurFramePose.solve(nMapFrameID2CameraPose, nMapMapPointID2MapPoint3D, nVecVisualMeasurement, nPtrCameraStereo);
    auto nPtrCameraPoseCur = nMapFrameID2CameraPose[nFrameIDCur];
    nFramePoseCur = nPtrCameraPoseCur->getMatx44f();
    //cout << "nFramePoseCur:" << nFrameIDCur <<endl << nFramePoseCur <<endl;
    cout    << "nFramePoseCur | nCosVisFrm | nMeasurement - "
            << nFrameIDCur << " | " << nMapFrameID2CameraPose.size() << " | " << nVecVisualMeasurement.size() <<endl 
            << nPtrCameraPoseCur->getPosition() <<endl;
    
    return nFramePoseCur;
}
}

}


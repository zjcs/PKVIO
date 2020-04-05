#include <iostream>
#include <opencv2/highgui.hpp>
#include "System.h"
#include "../DatasetManager/DatasetEuRoc.h"
#include "../Tools/Tools.h"


namespace PKVIO {
namespace System {

void System::exec(void) {
    initialize();
    doexec();
    exit();
    
}

void System::initialize(void) {
    //mPtrDataset = std::make_shared<DatasetEuRoc>("E:/DataSet/MH_01_easy/");
    string sDatasetPath = std::string("/home/ubuntu/work/dataset/DataSet/MH_01_easy/");
    
    mPtrDataset = std::make_shared<DatasetEuRoc>(sDatasetPath);
    mPtrDataset->initialize();
    
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
    
    auto FuncDorunVIO = [&](){
        for (int nIndex = 0; !mPtrDataset->isFinished(); ++nIndex) {
            Frame& mCurFrame = mPtrDataset->read();
            
            
            if(mCurFrame.getImage().empty() && DatasetManager::isOfflineDatasetType(mPtrDataset->type())){
                FrameInfo mFrmInfo = dynamic_cast<DatasetOfflineImp*>(mPtrDataset.get())->getFrameInfor(mCurFrame.FrameID());
                cout<< "Empyt:" << mFrmInfo.mFrameIndex << " - " << mFrmInfo.mStrFileName <<endl;
            }
            
            //cout << "Feature Matching ..." <<endl;
            const KeyPointManager::FrameMatchResult& mFrameMatchResult = mKeyPointMgr.solve(mCurFrame);
            //cout << "Feature Matching Finish." <<endl;
            
            //cout << "CoVis Graph ..." <<endl;
            mCoVisMgr.solve(mCurFrame, mFrameMatchResult);
            //cout << "CoVis Graph Finish." <<endl;
            
            KeyPointManager::TpOneFrameIDManager& mOneFrameIDMgr = mCoVisMgr.getFrameKptIDMgr(mCurFrame.FrameID());
            mPtrKeyFrameMgr->solve(mCurFrame, mFrameMatchResult, mOneFrameIDMgr);
            
            debugCountTrackingKptIDWihtMapPointID(mCurFrame, mFrameMatchResult, mOneFrameIDMgr);
            
            if(mPtrKeyFrameMgr->isKeyFrame(mCurFrame.FrameID())){
                // non-kf observe should insert to the mappoint.
            }else{
                
            }
            
            cv::Mat& mImgToShow = mCurFrame.getImage();
            if(mKeyPointMgr.queryDescriptorExisting(mCurFrame.FrameID())){
                KeyPointManager::TpOneFrameKptDescriptor& CurFrmKptsDescriptor = mKeyPointMgr.getDescriptor(mCurFrame.FrameID());
                mImgToShow = Tools::drawKeyPoints(mImgToShow, CurFrmKptsDescriptor.mKeyPointsLeft);
            }
            if(!mImgToShow.empty()){
                cv::imshow("Viewer", mImgToShow);
                cv::waitKey(40);
            }
        }
    };
    
    mPtrFuncDoExec = FuncDorunVIO;
    exec();
}


void System::debugCountTrackingKptIDWihtMapPointID(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    int nCurCountKptIDsWithMapPoint = mPtrKeyFrameMgr->countTrackKptIDsWithMapPointID(mFrameKptIDMgr);
    static int nPrevCountKptIDsWithMapPoint = 0;
    
    if(mPtrKeyFrameMgr->isKeyFrame(fFrame.FrameID())) {
        
    } else {
        if(nCurCountKptIDsWithMapPoint>nPrevCountKptIDsWithMapPoint) {
            
            cout << "**** Error: Match Point number should decreas... Prev|Cur - " << nPrevCountKptIDsWithMapPoint <<" | " << nCurCountKptIDsWithMapPoint << endl;
            
            
            // draw the match and debug.
            cv::Mat mFrameImgCurLeft = fFrame.getImage();
            auto& mMatchREsult = mFrameMatchResult.getOuterFrameDescriptorMatchResult(0);
            KeyPointManager::StereoFrameHistory& nFrameHistory = mKeyPointMgr.getStereoFrameHistory();
            assert(fFrame.FrameID() == mMatchREsult.getFrameIDRight());
            if(nFrameHistory.isExisting(mMatchREsult.getFrameIDLeft())){
                Frame& fPrevFrame = nFrameHistory.get(mMatchREsult.getFrameIDLeft());
                assert(fPrevFrame.FrameID() == mMatchREsult.getFrameIDLeft());
                cv::Mat mFrameImgPrevLeft = fPrevFrame.getImage();
                //mKeyPointMgr.getDescriptor();
                //mMatchREsult.getMatch();
            }
        }
    }
    nPrevCountKptIDsWithMapPoint = nCurCountKptIDsWithMapPoint;
}

}

}


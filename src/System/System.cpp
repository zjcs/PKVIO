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
            
            mKeyPointMgr.solve(mCurFrame);
            
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

}

}


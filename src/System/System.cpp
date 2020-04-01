#include "System.h"
#include "../DatasetManager/DatasetEuRoc.h"
#include <iostream>
#include <opencv2/highgui.hpp>

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
    for (int nIndex = 0; !mPtrDataset->isFinished(); ++nIndex) {
        Frame& mCurFrame = mPtrDataset->read();
        cv::imshow("Viewer", mCurFrame.getImage());
        cv::waitKey(40);
    }
}

}

}


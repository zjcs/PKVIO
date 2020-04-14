#ifndef __DATASETSIMULATOR_H__
#define __DATASETSIMULATOR_H__

#include "DatasetEuRoc.h"

namespace PKVIO
{
namespace DatasetManager
{
class DatasetSimulator: public DatasetEuRoc{
public:
    DatasetSimulator(const string& sDatasetPath):DatasetEuRoc(sDatasetPath){}
    
protected:
    virtual void     getImage(int nIndexToRead, cv::Mat& mLeft, cv::Mat& mRight) override {
        auto& nInnerParam = getPtrCamera()->getCameraLeft().getInnerParam();
        int nW = nInnerParam.getImageSize().width, nH = nInnerParam.getImageSize().height;
        mLeft  = cv::Mat(nInnerParam.getImageSize(), CV_8UC1, cv::Scalar(255));
        mRight = cv::Mat(nInnerParam.getImageSize(), CV_8UC1, cv::Scalar(255));
        cv::Mat nBlack = cv::Mat(cv::Size(nW-2,nH-3), CV_8UC1, cv::Scalar(0));
        nBlack.copyTo(mLeft(cv::Rect(1,1,nW-2,nH-3)));
        nBlack.copyTo(mRight(cv::Rect(1,1,nW-2,nH-3)));
        //nBlack.copyTo(mRight(cv::Rect(1,1,nW-2,nH-2)))));
        /*
        cv::line(mLeft, cv::Point2f(0,0), cv::Point2f(nW-1,0), cv::Scalar(255), 1);
        cv::line(mLeft, cv::Point2f(0,nH-1), cv::Point2f(nW-1,nH-1), cv::Scalar(255), 1);
        cv::line(mLeft, cv::Point2f(nW-1,0), cv::Point2f(nW-1,nH), cv::Scalar(255), 1);
        cv::line(mLeft, cv::Point2f(0,0), cv::Point2f(0,nH), cv::Scalar(255), 1);
        */
    }
    virtual bool        isFinished(void) override{return false;}
};
}
}

#endif

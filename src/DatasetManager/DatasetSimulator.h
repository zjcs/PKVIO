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
        mLeft  = cv::Mat(nInnerParam.getImageSize(), CV_8UC1, cv::Scalar(0));
        mRight = cv::Mat(nInnerParam.getImageSize(), CV_8UC1, cv::Scalar(0));
    }
    virtual bool        isFinished(void) override{return false;}
};
}
}

#endif

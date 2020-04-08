#ifndef __DATASETEUROC_H__
#define __DATASETEUROC_H__

#include "DatasetOfflineImp.h"
#include <string>
#include <opencv/cv.h>
#include "../Type/Camera.h"

using namespace std;

namespace PKVIO{
namespace DatasetManager{
class DatasetEuRoc: public DatasetOfflineImp{
public: 
    DatasetEuRoc(const string& sDatasetPath);
    
    virtual void      initialize(void) override;
    virtual Frame*    load(const int nIndexToRead) override;
    virtual TpDatasetType       type(void) override {return TpOfflineEuRoc;};
    
protected:
    virtual inline const TpTimeStamp&   getFrameTimeStamp(TpFrameIndex nFrmIndex){return mVecImageTimeStamp[nFrmIndex];}
    virtual const string                getFrameFileName(TpFrameIndex nFrmIndex){return Type::cvtTimeStampToString(getFrameTimeStamp(nFrmIndex))+".png";}
    virtual const string                getFrameAbsFileNmae(TpFrameIndex nFrmIndex, bool bTrueLeftFalseRight);
    
private:
    string            getLeftViewFolder(void);
    string            getRightViewFolder(void);
    string            getIMUFolder(void);
    
    void              getImage(int nIndexToRead, cv::Mat& mLeft, cv::Mat& mRight);
    void              getIMU(void);
    
    void              parseDatasetTimeStamp(void);
    
    void              parseCalibration(void);
    void              parseCalibrationIMU(const string& sIMUCalibFile);
    
protected:
    void              parseCalibrationCamera(const string& sCameraCalibFile, TpCameraInnerParam& nCameraInnerParam, TpCameraDistorParam& nCameraDistorParam, TpCameraOuterParam& nCameraOuterParam);
private:
    typedef pair<TpTimeStamp, cv::Vec6d>        TpInputIMU;
    vector<TpTimeStamp>                         mVecImageTimeStamp;
    vector<TpInputIMU>                          mVecIMUTimeStamp;
    
    Type::TpPtrCameraStereo                     mPtrCameraStereo;
};
}
}

#endif

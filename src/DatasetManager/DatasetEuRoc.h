#ifndef __DATASETEUROC_H__
#define __DATASETEUROC_H__

#include "DatasetOfflineImp.h"
#include <string>
#include <opencv/cv.h>
#include "../Type/Camera.h"

class TpStereoRectResult{
public:
    TpStereoRectResult(
    cv::Mat nR,cv::Mat nT,cv::Mat  nR1,cv::Mat nR2,cv::Mat nP1,cv::Mat nP2,cv::Mat nQ, cv::Size nSz
    ):R(nR), T(nT), R1(nR1), R2(nR2), P1(nP1), P2(nP2), Q(nQ), Sz(nSz){}
    cv::Mat R,T, R1,R2,P1,P2,Q;
    cv::Size Sz;
};

using namespace std;

namespace PKVIO{
namespace DatasetManager{
class DatasetEuRoc: public DatasetOfflineImp{
public: 
    DatasetEuRoc(const string& sDatasetPath);
    
    virtual void                        initialize(void) override;
    virtual Frame*                      load(const int nIndexToRead) override;
    virtual TpDatasetType               type(void) override {return TpOfflineEuRoc;};
    virtual const TpPtrCameraStereo     getPtrCamera(void)const override{return mPtrCameraStereo;}
    
    TpStereoRectResult*  mStereoRectResult;
protected:
    virtual inline const TpTimeStamp&   getFrameTimeStamp(TpFrameIndex nFrmIndex){return mVecImageTimeStamp[nFrmIndex];}
    virtual const string                getFrameFileName(TpFrameIndex nFrmIndex){return Type::cvtTimeStampToString(getFrameTimeStamp(nFrmIndex))+".png";}
    virtual const string                getFrameAbsFileNmae(TpFrameIndex nFrmIndex, bool bTrueLeftFalseRight);
    
    void              remapInputStereoFrame(StereoFrame* pStereoFrame);
    
    const bool        getUseUndistorInput(void)const{return mBoolUseUndistorCamera;}
    
    virtual void      getImage(int nIndexToRead, cv::Mat& mLeft, cv::Mat& mRight);
private:
    string            getLeftViewFolder(void);
    string            getRightViewFolder(void);
    string            getIMUFolder(void);
    
    void              getIMU(void);
    
    void              parseDatasetTimeStamp(void);
    
    void              parseCalibration(void);
    void              parseCalibrationIMU(const string& sIMUCalibFile);
    
protected:
    void              parseCalibrationCamera(const string& sCameraCalibFile, TpCameraInnerParam& nCameraInnerParam,
                                             TpCameraOuterParam& nCameraOuterParam);
private:
    typedef pair<TpTimeStamp, cv::Vec6d>        TpInputIMU;
    vector<TpTimeStamp>                         mVecImageTimeStamp;
    vector<TpInputIMU>                          mVecIMUTimeStamp;
    
    Type::TpPtrCameraStereo                     mPtrCameraStereo;
    
    bool                                        mBoolUseUndistorCamera;
};
}
}

#endif

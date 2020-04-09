#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "opencv2/core/types.hpp" 
#include "opencv2/imgproc.hpp"

#include <iostream>

using namespace std;

namespace PKVIO
{
namespace Type
{
    
typedef cv::Matx44f TpTranslate;

template<typename T>
cv::Vec2f cvtPointToVec(const T& nPt){
    return cv::Vec2f(nPt.x, nPt.y);
}

template<typename T>
const cv::Point2f cvtVecToPoint(const T& nPt){
    return cv::Point2f(nPt(0), nPt(1));
}

template<typename T>
const cv::Mat cvtPointToMat(const T& nPt){
    cv::Mat nMat  = cv::Mat(1, 1, CV_32FC2);
    nMat.at<cv::Point2f>(0,0) = nPt;
    return nMat;
}

template<typename T>
const cv::Mat cvtMatToPoint(const T& nPt){
    cv::Mat nMat  = cv::Mat(1, 1, CV_32FC2);
    nMat.at<cv::Vec2f>(0,0) = nPt;
    return nMat;
}


class TpCameraInnerParam
{
public:
    TpCameraInnerParam():fx(0),fy(0),cx(0),cy(0),wx(0),hy(0),mParam(){}
    TpCameraInnerParam(const cv::Vec2i& wxy, const cv::Vec4f& nfxycxy, const cv::Vec4f& nParam)
    : fx(nfxycxy[0]),fy(nfxycxy[1]),cx(nfxycxy[2]),cy(nfxycxy[3]),wx(wxy[0]),hy(wxy[1]), mParam(nParam)
    {}
    cv::Point2f cvtPixelToNorm(const cv::Point2f& nPixel) const {
        cv::Point2f nNorm;
        nNorm.x = (nPixel.x - cx)/wx;
        nNorm.y = (nPixel.y - cy)/hy;
        throw;
        return nNorm;
    }
    
    cv::Point2f cvtNormToPixel(const cv::Point2f& nNorm) const {
        cv::Point2f nPixel;
        //nNorm.x = (nPixel.x - cx)/wx;
        //nNorm.y = (nPixel.y - cy)/hy;
        throw;
        return nPixel;
    }
    
    inline const cv::KeyPoint undistor(cv::KeyPoint xDistored)const{
        cv::KeyPoint xUndistored = xDistored;
        xUndistored.pt = undistor(xDistored.pt);
        return xUndistored;
    }
    
    inline vector<cv::KeyPoint> undistor(const vector<cv::KeyPoint>& nVecKptDistort)const{
        vector<cv::KeyPoint> nVecKptUnDistort;
        nVecKptUnDistort.reserve(nVecKptDistort.size());
        for(size_t nIdxKpt=0,nSzKpts=nVecKptDistort.size();nIdxKpt<nSzKpts;++nIdxKpt){
            nVecKptUnDistort.push_back(undistor(nVecKptDistort[nIdxKpt]));
        }
        return nVecKptUnDistort;
    }
    
    inline const cv::Point2f undistor(const cv::Point& xDistored)const{
        //cv::Vec2f xU = cvtPointToVec(xDistored);
        //cv::Mat nImgDistort  = cv::Mat(1, 1, CV_32FC2);
        //nImgDistort.at<cv::Vec2f>(0,0) = xU;
        
        cv::Mat nImgDistort  = cvtPointToMat(xDistored);
        cv::Mat nImgUnDistort  = cv::Mat(1, 1, CV_32FC2);
        cv::undistortPoints(nImgDistort, nImgUnDistort, getInnerMat(), mParam, cv::noArray(), getInnerMat());
        //cv::undistortPoints(cvtPointToVec(xDistored), xUndistored, getInnerMat(), mParam);
        cv::Vec2f xUndistored;
        xUndistored = nImgUnDistort.at<cv::Vec2f>(0,0);
        return cvtVecToPoint(xUndistored);
    }
    
    inline cv::Mat undistor(const cv::Mat& nImgDistort)const{
        cv::Mat nImgUndistort;
        cv::undistort(nImgDistort, nImgUndistort, getInnerMat(), mParam);
        return nImgUndistort;
    }
    
    inline const cv::Point2f distor(cv::Point xUnDistored)const{
        
    }
    const float getfx() const {return fx;}
    const float getfy() const {return fy;}
    const float getcx() const {return cx;}
    const float getcy() const {return cy;}
    const float getwx() const {return wx;}
    const float gethy() const {return hy;}
    const cv::Mat   getInnerMat(void) const {
        cv::Matx33f nInnerMat = cv::Matx33f::eye();
        nInnerMat(0,0) = getfx(); nInnerMat(1,1) = getfy();
        nInnerMat(0,2) = getcx(); nInnerMat(1,2) = getcy();
        return cv::Mat(nInnerMat);
    }
private:
    float fx,fy,cx,cy,wx,hy;
    cv::Mat mParam;
};

class TpCameraOuterParam
{
public:
    TpCameraOuterParam():mcTw(cv::Matx44f::eye()){}
    TpCameraOuterParam(const TpTranslate& ncTw):mcTw(ncTw){}
    cv::Matx44f         get(void){return mcTw;}
    cv::Mat             getMat(void){return cv::Mat(mcTw);}
    void                set(const cv::Matx44f& ncTw){mcTw = ncTw;}
    void                set(const cv::Mat& ncTw){mcTw = ncTw;}
    TpCameraOuterParam  getWise(void) const {return mcTw.t();}
private:
    TpTranslate mcTw;
};
    
class Camera
{
public: 
    Camera(){}
    Camera(const TpCameraInnerParam&  nCameraInnerParam)
    : mCameraInnerParam(nCameraInnerParam){}
    
    Camera& operator=(const Camera& cp){
        this->mCameraInnerParam = cp.mCameraInnerParam;
        return *this;
    }
    
    virtual ~Camera(){}
    inline  const TpCameraInnerParam& getCameraInnerParam(int nIndexCamera = 0) const {return mCameraInnerParam;}
    virtual const TpCameraOuterParam  getCameraOuterParam(int nIndexCameraFrom =0,int nIndexCameraTo =1) const {return TpCameraOuterParam();}
private:
    TpCameraInnerParam  mCameraInnerParam;
};

class CameraMono: public Camera{
public:
private:
};

class CameraStereo :public Camera
{
public:
    CameraStereo(){}
    
    Camera&       CameraLeft(void){return *this;}
    Camera&       CameraRight(void){return this->mCameraRight;}
    
    const Camera&       getCameraLeft(void)const{return *this;}
    const Camera&       getCameraRight(void)const{return this->mCameraRight;}
    
    inline void         setCameraOuterParamCvtPtLViewToRView(const TpCameraOuterParam& n){mCameraOuterParamCvtPtLViewToRView = n;}
    
    // getTranslateConvertPointInLeftViewToPointInRightView
    inline const TpCameraOuterParam getTranslateCvtPtLViewToRView(void) const {
        return mCameraOuterParamCvtPtLViewToRView;
    }
    // getTranslateConvertPointInLeftViewToPointInRightView
    inline const TpCameraOuterParam getTranslateCvtPtRViewToLView(void) const {
        return mCameraOuterParamCvtPtLViewToRView.getWise();
    }
    
    virtual const TpCameraOuterParam  getCameraOuterParam(int nIndexCameraPtFrom =0,int nIndexCameraPtTo =1) const override {
        if(nIndexCameraPtFrom == 0 && nIndexCameraPtTo == 1){
            return getTranslateCvtPtLViewToRView();
        }else if(nIndexCameraPtFrom == 1 && nIndexCameraPtTo == 0){
            return getTranslateCvtPtRViewToLView();
        }else{
            TpCameraOuterParam();
        }
    }
private:
    Camera              mCameraRight;
    TpCameraOuterParam  mCameraOuterParamCvtPtLViewToRView;
};
    
typedef std::shared_ptr<CameraStereo> TpPtrCameraStereo;

}
}

#endif

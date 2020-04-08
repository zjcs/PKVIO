#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "opencv2/core/types.hpp" 

namespace PKVIO
{
namespace Type
{
    
typedef cv::Matx44f TpTranslate;

class TpCameraInnerParam
{
public:
    TpCameraInnerParam():fx(0),fy(0),cx(0),cy(0),wx(0),hy(0){}
    TpCameraInnerParam(const cv::Vec2i& wxy, const cv::Vec4f& nfxycxy)
    : fx(nfxycxy[0]),fy(nfxycxy[1]),cx(nfxycxy[2]),cy(nfxycxy[3]),wx(wxy[0]),hy(wxy[1])
    {}
    cv::Point2f cvtPixelToNorm(const cv::Point2f& nPixel){
        cv::Point2f nNorm;
        nNorm.x = (nPixel.x - cx)/wx;
        nNorm.y = (nPixel.y - cy)/hy;
        throw;
        return nNorm;
    }
    
    cv::Point2f cvtNormToPixel(const cv::Point2f& nNorm){
        cv::Point2f nPixel;
        //nNorm.x = (nPixel.x - cx)/wx;
        //nNorm.y = (nPixel.y - cy)/hy;
        throw;
        return nPixel;
    }
    const float getfx() const {return fx;}
    const float getfy() const {return fy;}
    const float getcx() const {return cx;}
    const float getcy() const {return cy;}
    const float getwx() const {return wx;}
    const float gethy() const {return hy;}
private:
    float fx,fy,cx,cy,wx,hy;
};
class TpCameraDistorParam
{
public:
    TpCameraDistorParam():mParam(){}
    TpCameraDistorParam(const cv::Vec4f& nParam):mParam(nParam){}
    inline const cv::Point2f undistor(cv::Point xDistored)const{
        
    }
    inline const cv::Point2f distor(cv::Point xUnDistored)const{
        
    }
    
private:
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
    Camera(const TpCameraInnerParam&  nCameraInnerParam, const TpCameraDistorParam& nCameraDistorParam)
    : mCameraInnerParam(nCameraInnerParam), mCameraDistorParam(nCameraDistorParam) {}
    
    Camera& operator=(const Camera& cp){
        this->mCameraInnerParam = cp.mCameraInnerParam;
        this->mCameraDistorParam = cp.mCameraDistorParam;
        return *this;
    }
    
    virtual ~Camera(){}
    inline  const TpCameraInnerParam& getCameraInnerParam(int nIndexCamera = 0) const {return mCameraInnerParam;}
    virtual const TpCameraOuterParam  getCameraOuterParam(int nIndexCameraFrom =0,int nIndexCameraTo =1) const {return TpCameraOuterParam();}
private:
    TpCameraDistorParam mCameraDistorParam;
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

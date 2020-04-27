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
    TpCameraInnerParam(float nfx,float nfy,float ncx,float ncy,float nwx,float nhy):fx(nfx),fy(nfy),cx(ncx),cy(ncy),wx(nwx),hy(nhy),mParam(){}
    TpCameraInnerParam(const cv::Vec2i& wxy, const cv::Vec4f& nfxycxy, const cv::Vec4f& nParam)
    : fx(nfxycxy[0]),fy(nfxycxy[1]),cx(nfxycxy[2]),cy(nfxycxy[3]),wx(wxy[0]),hy(wxy[1]), mParam(nParam)
    {}
    TpCameraInnerParam(const cv::Vec2i& wxy, const cv::Vec4f& nfxycxy)
    : fx(nfxycxy[0]),fy(nfxycxy[1]),cx(nfxycxy[2]),cy(nfxycxy[3]),wx(wxy[0]),hy(wxy[1]), mParam(cv::Mat())
    {}
    
    
    cv::KeyPoint cvtPixelToNorm(const cv::KeyPoint& nPixel) const {
        cv::KeyPoint nKpt = nPixel;
        nKpt.pt = cvtPixelToNorm(nPixel.pt);
        return nKpt;
    }
    // Info: this normalized coordination is not same to the openGL coordination, which is in [-1,1].
    cv::Point2f cvtPixelToNorm(const cv::Point2f& nPixel) const {
        cv::Point2f nNorm;
        nNorm.x = (nPixel.x - cx)/fx;
        nNorm.y = (nPixel.y - cy)/fy;
        return nNorm;
    }
    
    // Info: this normalized coordination is not same to the openGL coordination, which is in [-1,1].
    cv::Point2f cvtNormToPixel(const cv::Point2f& nNorm) const {
        cv::Point2f nPixel;
        nPixel.x = nNorm.x*fx + cx;
        nPixel.y = nNorm.y*fy + cy;
        return nPixel;
    }
    
    inline const cv::KeyPoint undistor(const cv::KeyPoint& xDistored)const{
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
    
    inline const cv::Point2f undistor(const cv::Point2f& xDistored)const{
        cv::Mat nImgDistort  = cvtPointToMat(xDistored);
        cv::Mat nImgUnDistort  = cv::Mat(1, 1, CV_32FC2);
        cv::undistortPoints(nImgDistort, nImgUnDistort, getInnerMat(), mParam, cv::noArray(), getInnerMat());
        return nImgUnDistort.at<cv::Point2f>(0,0);
    }
    
    inline cv::Mat undistor(const cv::Mat& nImgDistort)const{
        cv::Mat nImgUndistort;
        cv::undistort(nImgDistort, nImgUndistort, getInnerMat(), mParam);
        return nImgUndistort;
    }
    
    inline const cv::Point2f distor(const cv::Point2f& xUnDistored)const{
        cout << "Error: distor not imp" <<endl;
        throw;
    }
    
    inline const cv::Mat& getDistorParam(void) const {return mParam;}
    inline const cv::Mat getDistorParam64F(void) const {
        cv::Mat nMatParam(mParam.size(), CV_64FC1);
        if(!mParam.empty()){
            nMatParam.at<double>(0,0) = mParam.at<float>(0,0);
            nMatParam.at<double>(1,0) = mParam.at<float>(1,0);
            nMatParam.at<double>(2,0) = mParam.at<float>(2,0);
            nMatParam.at<double>(3,0) = mParam.at<float>(3,0);
        }
        return nMatParam;
    }
    
    const float getfx() const {return fx;}
    const float getfy() const {return fy;}
    const float getcx() const {return cx;}
    const float getcy() const {return cy;}
    const float getwx() const {return wx;}
    const float gethy() const {return hy;}
    const cv::Size getImageSize() const {return cv::Size(wx,hy);}
    
    const cv::Mat   getInnerMat64F(void) const {
        cv::Matx33d nInnerMat = cv::Matx33d::eye();
        nInnerMat(0,0) = getfx(); nInnerMat(1,1) = getfy();
        nInnerMat(0,2) = getcx(); nInnerMat(1,2) = getcy();
        return cv::Mat(nInnerMat);
    }
    const cv::Mat   getInnerMat(void) const {
        return cv::Mat(getInnerMatx33f());
    }
    
    const cv::Matx33f getInnerMatx33f(void) const {
        cv::Matx33f nInnerMat = cv::Matx33f::eye();
        nInnerMat(0,0) = getfx(); nInnerMat(1,1) = getfy();
        nInnerMat(0,2) = getcx(); nInnerMat(1,2) = getcy();
        return nInnerMat;
    }
    
    const string str(void)const{
        stringstream sStrStream;
        sStrStream<< "  Fxy|Cxy|WxHy: "<<fx<<","<<fy<<","<<cx<<","<<cy<<","<<wx<<","<<hy<<endl;
        sStrStream<< "  Distor Param: "<< mParam.t() << endl;
        return sStrStream.str();
    }
private:
    float fx,fy,cx,cy,wx,hy;
    cv::Mat mParam;
};

class TpCameraOuterParam
{
public:
    TpCameraOuterParam():mcTw(cv::Matx44f::eye()){}
    TpCameraOuterParam(const cv::Mat& R, const cv::Mat& T):mcTw(cv::Matx44f::eye()){
        cout << "Error: TpCameraOuterParam not imp" <<endl;
        throw;
    }
    TpCameraOuterParam(const TpTranslate& ncTw):mcTw(ncTw){}
    cv::Matx44f         get(void) const {return mcTw;}
    cv::Mat             getMat(void) const {return cv::Mat(mcTw);}
    void                set(const cv::Matx44f& ncTw){mcTw = ncTw;}
    void                set(const cv::Mat& ncTw){mcTw = ncTw;}
    TpCameraOuterParam  getWise(void) const {return mcTw.inv();}
    
    const string str(void) const {
        stringstream sStrStream;
        sStrStream << mcTw <<endl;
        return sStrStream.str();
    }
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
    
    inline  const TpCameraInnerParam& getInnerParam(int nIndexCamera = 0) const {return mCameraInnerParam;}
    //virtual const TpCameraOuterParam  getCameraOuterParam(int nIndexCameraFrom =0,int nIndexCameraTo =1) const {return TpCameraOuterParam();}
    inline const TpCameraOuterParam  getCameraOuterParam(int nIndexCameraFrom =0,int nIndexCameraTo =1) const {return mCameraOuterParam; }
    
    void setOuterParamCvtPrTPc(const TpCameraOuterParam& nCvtPtInRefenceToThisCamera) {mCameraOuterParam = nCvtPtInRefenceToThisCamera;}
    
    const string str() const {
        return mCameraInnerParam.str() + "\n" + " T- PcTPref    : \n"  + mCameraOuterParam.str();
    }
protected:
private:
    TpCameraInnerParam  mCameraInnerParam;
    TpCameraOuterParam  mCameraOuterParam;
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
    
    inline void         setCameraOuterParamCvtPtLViewToRView(const TpCameraOuterParam& n){
        // mCameraOuterParamCvtPtLViewToRView = n;
        CameraLeft().setOuterParamCvtPrTPc(TpCameraOuterParam());
        CameraRight().setOuterParamCvtPrTPc(n);
    }
    
    // getTranslateConvertPointInLeftViewToPointInRightView
    inline const TpCameraOuterParam getTranslateCvtPtLViewToRView(void) const {
        return getCameraRight().getCameraOuterParam().get()*getCameraLeft().getCameraOuterParam().get().inv();
    }
    
    inline const TpCameraOuterParam getTranslateCvtPtRViewToLView(void) const {
        //return mCameraOuterParamCvtPtLViewToRView.getWise();
        return getTranslateCvtPtLViewToRView().get().inv();
    }
    
    const string str(void) const {
        stringstream sStrStream;
        sStrStream 
        << "Stereo Camera Config:" << endl
        << " Left  Camera: " <<endl << getCameraLeft().str() << endl
        << " Right Camera: " <<endl << getCameraRight().str() << endl
        << endl;
        return sStrStream.str();
    }
private:
    Camera              mCameraRight;
    //TpCameraOuterParam  mCameraOuterParamCvtPtLViewToRView;
};
    
typedef std::shared_ptr<CameraStereo> TpPtrCameraStereo;

}
}

#endif

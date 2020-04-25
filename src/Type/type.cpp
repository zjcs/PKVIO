#include "type.h"
#include <stdio.h>
#include <opencv2/calib3d.hpp>

namespace PKVIO{
namespace Type{
    
const TpFrameID     INVALIDFRAMEID      = -1;     
const TpKeyPointID  INVALIDKEYPOINTID   = -1;     
const TpMapPointID  INVALIDMAPPOINTID   = -1;


namespace TypeConvertor{
    
string cvtTimeStampToString(const TpTimeStamp& t){
    char a[100];
    sprintf(a,"%19.0f",t);
    return string(a);
}
    
TpMatchPair cvtMatchToMatchPair(const cv::DMatch& m){
    return std::make_pair(m.queryIdx, m.trainIdx);
}
}

cv::Mat cvtMat32fToMat64f(const cv::Mat&T){
    assert(T.type() == CV_32F);
    cv::Mat T64(T.size(), CV_64FC(T.channels()));
    for(auto nRow=0,nSzRows=T.rows;nRow<nSzRows;++nRow){
        for(auto nCol=0,nSzCols=T.rows;nCol<nSzCols;++nCol){
            switch(T.channels()){
                case 1: 
                    T64.at<double>(nRow,nCol) = T.at<float>(nRow,nCol);
                    break;
                case 2: 
                    T64.at<cv::Vec2d>(nRow,nCol) = T.at<cv::Vec2f>(nRow,nCol);
                    break;
                case 3: 
                    T64.at<cv::Vec3d>(nRow,nCol) = T.at<cv::Vec3f>(nRow,nCol);
                    break;
                case 4: 
                    T64.at<cv::Vec4d>(nRow,nCol) = T.at<cv::Vec4f>(nRow,nCol);
                    break;
                default: throw;
            }
        }
    }
    return T64;
}
void cvtMatx44fToRTvec(const cv::Mat&T, cv::Mat& R, cv::Mat&t){
    cv::Rodrigues(T, R, t);
}

void cvtMatx44fToR33T31(const cv::Mat&T, cv::Mat& R, cv::Mat&t){
    R = (T)(cv::Rect(0,0,3,3)).clone();
    t = (T)(cv::Rect(3,0,1,3)).clone();
}

void cvtMatx44fToR33T31(const cv::Matx44f&T, cv::Matx33f& R, cv::Matx31f&t)
{
    R = cv::Mat(T)(cv::Rect(0,0,3,3));
    t = cv::Mat(T)(cv::Rect(3,0,1,3));
}

void cvtMatx44fToPosition(const cv::Matx44f&T, cv::Vec3f& p)
{
    p = cvtMatx44fToPosition(T);
}

cv::Vec3f cvtMatx44fToPosition(const cv::Matx44f&T)
{
    cv::Mat R = cv::Mat(T)(cv::Rect(0,0,3,3));
    cv::Mat t = cv::Mat(T)(cv::Rect(3,0,1,3));
    cv::Matx31f mp = (cv::Mat(-R.t()*t));
    return cv::Vec3f(mp(0), mp(1), mp(2));
}

cv::Vec3f cvtMatx44fToZAxis(const cv::Matx44f&T){
    return cv::Vec3f(T(0,2), T(1,2), T(2,2));
}

void cvtProjectionMatrixToFxyCxy(const cv::Matx33f&P, cv::Vec2f& fxy, cv::Vec2f& cxy)
{
    fxy = cv::Vec2f(P(0,0), P(1,1));
    cxy = cv::Vec2f(P(0,2), P(1,2));
}

void cvtProjectionMatrixToFxyCxy(const cv::Mat&P, cv::Vec2f& fxy, cv::Vec2f& cxy)
{
    switch(P.type()){
        case CV_32F: {
            fxy = cv::Vec2f(P.at<float>(0,0), P.at<float>(1,1));
            cxy = cv::Vec2f(P.at<float>(0,2), P.at<float>(1,2));
        }break;
        case CV_64F: {
            fxy = cv::Vec2f(P.at<double>(0,0), P.at<double>(1,1));
            cxy = cv::Vec2f(P.at<double>(0,2), P.at<double>(1,2));
        }break;
        default: throw;
    }
}

cv::Matx44f cvtR33T31ToMatx44f(const cv::Mat& R, const cv::Mat&t)
{
    assert(R.type() == t.type());
    cv::Matx44f T = cv::Matx44f::eye();
    switch(R.type()){
        case CV_32FC1: {
            for(int nRow=0;nRow<3;++nRow){
                T(nRow, 3) = t.at<float>(nRow, 0);
                for(int nCol=0;nCol<3;++nCol){
                    T(nRow, nCol) = R.at<float>(nRow, nCol);
                }
            }
        }break;
        case CV_64FC1: {
            for(int nRow=0;nRow<3;++nRow){
                T(nRow, 3) = t.at<double>(nRow, 0);
                for(int nCol=0;nCol<3;++nCol){
                    T(nRow, nCol) = R.at<double>(nRow, nCol);
                }
            }
        }break;
        default: throw;
    }
    return T;
}
    
cv::Vec3f project(const cv::Matx44f& T, const cv::Vec3f& pt){
    cv::Vec4f npt(pt(0),pt(1),pt(2),1);
    cv::Vec4f proj = T*npt;
    //cout << T << pt << proj <<endl;
    return cv::Vec3f(proj(0),proj(1),proj(2));
}

cv::Vec2f project(const cv::Matx33f& P, const cv::Matx44f& T, const cv::Vec3f& pt){
    cv::Vec3f ptInCam = project(T, pt);
    cv::Vec3f ptInImg = P*ptInCam/ptInCam(2);
    //cout << P << ptInCam << ptInImg <<endl;
    return cv::Vec2f(ptInImg(0),ptInImg(1));
}

cv::Vec2f projectError(const cv::Matx33f& P, const cv::Matx44f& T, const cv::Vec3f& pt, const cv::Vec2f& pt2d){
    return project(P,T,pt) - pt2d;
}

std::vector<cv::Point2f> cvt(const std::vector<cv::KeyPoint>& vKpts){
    std::vector<cv::Point2f> vpt(vKpts.size());
    for(int nIdx=0,nSz=vKpts.size();nIdx<nSz;++nIdx){
        vpt[nIdx] = vKpts[nIdx].pt;
    }
    return vpt;
}

std::vector<cv::KeyPoint> cvt(const std::vector<cv::Point2f>& vpt){
    std::vector<cv::KeyPoint> vKpt(vpt.size());
    for(int nIdx=0,nSz=vpt.size();nIdx<nSz;++nIdx){
        vKpt[nIdx] = cv::KeyPoint(vpt[nIdx], 1);
    }
    return vKpt;
}

}
}

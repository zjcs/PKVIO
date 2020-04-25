#ifndef __CONVERTOR_H__
#define __CONVERTOR_H__

#include <opencv2/core/types.hpp>

namespace PKVIO
{
namespace Type
{
cv::Mat                                 cvtMat32fToMat64f(const cv::Mat&T);
void                                    cvtMatx44fToRTvec(const cv::Mat&T, cv::Mat& R, cv::Mat&t);
void                                    cvtMatx44fToR33T31(const cv::Mat&T, cv::Mat& R, cv::Mat&t);
cv::Matx44f                             cvtR33T31ToMatx44f(const cv::Mat& R, const cv::Mat&t);
void                                    cvtMatx44fToR33T31(const cv::Matx44f&T, cv::Matx33f& R, cv::Matx31f&t);
void                                    cvtMatx44fToPosition(const cv::Matx44f&T, cv::Vec3f& p);
cv::Vec3f                               cvtMatx44fToPosition(const cv::Matx44f&T);
cv::Vec3f                               cvtMatx44fToZAxis(const cv::Matx44f&T);

void        cvtProjectionMatrixToFxyCxy(const cv::Matx33f&P, cv::Vec2f& fxy, cv::Vec2f& cxy);
void        cvtProjectionMatrixToFxyCxy(const cv::Mat&P, cv::Vec2f& fxy, cv::Vec2f& cxy);
std::vector<cv::Point2f> cvt(const std::vector<cv::KeyPoint>& vKpts);
std::vector<cv::KeyPoint> cvt(const std::vector<cv::Point2f>& vKpts);
}
}

#endif

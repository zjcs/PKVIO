#ifndef __CONVERTOR_H__
#define __CONVERTOR_H__

#include <opencv2/core/types.hpp>

namespace PKVIO
{
namespace Type
{
void                                    cvtMatx44fToR33T31(const cv::Matx44f&T, cv::Matx33f& R, cv::Matx31f&t);
void                                    cvtMatx44fToPosition(const cv::Matx44f&T, cv::Vec3f& p);
cv::Vec3f                               cvtMatx44fToPosition(const cv::Matx44f&T);
}
}

#endif

#ifndef __DRAW_H__
#define __DRAW_H__

#include "../Type/type.h"

namespace PKVIO{
    namespace Draw{
        cv::Mat drawKeyPoints(const cv::Mat& mImg, const TpVecKeyPoints& mKpts);
        cv::Mat drawMatch(const cv::Mat& mImgLeft, const Type::TpVecKeyPoints& mKptsLeft, const cv::Mat& mImgRight, const Type::TpVecKeyPoints& mKptsRight, bool bTrueHoriFalseVetical = true, const std::string sWindowTitle_ShowIfNotEmpty = "");
    }
}

#endif

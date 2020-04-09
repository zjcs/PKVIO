#ifndef __DRAW_H__
#define __DRAW_H__

#include <opencv2/features2d.hpp>
#include "../Type/type.h"

namespace PKVIO{
    namespace Draw{
        cv::Mat drawKeyPoints(const cv::Mat& mImg, const TpVecKeyPoints& mKpts);
        
        cv::Mat rotate90(const cv::Mat& mImg);
        TpKeyPoint rotate90(const TpKeyPoint& nKpt, cv::Size nSz);
        TpVecKeyPoints rotate90(const TpVecKeyPoints& nVecKpt, cv::Size nSz);
        
        cv::Mat     showImageIfTitleNotEmpty(const cv::Mat& mImg, const std::string sWindowTitle_ShowIfNotEmpty = "");
        
        cv::Mat merge(const cv::Mat& mImgLeft,const cv::Mat& mImgRight, bool bTrueHoriFalseVetical = true);
        cv::Mat drawMatch(const cv::Mat& mImgLeft, const Type::TpVecKeyPoints& mKptsLeft, const cv::Mat& mImgRight, const Type::TpVecKeyPoints& mKptsRight, bool bTrueHoriFalseVetical = true, const std::string sWindowTitle_ShowIfNotEmpty = "");
        cv::Mat drawMatch(const cv::Mat& mImgLeft, const Type::TpVecKeyPoints& mKptsLeft, const cv::Mat& mImgRight, const Type::TpVecKeyPoints& mKptsRight, const Type::TpVecMatchResult& mMatch, bool bTrueHoriFalseVetical = true, const std::string sWindowTitle_ShowIfNotEmpty = "");
    }
}

#endif

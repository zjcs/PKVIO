#include "Draw.h"
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;

namespace PKVIO{
namespace Draw{

cv::Mat drawKeyPoints(const cv::Mat& mImg, const TpVecKeyPoints& mKpts)
{
    bool bSomethingWrong = mImg.empty();
    cv::Mat mImgWithKpts;
    //cv::drawKeypoints (mImg, mKpts, mImgWithKpts, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::drawKeypoints (mImg, mKpts, mImgWithKpts);
    return mImgWithKpts;
    //TODO
    
}
    
cv::Mat drawMatch(const cv::Mat& mImgLeft, const Type::TpVecKeyPoints& mKptsLeft,
                  const cv::Mat& mImgRight, const Type::TpVecKeyPoints& mKptsRight,
                  bool bTrueHoriFalseVetical /*= true*/, const std::string sWindowTitle_ShowIfNotEmpty /*= ""*/)
{
    cv::Mat mImgKptLeft = drawKeyPoints(mImgKptLeft, mKptsLeft);
    cv::Mat mImgKptRight = drawKeyPoints(mImgKptRight, mKptsRight);
    
    cv::Mat mDrawMatchResult;
    if(!sWindowTitle_ShowIfNotEmpty.empty()){
        cv::imshow(sWindowTitle_ShowIfNotEmpty, mDrawMatchResult);
        cv::waitKey(1);
    }
    return mDrawMatchResult;
}

cv::Mat drawMatch(const cv::Mat& mImgLeft, const Type::TpVecKeyPoints& mKptsLeft, const cv::Mat& mImgRight, const Type::TpVecKeyPoints& mKptsRight, const Type::TpVecMatchResult& mMatch, bool bTrueHoriFalseVetical /*= true*/, const std::string sWindowTitle_ShowIfNotEmpty /*= ""*/){
    
    cv::Mat mDrawMatchResult;
    cv::drawMatches(mImgLeft, mKptsLeft, mImgRight, mKptsRight, mMatch, mDrawMatchResult);
    
    if(!sWindowTitle_ShowIfNotEmpty.empty()){
        cv::imshow(sWindowTitle_ShowIfNotEmpty, mDrawMatchResult);
        cv::waitKey(1);
    }
    
    return mDrawMatchResult;
    
}

}
}


#include "Draw.h"
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;

namespace PKVIO{
namespace Draw{
    
cv::Mat showImageIfTitleNotEmpty(const cv::Mat& mImg, const std::string sWindowTitle_ShowIfNotEmpty /*= ""*/)
{
        if(!sWindowTitle_ShowIfNotEmpty.empty()){
            cv::imshow(sWindowTitle_ShowIfNotEmpty, mImg);
            cv::waitKey(1);
        }
        return mImg;
}

void drawKeyPointsSelf(cv::Mat& mImg, const TpVecKeyPoints& mKpts){
    cv::drawKeypoints (mImg, mKpts, mImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
}

cv::Mat drawKeyPoints(const cv::Mat& mImg, const TpVecKeyPoints& mKpts)
{
    bool bSomethingWrong = mImg.empty();
    cv::Mat mImgWithKpts;
    //cv::drawKeypoints (mImg, mKpts, mImgWithKpts, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::drawKeypoints (mImg, mKpts, mImgWithKpts);
    return mImgWithKpts;
    //TODO
    
}

cv::Mat rotate90(const cv::Mat& mImg)
{
    cv::Mat mResult;
    cv::flip(mImg.t(), mResult, 1); // transpose, then flip as x axis;
    return mResult;
}

cv::Mat merge(const cv::Mat& mImgLeft,const cv::Mat& mImgRight, bool bTrueHoriFalseVetical /*= true*/)
{
    if(mImgLeft.empty())
        return mImgRight.clone();
    if(mImgRight.empty())
        return mImgLeft.clone();
    auto FuncMerge = [=](const cv::Mat& mImgL, const cv::Mat& mImgR)->cv::Mat{
        size_t nW = mImgL.cols + mImgR.cols;
        size_t nH = mImgL.rows + mImgR.rows;
        size_t nHMax = std::max(mImgL.rows, mImgR.rows);
        size_t nWMax = std::max(mImgL.cols, mImgR.cols);
        if(bTrueHoriFalseVetical){
            cv::Mat mImgResult(nHMax, nW, mImgL.type(), cv::Scalar(0));
            mImgL.copyTo(mImgResult(cv::Rect(0,0,mImgL.cols, mImgL.rows)));
            mImgR.copyTo(mImgResult(cv::Rect(mImgL.cols, 0, mImgR.cols, mImgR.rows)));
            return mImgResult;
        }else{
            cv::Mat mImgResult(nH, nWMax, mImgL.type(), cv::Scalar(0));
            mImgL.copyTo(mImgResult(cv::Rect(0,0,mImgL.cols, mImgL.rows)));
            mImgR.copyTo(mImgResult(cv::Rect(0, mImgL.rows, mImgR.cols, mImgR.rows)));
            return mImgResult;
        }
    };
    
    if(mImgLeft.type()!=mImgRight.type()) {
        cv::Mat mImgRcp = cv::Mat(mImgRight.size(), mImgLeft.type(), cv::Scalar(0));
        cout <<"Error: Image type not match in Tools::merge - " << mImgLeft.channels() << mImgRight.channels()<<  mImgRcp.channels() <<endl;
        return FuncMerge(mImgLeft, mImgRcp);
    }else{
        return FuncMerge(mImgLeft, mImgRight);
    }
    
}

TpKeyPoint rotate90(const TpKeyPoint& nKpt, cv::Size nSz){
    TpKeyPoint nKpt90 = nKpt;
    nKpt90.pt.x = nSz.height - nKpt.pt.y;
    nKpt90.pt.y = nKpt.pt.x;
    return nKpt90;
}

TpVecKeyPoints rotate90(const TpVecKeyPoints& nVecKpt, cv::Size nSz){
    TpVecKeyPoints nVecKpt90;
    nVecKpt90.reserve(nVecKpt.size());
    for(size_t nIdxKpt=0,nSzKpts=nVecKpt.size();nIdxKpt<nSzKpts;++nIdxKpt){
        nVecKpt90.push_back(rotate90(nVecKpt[nIdxKpt], nSz));
    }
    return nVecKpt90;
}
    
cv::Mat drawMatch(const cv::Mat& mImgLeft, const Type::TpVecKeyPoints& mKptsLeft,
                  const cv::Mat& mImgRight, const Type::TpVecKeyPoints& mKptsRight,
                  bool bTrueHoriFalseVetical /*= true*/, const std::string sWindowTitle_ShowIfNotEmpty /*= ""*/)
{
    if(mKptsLeft.size()!=mKptsRight.size()){
        cv::Mat mImgKptLeft = drawKeyPoints(mImgKptLeft, mKptsLeft);
        cv::Mat mImgKptRight = drawKeyPoints(mImgKptRight, mKptsRight);
        
        cv::Mat mDrawMatchResult = merge(mImgKptLeft, mImgKptRight, bTrueHoriFalseVetical);
        return showImageIfTitleNotEmpty(mDrawMatchResult, sWindowTitle_ShowIfNotEmpty);
    }else{
        TpVecMatchResult mMatch; mMatch.reserve(mKptsLeft.size());
        for(int nIdxKpt=0,nSzKpts=mKptsLeft.size();nIdxKpt<nSzKpts;++nIdxKpt)
            mMatch.push_back(cv::DMatch(nIdxKpt, nIdxKpt, -1));
        
        return drawMatch(mImgLeft, mKptsLeft, mImgRight, mKptsRight, mMatch, bTrueHoriFalseVetical, sWindowTitle_ShowIfNotEmpty);
    }
    
}

cv::Mat drawMatch(const cv::Mat& mImgLeft, const Type::TpVecKeyPoints& mKptsLeft, const cv::Mat& mImgRight, const Type::TpVecKeyPoints& mKptsRight, const Type::TpVecMatchResult& mMatch, bool bTrueHoriFalseVetical /*= true*/, const std::string sWindowTitle_ShowIfNotEmpty /*= ""*/)
{
    
    cv::Mat mImgKptLeft = drawKeyPoints(mImgLeft, mKptsLeft);
    cv::Mat mImgKptRight = drawKeyPoints(mImgRight, mKptsRight);
    
    cv::Mat mDrawMatchResult;
    if(bTrueHoriFalseVetical){
        cv::drawMatches(mImgKptLeft, mKptsLeft, mImgKptRight, mKptsRight, mMatch, mDrawMatchResult);
    }else{
        //TpVecKeyPoints nKptsLeft90  = rotate90(mKptsLeft, mImgKptLeft.size());
        //TpVecKeyPoints nKptsRight90 = rotate90(mKptsRight, mImgKptRight.size());
        //mImgKptLeft = rotate90(mImgKptLeft); mImgKptRight = rotate90(mImgKptRight);
        
        //cv::drawMatches(mImgKptLeft, nKptsLeft90, mImgKptRight, nKptsRight90, mMatch, mDrawMatchResult);
        mDrawMatchResult = merge(mImgKptLeft, mImgKptRight, bTrueHoriFalseVetical);
        for(int nIdxMatch=0, nSzMatch = mMatch.size();nIdxMatch<nSzMatch;++nIdxMatch){
            cv::line(mDrawMatchResult, 
                     mKptsLeft[mMatch[nIdxMatch].queryIdx].pt,
                     mKptsRight[mMatch[nIdxMatch].trainIdx].pt + cv::Point2f(0, mImgKptLeft.rows),
                     cv::Scalar(random()%256,random()%256,random()%256), 2 
            );
        }
    }
    
    return showImageIfTitleNotEmpty(mDrawMatchResult, sWindowTitle_ShowIfNotEmpty);
}

}
}


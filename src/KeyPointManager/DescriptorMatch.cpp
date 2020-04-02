#include "DescriptorMatch.h"
#include <iostream>
#include <opencv2/features2d.hpp>
#include "../Tools/Tools.h"


namespace PKVIO
{
namespace KeyPointManager{

const TpVecMatchPairs TpDescriptorMatchResult::getMatchPairs() const {
    TpVecMatchPairs vPairs;
    vPairs.reserve(mMatch.size());
    for_each(mMatch.begin(),mMatch.end(), [&](const cv::DMatch& m){
        auto p = cvtMatchToMatchPair(m);
        vPairs.push_back(p);
    });
    return vPairs;
}

TpDescriptorMatchResult DescriptorMatch::match(const Type::Frame& fFrame, const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc) {
    switch(mEnMatchMethod){
        case EnKnnWholeImage: return matchByKnn(fFrame, fKptsDesc);
        case EnBrutForceInWindow: return matchByBrutForceInWindow(fFrame, fKptsDesc);
        default: throw;
    }
}

TpDescriptorMatchResult DescriptorMatch::matchByKnn(const Type::Frame& fFrame, const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc) {
    vector<TpVecMatchResult> vVecMatchResult;
    const auto& vDescLeft = fKptsDesc.mDescriptorsLeft, vDescRight = fKptsDesc.mDescriptorsRight;
    cv::Ptr<cv::DescriptorMatcher> pKnnMatch = cv::DescriptorMatcher::create("BruteForce");
    pKnnMatch->knnMatch(vDescLeft, vDescRight, vVecMatchResult, 2);
    
    
    //cout << "Feat: " << vDescLeft.rows << "|" << vDescRight.rows << " Match: " << vVecMatchResult.size() << " - " << vVecMatchResult[0].size() <<endl;
    
    TpVecMatchResult mBestVecMatchResult; 
    int nInitMatch = vVecMatchResult.size();
    mBestVecMatchResult.reserve(nInitMatch);
    for(int nIdx=0,nSz=nInitMatch;nIdx<nSz;++nIdx){
        auto& m1st = vVecMatchResult[nIdx][0];
        auto& m2nd = vVecMatchResult[nIdx][1];
         //cout << "1st|2st: " << m1st.distance << "|" << m2nd.distance << endl;
        if(m1st.distance < m2nd.distance*0.8){
            // auto mKpt1stLeft = fKptsDesc.mKeyPointsLeft[m1st.queryIdx];
            // auto mKpt1stRight = fKptsDesc.mKeyPointsRight[m1st.trainIdx];
            // auto mDistance = mKpt1stLeft.pt - mKpt1stRight.pt;
            // // dx is about 5pixel, dy is about 15pixel;
            // cout << "mDistance: " << mDistance<< endl;  
            mBestVecMatchResult.push_back(m1st);
        }
    }
    
    auto r = TpDescriptorMatchResult(mBestVecMatchResult);
    // Best|Init: 50|500, only about 10%, should limited the search area, and some is wrong match(20%). even with a more stric filter: 1st < 0.8*2nd;
    showMatchResult(fFrame, fKptsDesc,  r);
    return r;
    
}


TpDescriptorMatchResult DescriptorMatch::matchByBrutForceInWindow(const Type::Frame& fFrame, 
                                                                  const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc) {
    //
    cv::Size mSearchWindowSize(30, 25); 
    cv::Point mSearchWindowOrig(mSearchWindowSize/2);
    
    //cout << fKptsDesc.mDescriptorsLeft.rows << "-" << fKptsDesc.mDescriptorsLeft.cols << "-" << fKptsDesc.mDescriptorsLeft.channels() << "-" << fKptsDesc.mDescriptorsLeft.type() << endl;
    //cout << cv::CV_8UC32 <<endl;
    //cv::CV_8UC3;
    
    TpVecMatchResult mBestVecMatchResult;
    for(int nKptIdxLeft = 0, nSzKptLeft = (int)fKptsDesc.mKeyPointsLeft.size();nKptIdxLeft<nSzKptLeft;++nKptIdxLeft){
        auto& KptLeft = fKptsDesc.mKeyPointsLeft[nKptIdxLeft].pt;
        auto _KptLeft = cv::Point(KptLeft.x+mSearchWindowOrig.x, KptLeft.y+mSearchWindowOrig.y);
        
        int nMinKptIdxRight = -1; float mMinDistance = 1000000, mMin2ndDistance = mMinDistance+1; 
        for(int nKptIdxRight = 0, nSzKptRight = (int)fKptsDesc.mKeyPointsRight.size();nKptIdxRight<nSzKptRight;++nKptIdxRight){
            auto& KptRight = fKptsDesc.mKeyPointsRight[nKptIdxRight].pt;
            auto dDistance = cv::Point(_KptLeft.x-KptRight.x,_KptLeft.y-KptRight.y);
            if(Tools::isInWindow(dDistance, mSearchWindowSize)){
                auto mKptLeftDesc = (cv::Mat)fKptsDesc.mDescriptorsLeft.row(nKptIdxLeft);
                auto mKptRightDesc = (cv::Mat)fKptsDesc.mDescriptorsRight.row(nKptIdxRight);
                //cout << mKptLeftDesc.rows << "-" << mKptLeftDesc.cols << "-" << mKptLeftDesc.channels() << "-" << mKptLeftDesc.type() << endl;
                //cout << mKptRightDesc.rows << "-" << mKptRightDesc.cols << "-" << mKptRightDesc.channels() << "-" << mKptRightDesc.type() << endl;
                auto fDistance = 0;
                for(int nIdxCol=0,nSzCols=mKptLeftDesc.cols;nIdxCol<nSzCols;++nIdxCol){
                    auto vLeft = mKptLeftDesc.at<uchar>(0,nIdxCol);
                    auto vRight = mKptRightDesc.at<uchar>(0,nIdxCol);
                    auto e = vLeft -vRight;
                    fDistance += e*e;
                }
                fDistance = std::sqrt(fDistance);
                //cout << fDistance<<endl;
                if(fDistance<mMinDistance){
                    nMinKptIdxRight = nKptIdxRight;
                    mMin2ndDistance = mMinDistance;
                    mMinDistance = fDistance;
                }
                
            }
            
        }
        if(mMinDistance<mMin2ndDistance*0.95){
            mBestVecMatchResult.push_back(cv::DMatch(nKptIdxLeft,nMinKptIdxRight, mMinDistance));
        }
    }
    auto r = TpDescriptorMatchResult(mBestVecMatchResult);
    // Best|Init: 230|500, almost about 50%, and all is right match. even with a less stric filter: 1st < 0.95*2nd;
    showMatchResult(fFrame, fKptsDesc,  r);
    
    return r;
}

void DescriptorMatch::showMatchResult(const Frame& fFrame, const TpOneFrameKptDescriptor& fKptsDesc, const TpDescriptorMatchResult& mBestVecMatchResult){
    if(!mShowMatchResult)return;
    
    // Log accuracy.
    // int nInitMatch = (int)fKptsDesc.mKeyPointsLeft.size();
    // int nBestMatch = (int)mBestVecMatchResult.get().size();
    // cout << "Match: Best|Init - " << nBestMatch <<"|" <<nInitMatch<<endl;
    
    const StereoFrame& fStereoFrame = dynamic_cast<const StereoFrame&>(fFrame);
    cv::Mat mDrawMatchResult = Tools::drawMatch(fStereoFrame.ImageLeft(), fKptsDesc.mKeyPointsLeft, fStereoFrame.ImageRight(), fKptsDesc.mKeyPointsRight, mBestVecMatchResult.get(), false , "mBestMatch");
}

}
}
    

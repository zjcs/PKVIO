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

TpDescriptorMatchResult DescriptorMatch::match(const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc) {
    TpDescriptorMatchResult nMatResult;
    switch(mEnMatchMethod){
        case EnKnnWholeImage: nMatResult = matchByKnn(fKptsDesc); break;
        case EnBrutForceInWindow: nMatResult = matchByBrutForceInWindow(fKptsDesc); break;
        default: throw;
    }
    return removeDuplicatedMatch(fKptsDesc, nMatResult);
}

TpDescriptorMatchResult DescriptorMatch::matchByKnn(const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc) {
    
    const int nCountKptsLeft = (int)fKptsDesc.mKeyPointsLeft.size(), nCountKptsRight = (int)fKptsDesc.mKeyPointsRight.size();
    
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
    
    // Best|Init: 50|500, only about 10%, should limited the search area, and some is wrong match(20%). even with a more stric filter: 1st < 0.8*2nd;
    return TpDescriptorMatchResult(fKptsDesc.FrameIDLeft(), fKptsDesc.FrameIDRight(), mBestVecMatchResult, nCountKptsLeft, nCountKptsRight);
}


TpDescriptorMatchResult DescriptorMatch::matchByBrutForceInWindow(const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc) {
    //
    cv::Size mSearchWindowSize(30, 25); 
    cv::Point mSearchWindowOrig(mSearchWindowSize/2);
    
    //cout << fKptsDesc.mDescriptorsLeft.rows << "-" << fKptsDesc.mDescriptorsLeft.cols << "-" << fKptsDesc.mDescriptorsLeft.channels() << "-" << fKptsDesc.mDescriptorsLeft.type() << endl;
    //cout << cv::CV_8UC32 <<endl;
    //cv::CV_8UC3;
    
    const int nCountKptsLeft = (int)fKptsDesc.mKeyPointsLeft.size(), nCountKptsRight = (int)fKptsDesc.mKeyPointsRight.size();
    
    TpVecMatchResult mBestVecMatchResult;
    for(int nKptIdxRight = 0;nKptIdxRight<nCountKptsRight;++nKptIdxRight){
        auto& KptRight = fKptsDesc.mKeyPointsRight[nKptIdxRight].pt;
        auto _KptRight = cv::Point(KptRight.x+mSearchWindowOrig.x, KptRight.y+mSearchWindowOrig.y);
        auto mKptRightDesc = (cv::Mat)fKptsDesc.mDescriptorsRight.row(nKptIdxRight);
        
        int nMinKptIdxLeft = -1; float mMinDistance = 1000000, mMin2ndDistance = mMinDistance+1; 
        for(int nKptIdxLeft = 0;nKptIdxLeft<nCountKptsLeft;++nKptIdxLeft){
            auto& KptLeft = fKptsDesc.mKeyPointsLeft[nKptIdxLeft].pt;
            auto dDistance = cv::Point(_KptRight.x-KptLeft.x,_KptRight.y-KptLeft.y);
            
            if(Tools::isInWindow(dDistance, mSearchWindowSize)){
                auto mKptLeftDesc = (cv::Mat)fKptsDesc.mDescriptorsLeft.row(nKptIdxLeft);
                //cout << mKptLeftDesc.rows << "-" << mKptLeftDesc.cols << "-" << mKptLeftDesc.channels() << "-" << mKptLeftDesc.type() << endl;
                //cout << mKptRightDesc.rows << "-" << mKptRightDesc.cols << "-" << mKptRightDesc.channels() << "-" << mKptRightDesc.type() << endl;
                auto fDistance = distance(mKptLeftDesc,mKptRightDesc);
                if(fDistance<mMinDistance){
                    nMinKptIdxLeft = nKptIdxLeft;
                    mMin2ndDistance = mMinDistance;
                    mMinDistance = fDistance;
                }
                
            }
            
        }
        if(mMinDistance<mMin2ndDistance*0.95){
            mBestVecMatchResult.push_back(cv::DMatch(nMinKptIdxLeft,nKptIdxRight,mMinDistance));
        }
    }
    
    // Best|Init: 230|500, almost about 50%, and all is right match. even with a less stric filter: 1st < 0.95*2nd;
    return TpDescriptorMatchResult(fKptsDesc.FrameIDLeft(), fKptsDesc.FrameIDRight(), mBestVecMatchResult, nCountKptsLeft, nCountKptsRight);
}

cv::Mat DescriptorMatch::showMatchResult(const Type::Frame& fFrame, const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc, const TpDescriptorMatchResult& mBestVecMatchResult, const string sWindowTitle) {
    // Log accuracy.
    // int nInitMatch = (int)fKptsDesc.mKeyPointsLeft.size();
    // int nBestMatch = (int)mBestVecMatchResult.get().size();
    // cout << "Match: Best|Init - " << nBestMatch <<"|" <<nInitMatch<<endl;
    
    const StereoFrame& fStereoFrame = dynamic_cast<const StereoFrame&>(fFrame);
    cv::Mat mDrawMatchResult = Tools::drawMatch(fStereoFrame.getImageLeft(), fKptsDesc.mKeyPointsLeft, fStereoFrame.getImageRight(), fKptsDesc.mKeyPointsRight, mBestVecMatchResult.get(), false , sWindowTitle);
    
    //assert(mPtrDesciptorMatcher->debugDuplicatedMatch(f, mKptsDescriptors, mMatchResult));
    debugDuplicatedMatch(fFrame, fKptsDesc, mBestVecMatchResult);
    
    return mDrawMatchResult;
}


bool DescriptorMatch::debugDuplicatedMatch(const Type::Frame& fFrame, const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc, const TpDescriptorMatchResult& mBestVecMatchResult) 
{
    std::set<TpKeyPointIndex> nUniqueKptIdxLeft, nDuplicatedKptIdxLeft;
    for(int nIdxMatch = 0, nSzMatch = mBestVecMatchResult.getCountMatchKpts();nIdxMatch<nSzMatch;++nIdxMatch ){
        const auto& nMatch = mBestVecMatchResult.get()[nIdxMatch];
        TpKeyPointIndex nKptIdxLeft = nMatch.queryIdx;
        if(nUniqueKptIdxLeft.count(nKptIdxLeft)){
            //cout << "**** Error: Duplicated Match KptIdx Left: " <<nKptIdxLeft << "|" << nMatch.trainIdx<<"|"<<nMatch.distance<< m endl;
            nDuplicatedKptIdxLeft.insert(nKptIdxLeft);
        }
        nUniqueKptIdxLeft.insert(nKptIdxLeft);
    }
    
    if(!nDuplicatedKptIdxLeft.size())
        return true;
    
    TpVecMatchResult nVecMatchResult; nVecMatchResult.reserve(nDuplicatedKptIdxLeft.size());
    for(int nIdxMatch = 0, nSzMatch = mBestVecMatchResult.getCountMatchKpts();nIdxMatch<nSzMatch;++nIdxMatch ){
        const auto& nMatch = mBestVecMatchResult.get()[nIdxMatch];
        TpKeyPointIndex nKptIdxLeft = nMatch.queryIdx;
        if(nDuplicatedKptIdxLeft.count(nKptIdxLeft)) {
            nVecMatchResult.push_back(nMatch);
        }
    }
    
    cout << "Duplicated:" << fKptsDesc.FrameIDLeft() << " | " << fKptsDesc.FrameIDRight() << endl;
    
    const StereoFrame& fStereoFrame = dynamic_cast<const StereoFrame&>(fFrame);
    cv::Mat mDrawMatchResult = Tools::drawMatch(fStereoFrame.getImageLeft(), fKptsDesc.mKeyPointsLeft, fStereoFrame.getImageRight(), fKptsDesc.mKeyPointsRight, mBestVecMatchResult.get(), false , "Duplicated");
     cv::waitKey();
    return false;
}


TpDescriptorMatchResult DescriptorMatch::removeDuplicatedMatch(const PKVIO::KeyPointManager::TpOneFrameKptDescriptor& fKptsDesc, TpDescriptorMatchResult& mBestVecMatchResult) 
{
    std::map<int, pair<int, float>> nMapDuplicatedKptIdx2MatchIdxAndDescDistance;
    std::set<TpKeyPointIndex> nUniqueKptIdxLeft, nDuplicatedKptIdxLeft; int nSzDuplicated = 0;
    for(int nIdxMatch = 0, nSzMatch = mBestVecMatchResult.getCountMatchKpts();nIdxMatch<nSzMatch;++nIdxMatch ){
        const auto& nMatch = mBestVecMatchResult.get()[nIdxMatch];
        TpKeyPointIndex nKptIdxLeft = nMatch.queryIdx;
        if(nUniqueKptIdxLeft.count(nKptIdxLeft)){
            //cout << "**** Error: Duplicated Match KptIdx Left: " <<nKptIdxLeft << "|" << nMatch.trainIdx<<"|"<<nMatch.distance<< m endl;
            nDuplicatedKptIdxLeft.insert(nKptIdxLeft);
            ++nSzDuplicated;
        }
        nUniqueKptIdxLeft.insert(nKptIdxLeft);
    }
    nSzDuplicated+=nDuplicatedKptIdxLeft.size();
    
    if(!nDuplicatedKptIdxLeft.size())
        return mBestVecMatchResult;
    
    TpVecMatchResult nVecMatchResult; nVecMatchResult.resize(mBestVecMatchResult.getCountMatchKpts()-nSzDuplicated);
    const auto& nOldVecMatchResult = mBestVecMatchResult.get();
    std::copy_if(nOldVecMatchResult.cbegin(),nOldVecMatchResult.cend(),nVecMatchResult.begin(), [&](const TpOneMatchResult& r){
        return !nDuplicatedKptIdxLeft.count(r.queryIdx);
    });
    mBestVecMatchResult = TpDescriptorMatchResult(mBestVecMatchResult, nVecMatchResult);
    return mBestVecMatchResult;
    
    /*
    TpVecMatchResult nVecMatchResult; nVecMatchResult.reserve(nDuplicatedKptIdxLeft.size());
    for(int nIdxMatch = 0, nSzMatch = mBestVecMatchResult.getCountMatchKpts();nIdxMatch<nSzMatch;++nIdxMatch ){
        const auto& nMatch = mBestVecMatchResult.get()[nIdxMatch];
        TpKeyPointIndex nKptIdxLeft = nMatch.queryIdx;
        if(nDuplicatedKptIdxLeft.count(nKptIdxLeft)) {
            nVecMatchResult.push_back(nMatch);
        }
    }
    */
 
}


float DescriptorMatch::distance(const cv::Mat& nDescA, const cv::Mat& nDescB) {
    auto fDistance = 0;
    for(int nIdxCol=0,nSzCols=nDescA.cols;nIdxCol<nSzCols;++nIdxCol){
        auto vLeft = nDescA.at<uchar>(0,nIdxCol);
        auto vRight = nDescB.at<uchar>(0,nIdxCol);
        auto e = vLeft -vRight;
        fDistance += e*e;
    }
    fDistance = std::sqrt(fDistance);
    //cout << fDistance<<endl;    
    return fDistance;
}

}
}
    

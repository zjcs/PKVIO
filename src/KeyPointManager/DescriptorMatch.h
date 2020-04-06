#ifndef __DESCRIPTORMATCH_H__
#define __DESCRIPTORMATCH_H__

#include <vector>
#include "FrameKptsDescriptor.h"
#include "../Type/type.h"
#include <memory>
#include <iostream>

#include "../DebugManager/DebugManager.h"

using namespace std;

namespace PKVIO
{
namespace KeyPointManager{

class TpDescriptorMatchResult{
public:
    TpDescriptorMatchResult()
    : mMatch(0), mFrameIDLeft(INVALIDFRAMEID), mFrameIDRight(INVALIDFRAMEID)
    , mCountKptsLeft(-1), mCountKptsRight(-1)
    {}
    TpDescriptorMatchResult(const TpFrameID nFrameIDLeft, const TpFrameID nFrameIDRight, const TpVecMatchResult& r, const int nCountKptsLeft = -1,const int nCountKptsRight = -1)
    : mMatch(r) 
    , mFrameIDLeft(nFrameIDLeft) , mFrameIDRight(nFrameIDRight)
    , mCountKptsLeft(nCountKptsLeft), mCountKptsRight(nCountKptsRight)
    {}
    TpDescriptorMatchResult(const TpDescriptorMatchResult& cp)
    : mMatch(cp.mMatch), mFrameIDLeft(cp.mFrameIDLeft), mFrameIDRight(cp.mFrameIDRight)
    , mCountKptsLeft(cp.mCountKptsLeft),mCountKptsRight(cp.mCountKptsRight)
    {}
    TpDescriptorMatchResult(const TpDescriptorMatchResult& cp, const TpVecMatchResult& m)
    : mMatch(m), mFrameIDLeft(cp.mFrameIDLeft), mFrameIDRight(cp.mFrameIDRight)
    , mCountKptsLeft(cp.mCountKptsLeft),mCountKptsRight(cp.mCountKptsRight)
    {}
    
    inline const TpVecMatchResult&  get(void) const {return getMatch();} 
    const TpVecMatchPairs           getMatchPairs(void) const;
    inline const TpVecMatchResult&  getMatch(void)const{return mMatch;}
    
    inline void                     getMatchKptIndex(const int nIdxMatch, int& nKptIdxInLeft, int& nKptIdxInRight) const {
                                        const auto& mOneMatch   = mMatch[nIdxMatch];
                                        nKptIdxInLeft           = mOneMatch.queryIdx;
                                        nKptIdxInRight          = mOneMatch.trainIdx;
                                    }
    
    inline const int                getCountMatchKpts(void) const {return mMatch.size();}
    inline const int                getCountNonDuplicateKpts(void) const {return getCountKptsLeft()+getCountKptsRight()-getCountMatchKpts();}
    
    inline const TpFrameID          getFrameIDLeft(void)const{return mFrameIDLeft;}
    inline const TpFrameID          getFrameIDRight(void)const{return mFrameIDRight;}
    inline const int                getCountKptsLeft(void)const {return mCountKptsLeft;}
    inline const int                getCountKptsRight(void)const {return mCountKptsRight;}
private:
    TpVecMatchResult                mMatch;
    TpFrameID                       mFrameIDLeft,   mFrameIDRight;
    int                             mCountKptsLeft, mCountKptsRight;
};

class FrameMatchResult{
public:
    FrameMatchResult():mbInnerFrameDescriptorMatchResult(false){}
    const TpFrameID                         FrameID(void){
            if( size() == 0){
                cout << "Error: try to get the FrameID of Empty FrameMatchResult is illegal. exit." <<endl;
                throw;
            }
            return getFrameDescriptoreMatchResult(0).getFrameIDRight();
    }
    void                                    clear(void){mbInnerFrameDescriptorMatchResult=false;mVecDescriptorMatchResult.clear();}
    bool                                    isExistInnerFrameDescriptorMatchResult(void) const { return mbInnerFrameDescriptorMatchResult;}
    void                                    pushInnerFrameDescriptorMatchResult(TpDescriptorMatchResult& mInnerFrameDescMatchResult){
        mbInnerFrameDescriptorMatchResult = true;
        mVecDescriptorMatchResult.insert(mVecDescriptorMatchResult.begin(), mInnerFrameDescMatchResult);
    }
    const TpDescriptorMatchResult&          getInnerFrameDescriptorMatchResult(void) const {
        if(!mbInnerFrameDescriptorMatchResult)
            throw;
        return mVecDescriptorMatchResult[0];
    }
    
    inline int                              size(void) const {return (int)mVecDescriptorMatchResult.size();}
    inline const TpDescriptorMatchResult&   getFrameDescriptoreMatchResult(const int nIndex) const { return mVecDescriptorMatchResult[nIndex]; }
    inline int                              getOuterIndex(const TpFrameID nFrameIDMaster) const {
        int nStepDueToInner = (isExistInnerFrameDescriptorMatchResult()?1:0);
        for(int nIndex = nStepDueToInner, nSz = size();nIndex<nSz;++nIndex){
            if(getFrameDescriptoreMatchResult(nIndex).getFrameIDLeft() == nFrameIDMaster){
                return nIndex - nStepDueToInner;
            }
        }
        return -1;
    }
    inline const int                        sizeOuterFrameDescriptorMatchResult(void) const {return isExistInnerFrameDescriptorMatchResult()?size()-1:size();}
    inline void                             pushOuterFrameDescriptorMatchResult(TpDescriptorMatchResult& mOuterFrameDescMatchResult){mVecDescriptorMatchResult.push_back(mOuterFrameDescMatchResult);}
    inline const TpDescriptorMatchResult&   getOuterFrameDescriptorMatchResult(const int nOuterIndex) const {
        return mVecDescriptorMatchResult[mbInnerFrameDescriptorMatchResult?nOuterIndex+1:nOuterIndex];
    }
    
    inline const int                        getCountKptsOnThisFrame(void) const {
                                                if(isExistInnerFrameDescriptorMatchResult()){
                                                    return getInnerFrameDescriptorMatchResult().getCountNonDuplicateKpts();
                                                }else{
                                                    cout << "Mono. Not Support Now. exit" <<endl;
                                                    throw;
                                                }
                                                
                                            }
private:
    bool                                    mbInnerFrameDescriptorMatchResult;
    // 0st: left vs right, others: prev-frame/KF vs this-left;
    std::vector<TpDescriptorMatchResult>    mVecDescriptorMatchResult;
};

class FrameMatchResultHistory : public TpFrameDataHistory<FrameMatchResult>
{
public:
    FrameMatchResultHistory(): TpFrameDataHistory(FuncIsIDAndFrameDataMatch<FrameMatchResult>){ }
};

     
class DescriptorMatch{
    typedef enum {
        EnKnnWholeImage,
        EnBrutForceInWindow
    } EnMatchMethod;
public:
    DescriptorMatch(EnMatchMethod eMethod = EnBrutForceInWindow)
    : mEnMatchMethod(eMethod)
    , mStrDefaultWindowTitle("mBestMatch")
    {}
    
    TpDescriptorMatchResult     match(const TpOneFrameKptDescriptor& fKptsDesc);
    cv::Mat                     showMatchResult(const Frame& fFrame, const TpOneFrameKptDescriptor& fKptsDesc, const TpDescriptorMatchResult& mBestVecMatchResult, const string sWindowTitle = "mBestMatch");
    
    bool                        debugDuplicatedMatch(const Frame& fFrame, const TpOneFrameKptDescriptor& fKptsDesc, const TpDescriptorMatchResult& mBestVecMatchResult);
    
    inline const DebugManager::DebugMatchInfo& 
                                getDebugMatchInfo(void)const{return mDebugMatchInfo;}
protected:
    TpDescriptorMatchResult     matchByKnn(const TpOneFrameKptDescriptor& fKptsDesc);
    TpDescriptorMatchResult     matchByBrutForceInWindow(const TpOneFrameKptDescriptor& fKptsDesc);
    
    TpDescriptorMatchResult     removeDuplicatedMatchM1vSn(const TpOneFrameKptDescriptor& fKptsDesc, TpDescriptorMatchResult& mBestVecMatchResult);
    
    float                       distance(const cv::Mat& nDescA, const cv::Mat& nDescB);
private:
    const EnMatchMethod         mEnMatchMethod; 
    
    const string                mStrDefaultWindowTitle;
    
    DebugManager::DebugMatchInfo    mDebugMatchInfo;
};   

typedef std::shared_ptr<DescriptorMatch> PtrDescriptorMatch;
    
}
}

#endif

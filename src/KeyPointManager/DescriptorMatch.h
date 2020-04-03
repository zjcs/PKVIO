#ifndef __DESCRIPTORMATCH_H__
#define __DESCRIPTORMATCH_H__

#include <vector>
#include "FrameKptsDescriptor.h"
#include "../Type/type.h"
#include <memory>

using namespace std;

namespace PKVIO
{
namespace KeyPointManager{

class TpDescriptorMatchResult{
public:
    TpDescriptorMatchResult(const TpFrameID nFrameIDLeft, const TpFrameID nFrameIDRight, const TpVecMatchResult& r, const int nCountKptsLeft = -1,const int nCountKptsRight = -1)
    : mMatch(r) 
    , mFrameIDLeft(nFrameIDLeft) , mFrameIDRight(nFrameIDRight)
    , mCountKptsLeft(nCountKptsLeft), mCountKptsRight(nCountKptsRight)
    {}
    
    inline const TpVecMatchResult&  get(void) const {return getMatch();} 
    const TpVecMatchPairs           getMatchPairs(void) const;
    inline const TpVecMatchResult&  getMatch(void)const{return mMatch;}
    
    inline void getMatchKptIndex(const int nIdxMatch, int& nKptIdxInLeft, int& nKptIdxInRight) const {
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
protected:
    TpDescriptorMatchResult     matchByKnn(const TpOneFrameKptDescriptor& fKptsDesc);
    TpDescriptorMatchResult     matchByBrutForceInWindow(const TpOneFrameKptDescriptor& fKptsDesc);
    
private:
    const EnMatchMethod         mEnMatchMethod; 
    
    const string                mStrDefaultWindowTitle;
};   

typedef std::shared_ptr<DescriptorMatch> PtrDescriptorMatch;
    
}
}

#endif

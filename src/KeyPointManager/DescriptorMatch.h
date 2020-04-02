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
    TpDescriptorMatchResult(const TpVecMatchResult& r):mMatch(r){}
    inline const TpVecMatchResult& get(void) const {return mMatch;}
    const TpVecMatchPairs getMatchPairs(void) const;
private:
    TpVecMatchResult mMatch;
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

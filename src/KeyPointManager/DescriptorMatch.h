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
    DescriptorMatch(bool bShowMatchResult = false, EnMatchMethod eMethod = EnBrutForceInWindow): mEnMatchMethod(eMethod), mShowMatchResult(bShowMatchResult){}
    TpDescriptorMatchResult  match(const Frame& fFrame, const TpOneFrameKptDescriptor& fKptsDesc);
protected:
    TpDescriptorMatchResult     matchByKnn(const Frame& fFrame, const TpOneFrameKptDescriptor& fKptsDesc);
    TpDescriptorMatchResult     matchByBrutForceInWindow(const Frame& fFrame, const TpOneFrameKptDescriptor& fKptsDesc);
    
    void                        showMatchResult(const Frame& fFrame, const TpOneFrameKptDescriptor& fKptsDesc, const TpDescriptorMatchResult& mBestVecMatchResult);
private:
    const EnMatchMethod         mEnMatchMethod; 
    const bool                  mShowMatchResult;
};   

typedef std::shared_ptr<DescriptorMatch> PtrDescriptorMatch;
    
}
}

#endif

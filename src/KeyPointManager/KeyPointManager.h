#ifndef __KEYPOINTMANAGER_H__
#define __KEYPOINTMANAGER_H__

#include <memory>
#include "../Type/type.h"
#include "ORBextractor.h"
#include "FrameKptsDescriptor.h"
#include "DescriptorMatch.h"

namespace PKVIO{
namespace KeyPointManager{
    
class KeyPointManager{
public:
    KeyPointManager(void);
    ~KeyPointManager();
    
    typedef std::shared_ptr<ORB_SLAM2::ORBextractor> TpFeatureExtractor;
    
    void                    extract(const Frame& f, TpOneFrameKptDescriptor& mKptsDescriptors);
    void                    solve(const Frame& f);
    
    inline bool             queryDescriptorExisting(const TpFrameID nFrmID){ return mFrameKptsDescriptorHistoryRecord.isExisting(nFrmID); }
    TpOneFrameKptDescriptor& getDescriptor(const TpFrameID nFrmID) { return mFrameKptsDescriptorHistoryRecord.get(nFrmID); }
protected:
    void                    track(const Frame& fCurFrame, const TpOneFrameKptDescriptor& fCurFrameKptDescriptor);
    const Frame&            getLastFrame(void){
        
    }
private:
    inline void             initialize(void);
    
    void                    initializeFeatureExtroctor(void);
    void                    initializeFeatureMatcher(void);
    inline void             initializeHistoryRecord(void);
    
    
    TpFeatureExtractor      mPtrORBExtractorLeft;
    TpFeatureExtractor      mPtrORBExtractorRight;
    
    PtrDescriptorMatch      mPtrDesciptorMatcher;
    
    FrameKptsDescriptorHistory  mFrameKptsDescriptorHistoryRecord;
    StereoFrameHistory                mFrameHistoryRecord;
}; 

}
}

#endif

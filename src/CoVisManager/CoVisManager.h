#ifndef __COVISMANAGER_H__
#define __COVISMANAGER_H__

#include "../KeyPointManager/KeyPointManager.h"
#include "../Type/type.h"
#include "KeyPointIDManager.h"

namespace PKVIO
{
namespace CoVisManager
{

typedef std::vector<TpFrameID> TpVecCoVisFrameIDs;
    
class CoVisManager
{
public:
    void solve(const Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult);
    
    TpVecCoVisFrameIDs getCoVisFrameIDs(const TpFrameID nFrameIDQuery) const;
    
protected:
    TpMapFrameID2FrameIndex initFrameID2FrameIndexOfMatchResult(const KeyPointManager::FrameMatchResult& mFrameMatchResult);
    
    void copyIDToCurFrameOrGenerateIDForBothMatchFrames(const KeyPointManager::FrameMatchResult& mFrameMatchResult);
    
    void updateCoVisGraph(void);
private:
    KeyPointManager::KeyPointIDManager mKeyPointIDManager;
};
    
}
}

#endif

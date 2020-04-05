#ifndef __COVISMANAGER_H__
#define __COVISMANAGER_H__

#include "../KeyPointManager/KeyPointManager.h"
#include "../Type/type.h"
#include "KeyPointIDManager.h"
#include "CoVisGraph.h"

namespace PKVIO
{
namespace CoVisManager
{

typedef std::vector<TpFrameID> TpVecCoVisFrameIDs;
    
class CoVisManager
{
public:
    CoVisManager();
    
    void                        solve(const Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult);
    
    TpVecCoVisFrameIDs          getCoVisFrameIDs(const TpFrameID nFrameIDQuery) const;
    
    inline KeyPointManager::TpOneFrameIDManager& 
                                getCurrentFrameKptIDMgr(void){
                                    throw;
                                    //TODO: how to get current frame ID.
                                }
    inline KeyPointManager::TpOneFrameIDManager& 
                                getFrameKptIDMgr(const TpFrameID& nFrameID){ return mKeyPointIDManager.OneFrameIDManager(nFrameID); }
protected:
    TpMapFrameID2FrameIndex     initFrameID2FrameIndexOfMatchResult(const KeyPointManager::FrameMatchResult& mFrameMatchResult);
    
    int                         copyIDToCurFrameOrGenerateIDForBothMatchFrames(const KeyPointManager::FrameMatchResult& mFrameMatchResult);
    
    void                        updateCoVisGraph(void);
    

private:
    KeyPointManager::KeyPointIDManager mKeyPointIDManager;
    TpPtrCoVisGraph                    mPtrCoVisGraph;
};
    
}
}

#endif

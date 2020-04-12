#ifndef __COVISMANAGER_H__
#define __COVISMANAGER_H__

#include "../KeyPointManager/KeyPointManager.h"
#include "../Type/type.h"
#include "KeyPointIDManager.h"
#include "CoVisGraph.h"
#include "../DebugManager/DebugManager.h"

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
    
    void                        getCoVis(const TpFrameID nFrameIDFrom, const TpFrameID nFrameIDTo, TpVecKeyPointID& nCovisKptID, TpVecMatchResult& nCoVisMatch);
    
    TpVecMatchResult            getCoVis(const TpFrameID nFrameIDFrom, const TpFrameID nFrameIDTo);
    void                        getCoVis(const TpFrameID nFrameIDFrom, const TpFrameID nFrameIDTo, TpVecMatchResult& nCoVisMatch);
    
    inline KeyPointManager::TpOneFrameIDManager& 
                                getCurrentFrameKptIDMgr(void){
                                    throw;
                                    //TODO: how to get current frame ID.
                                }
    inline KeyPointManager::TpOneFrameIDManager& 
                                OneFrameKptIDMgrByFrameID(const TpFrameID& nFrameID){ return mKeyPointIDManager.OneFrameIDManager(nFrameID); }
                                
    template<typename Func>
    void                        collectCoVisInfo(const TpFrameID& nFrameID, Func f){
        
                                    auto pNodeCur               = mPtrCoVisGraph->getFrameByFrameID(nFrameID);
                                    auto FuncVisitNode          = [&](CoVisGraph::TpPtrNode& pNodeFrom, CoVisGraph::TpPtrNode& pNodeTo){
                                        bool nbIsNodeCurFrame   = (pNodeFrom == nullptr);
                                        //if(nbIsNodeCurFrame) {
                                        //    return;   
                                        //}
                                        
                                        //const TpFrameID nFrameIDCur = getFrameIDTemplate(pNode->getData());
                                        const TpFrameID nFrameIDCur = nFrameID;
                                        
                                        const TpFrameID nFrameIDTo  = getFrameIDTemplate(pNodeTo->getData());
                                        
                                        f(nFrameIDTo);
                                    };
                                    
                                    auto FuncSkipNode = [&](CoVisGraph::TpPtrNode& pNodeFrom, CoVisGraph::TpPtrNode& pNodeTo){
                                        //const TpFrameID nFrameIDTo = pNodeTo->getData().FrameID();
                                        return false;
                                    };
    
                                    mPtrCoVisGraph->BreadthFristSearch(pNodeCur, 2, FuncVisitNode, FuncSkipNode);
                                }
protected:
    TpMapFrameID2FrameIndex     initFrameID2FrameIndexOfMatchResult(const KeyPointManager::FrameMatchResult& mFrameMatchResult);
    
    int                         copyIDToCurFrameOrGenerateIDForBothMatchFrames(const KeyPointManager::FrameMatchResult& mFrameMatchResult);
    
    void                        updateCoVisGraph(CoVisGraph::TpPtrNode& pNodeCurFrame);
    void                        updateCoVisGraph(CoVisGraph::TpPtrNode& pNodeCurFrame, const TpVecFrameID& nVecFrameIDsDirectAdjoin);
    

private:
    KeyPointManager::KeyPointIDManager mKeyPointIDManager;
    TpPtrCoVisGraph                    mPtrCoVisGraph;
    DebugManager::DebugCoVisInfo       mDebugCoVisInfo; 
};
    
}
}

#endif

#include "CoVisManager.h"
#include <map>
#include <iostream>
#include "DescriptorMatch.h"
#include "../Tools/Tools.h"

using namespace std;

namespace PKVIO
{

namespace CoVisManager
{

CoVisManager::CoVisManager() 
: mPtrCoVisGraph(std::make_shared<CoVisGraph>())
{
    
}
    
void CoVisManager::solve (const Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult ) 
{
    auto FuncLogDebugCoVisInfo = [&](){
        mDebugCoVisInfo.mFrameID = fFrame.FrameID();
        mDebugCoVisInfo.log();
    };
    Tools::Timer tTimer(FuncLogDebugCoVisInfo, "CoVis Whole", false ,&mDebugCoVisInfo.mTimeCostWhole);
    
    const TpFrameID nCurFrameID = fFrame.FrameID();
    int nCountKptsOnThisFrame = mFrameMatchResult.getCountKptsOnThisFrame();
    mKeyPointIDManager.addOneFrameIDManager(fFrame.getFrameIndex(), nCurFrameID, nCountKptsOnThisFrame);
    
    int nCountSumTrackKpts = copyIDToCurFrameOrGenerateIDForBothMatchFrames(mFrameMatchResult);
}


Type::TpMapFrameID2FrameIndex CoVisManager::initFrameID2FrameIndexOfMatchResult(const KeyPointManager::FrameMatchResult& mFrameMatchResult) {
    TpMapFrameID2FrameIndex mMapFrameID2FrameIndex;
    int nSzMatch = mFrameMatchResult.size();
    for(int nIdxMatch =0; nIdxMatch<nSzMatch; ++nIdxMatch) {
        auto mLeftAndRightFrameMatchResult = mFrameMatchResult.getFrameDescriptoreMatchResult(nIdxMatch);

        const TpFrameID nLeftFrameID = mLeftAndRightFrameMatchResult.getFrameIDLeft();
        mMapFrameID2FrameIndex[nLeftFrameID] = mKeyPointIDManager.getFrameIndex(nLeftFrameID);

        const TpFrameID nRightFrameID = mLeftAndRightFrameMatchResult.getFrameIDRight();
        mMapFrameID2FrameIndex[nRightFrameID] = mKeyPointIDManager.getFrameIndex(nRightFrameID);
    }
    return mMapFrameID2FrameIndex;
}

int CoVisManager::copyIDToCurFrameOrGenerateIDForBothMatchFrames(const KeyPointManager::FrameMatchResult& mFrameMatchResult)
{
    int nCountSumTrackKpts = 0;
    
    int nSzMatch = mFrameMatchResult.size();
    {
        set<TpFrameID> nSetFrameIDSlavers;
        for(int nIdxMatch=0;nIdxMatch<nSzMatch;++nIdxMatch)
            nSetFrameIDSlavers.insert(mFrameMatchResult.getFrameDescriptoreMatchResult(nIdxMatch).getFrameIDRight());
        if(nSetFrameIDSlavers.size()>1){
            map<TpFrameID, vector<int>> nMapFrameID2MatchIndex;
            for(int nIdxMatch=0;nIdxMatch<nSzMatch;++nIdxMatch){
                nMapFrameID2MatchIndex[mFrameMatchResult.getFrameDescriptoreMatchResult(nIdxMatch).getFrameIDRight()].push_back(nIdxMatch);
            }
            
            for(auto Iter = nMapFrameID2MatchIndex.begin(),EndIter= nMapFrameID2MatchIndex.end();Iter!=EndIter;++Iter){
                auto& nVecMatchIndex = Iter->second;
                KeyPointManager::FrameMatchResult nOneFrameIDMatchResult;
                for(int nIdxMatchIndex=0,nSzMatchIndex=nVecMatchIndex.size();nIdxMatchIndex<nSzMatchIndex;++nIdxMatchIndex){
                    auto& nOneFrameMatchResult = const_cast<KeyPointManager::TpDescriptorMatchResult&> (mFrameMatchResult.getFrameDescriptoreMatchResult(nVecMatchIndex[nIdxMatchIndex]));
                    nOneFrameIDMatchResult.pushFrameDescriptorMatchResult(nOneFrameMatchResult);
                }
                nCountSumTrackKpts += copyIDToCurFrameOrGenerateIDForBothMatchFrames(nOneFrameIDMatchResult);
            }
            return nCountSumTrackKpts;
        }
    }
    
    // The bellow code while update co-vis graph for each new frame, so the code above will make sure the matchresults are from same one frame.
    if(nSzMatch<1 || (nSzMatch == 1 && mFrameMatchResult.isExistInnerFrameDescriptorMatchResult()) )
        return nCountSumTrackKpts;
    
    // about 0.05ms for one Co-Vis Frame Pair.
    //Tools::Timer CoVisCopyIDTimer("CoVis: copyID");
    
    const TpFrameID nCurFrameID = mFrameMatchResult.getFrameDescriptoreMatchResult(0).getFrameIDRight();
    TpMapFrameID2FrameIndex mMapFrameID2FrameIndex = initFrameID2FrameIndexOfMatchResult(mFrameMatchResult);
    
    TpVecFrameID nVecFrameIDsDirectAdjoin;
    
    //TODO: if stereo, two keypoints in left and right should have a ID when they satisfy the rule below.
    int nSzOuterMatch = mFrameMatchResult.sizeOuterFrameDescriptorMatchResult();
    TpVecCoVisFramePairAndWeight nVecCoVisFramePairAndWeight;
    for(int nIdxOuterMatch =0; nIdxOuterMatch<nSzOuterMatch; ++nIdxOuterMatch)
    {
        auto& mPrevAndCurFrameMatchResult    = mFrameMatchResult.getOuterFrameDescriptorMatchResult(nIdxOuterMatch);
        
        const TpFrameID     nPrevFrameID    = mPrevAndCurFrameMatchResult.getFrameIDLeft();
        const TpFrameIndex  nPrevFrameIndex = mMapFrameID2FrameIndex[nPrevFrameID];
        
        const TpFrameID     nCurFrameID     = mPrevAndCurFrameMatchResult.getFrameIDRight();
        const TpFrameIndex  nCurFrameIndex  = mMapFrameID2FrameIndex[nCurFrameID];
        
        int nKptIdxInPrev, nKptIdxInCur;
        int nSzKptsMatch                    = mPrevAndCurFrameMatchResult.getCountMatchKpts();
        KeyPointManager::TpOneFrameIDManager& mPrevFrameIDMgr = mKeyPointIDManager.OneFrameIDManagerByFrameIndex(nPrevFrameIndex);
        KeyPointManager::TpOneFrameIDManager& mCurFrameIDMgr  = mKeyPointIDManager.OneFrameIDManagerByFrameIndex(nCurFrameIndex);
        
        int nCountCoVisID = 0;
        
        for(int nIdxKptMatch = 0;nIdxKptMatch<nSzKptsMatch;++nIdxKptMatch)
        {
            mPrevAndCurFrameMatchResult.getMatchKptIndex((const int)nIdxKptMatch, nKptIdxInPrev, nKptIdxInCur);
            TpKeyPointID& mKptIDInPrev = mPrevFrameIDMgr.KeyPointID(nKptIdxInPrev);
            
            bool bNewKptID = false;
            if(Type::isInvalideKeyPointID(mKptIDInPrev)){
                //TODO : should have enough parallax to be able triangule a new 3d point, then general ID.
                mPrevFrameIDMgr.InitializeKptID(mKptIDInPrev, mKeyPointIDManager.GenerateKeyPointID());
                bNewKptID = true;
            }
            
            if(!Type::isInvalideKeyPointID(mKptIDInPrev)){
                // prev kpt has got a ID, copy it to its corresponding kpts in current frame.
                TpKeyPointID& mKptIDInCur = mCurFrameIDMgr.KeyPointID(nKptIdxInCur);
                if(!Type::isInvalideFrameID(mKptIDInCur)) { 
                    //TODO : need merge keypoint
                    cout << "Warning: need merge keypoint or bug in code. exit"<<endl;
                    throw;
                }
                mCurFrameIDMgr.InitializeKptID(mKptIDInCur, mKptIDInPrev);
                if(bNewKptID){
                    mCurFrameIDMgr.setKptIDIsFirstDetectedDueToCurrentFrame(nKptIdxInCur, mKptIDInCur);
                }
                ++nCountCoVisID;
            }
        }
        
        if(nCountCoVisID>0){
            assert(nCountCoVisID ==  mKeyPointIDManager.sizeCoVisKptIDs(nPrevFrameID, nCurFrameID));
            nVecCoVisFramePairAndWeight.push_back(CoVisFramePairAndWeight(nPrevFrameID, nCurFrameID, nCountCoVisID));
            
            nVecFrameIDsDirectAdjoin.push_back(nPrevFrameID);
        }
        nCountSumTrackKpts += nCountCoVisID;
    }
    //cout << "build covis begin ..." <<endl;
    mPtrCoVisGraph->buildCoVisBetween(nVecCoVisFramePairAndWeight);
    //cout << "build covis end ..." <<endl;
    
    auto pNodeCurFrame = mPtrCoVisGraph->getFrameByFrameID(nCurFrameID);
    // method #1: finnal used.
    //updateCoVisGraph(pNodeCurFrame);
    // method #2: test code.
    updateCoVisGraph(pNodeCurFrame, nVecFrameIDsDirectAdjoin);
    
    return nCountSumTrackKpts;
}


void CoVisManager::updateCoVisGraph(CoVisGraph::TpPtrNode& pNodeCurFrame) {
    // it's better to be implemented by adviser-notifier.
    //TODO
    
    auto FuncVisitNode = [&](CoVisGraph::TpPtrNode& pNodeFrom, CoVisGraph::TpPtrNode& pNodeTo){
        bool nbIsNodeCurFrame = (pNodeFrom == nullptr);
        if(nbIsNodeCurFrame) return;
        
        const TpFrameID nFrameIDCur = getFrameIDTemplate(pNodeCurFrame->getData());
        const TpFrameID nFrameIDTo  = getFrameIDTemplate(pNodeTo->getData());
        
        // TODO: need optimize
        const int nSzCosVisKptIDs   = mKeyPointIDManager.sizeCoVisKptIDs(nFrameIDCur, nFrameIDTo);
        //if(nSzCosVisKptIDs>10)
        mPtrCoVisGraph->buildCoVisBetween(pNodeTo, pNodeCurFrame, CoVisFramePairAndWeight(nFrameIDTo, nFrameIDCur, nSzCosVisKptIDs));
    };
    auto FuncSkipNode = [&](CoVisGraph::TpPtrNode& pNodeFrom, CoVisGraph::TpPtrNode& pNodeTo){
        return false;
    };
    mPtrCoVisGraph->BreadthFristSearch(pNodeCurFrame, 3, FuncVisitNode, FuncSkipNode);
}

void CoVisManager::updateCoVisGraph(CoVisGraph::TpPtrNode& pNodeCurFrame, const Type::TpVecFrameID& nVecFrameIDsDirectAdjoin) 
{
    //cout << "update covis graph begin ..." <<endl;
    
    TpSetFrameID nSetFrameIDsDirectAdjoinToCurFrame(nVecFrameIDsDirectAdjoin.begin(),nVecFrameIDsDirectAdjoin.end());
    auto FuncVisitNode = [&](CoVisGraph::TpPtrNode& pNodeFrom, CoVisGraph::TpPtrNode& pNodeTo){
        bool nbIsNodeCurFrame = (pNodeFrom == nullptr);
        if(nbIsNodeCurFrame) return;
        
        const TpFrameID nFrameIDCur = getFrameIDTemplate(pNodeCurFrame->getData());
        const TpFrameID nFrameIDTo  = getFrameIDTemplate(pNodeTo->getData());
        
        //cout << "add covis between "<< nFrameIDTo << "->" << nFrameIDCur << "..." <<endl;
        // TODO: need optimize
        const int nSzCosVisKptIDs   = mKeyPointIDManager.sizeCoVisKptIDs(nFrameIDCur, nFrameIDTo);
        //if(nSzCosVisKptIDs>10)
        mPtrCoVisGraph->buildCoVisBetween(pNodeTo, pNodeCurFrame, CoVisFramePairAndWeight(nFrameIDTo, nFrameIDCur, nSzCosVisKptIDs));
        
        cout << "add covis between "<< nFrameIDTo << "->" << nFrameIDCur << "end." <<endl;
    };
    auto FuncSkipNode = [&](CoVisGraph::TpPtrNode& pNodeFrom, CoVisGraph::TpPtrNode& pNodeTo){
        const TpFrameID nFrameIDTo = pNodeTo->getData().FrameID();
        if(nSetFrameIDsDirectAdjoinToCurFrame.count(nFrameIDTo))
            return true;
        return false;
    };
    mPtrCoVisGraph->BreadthFristSearch(pNodeCurFrame, 3, FuncVisitNode, FuncSkipNode);    
}

TpVecCoVisFrameIDs CoVisManager::getCoVisFrameIDs(const Type::TpFrameID nFrameIDQuery) const {
    TpVecCoVisFrameIDs vCoVisFrameIDs;
    //TODO
    throw;
    return vCoVisFrameIDs;
}

    
}

}

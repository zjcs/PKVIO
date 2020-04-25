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
    //int nCountKptsOnThisFrame = mFrameMatchResult.getCountKptsOnThisFrame();
    int nCountKptsOnThisFrame = mFrameMatchResult.getInnerFrameDescriptorMatchResult().getCountKptsLeft();
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
    std::map<TpKeyPointID, TpKeyPointIndex> nMapKptID2KptIdxCur;
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
        
        // 1. try to new KptID, according this track;
        int nCountCoVisID = 0;
        std::vector<int> nVecMatchIndexDuplicatedToDelete; 
        for(int nIdxKptMatch = 0;nIdxKptMatch<nSzKptsMatch;++nIdxKptMatch)
        {
            mPrevAndCurFrameMatchResult.getMatchKptIndex((const int)nIdxKptMatch, nKptIdxInPrev, nKptIdxInCur);
            TpKeyPointID& mKptIDInPrev = mPrevFrameIDMgr.KeyPointID(nKptIdxInPrev);
            TpKeyPointID& mKptIDInCur = mCurFrameIDMgr.KeyPointID(nKptIdxInCur);
            
            bool nBoolCoVis = false;
            bool bNewKptID = false;
            if(Type::isInvalideKeyPointID(mKptIDInPrev)){
                //TODO : should have enough parallax to be able triangule a new 3d point, then general ID.
                if(Type::isValideKeyPointID(mKptIDInCur)){  // Try to fix bug below(merge keypoint), it's not a finnal solution and only 2Frame tracked is Okay.
                    nBoolCoVis = false;
                    // have bug, mKptIDInCur maybe conflicted with KptIDs in nFrameIDPrev; so delete the co-vis;
                    //mPrevFrameIDMgr.InitializeKptID(mKptIDInPrev, mKptIDInCur);
                    //nBoolCoVis = true;
                }else{
                    mPrevFrameIDMgr.InitializeKptID(mKptIDInPrev, mKeyPointIDManager.GenerateKeyPointID());
                    //cout << "New KptID:KptID|FrmPrev|FrmCur" << mKptIDInPrev<<"|"<<nPrevFrameID<<"|"<<nCurFrameID<<endl;
                    bNewKptID = true;
                    nBoolCoVis = false;
                }
            }
            
            if(Type::isValideKeyPointID(mKptIDInPrev)){
                // prev kpt has got a ID, copy it to its corresponding kpts in current frame.
                if(Type::isValideKeyPointID(mKptIDInCur) && (bNewKptID || mKptIDInPrev!=mKptIDInCur)) { 
                    //TODO : need merge keypoint, this problem is caused of dis-continue co-vis merged by one keypoint.
                    // Optical-Flow donot have this problem, only descriptor-based keypoint match is effected.
                    if(bNewKptID){
                        cout << "Warning: need merge keypoint or bug in code. exit"<<endl;
                        throw;
                        nBoolCoVis = true;
                    }else{
                        nBoolCoVis = false;
                    }
                }else{
                    auto Iter = nMapKptID2KptIdxCur.find(mKptIDInPrev);
                    if(Iter == nMapKptID2KptIdxCur.end() || Iter->second == nKptIdxInCur){
                        nBoolCoVis = true;
                        mCurFrameIDMgr.InitializeKptID(mKptIDInCur, mKptIDInPrev);
                        nMapKptID2KptIdxCur[mKptIDInCur] = nKptIdxInCur;
                        if(bNewKptID){
                            mCurFrameIDMgr.setKptIDIsFirstDetectedDueToCurrentFrame(nKptIdxInCur, mKptIDInCur);
                        }
                    }else{
                        nBoolCoVis = false;
                    }
                }
            }else{
                if(nBoolCoVis)
                    throw;
            }
            
            if(nBoolCoVis){
                //cout << "CoVis1:" << nCountCoVisID << " - " << mKptIDInCur << endl;
                ++nCountCoVisID;
            }else{
                nVecMatchIndexDuplicatedToDelete.push_back(nIdxKptMatch);
            }
        }
        
        const_cast<KeyPointManager::TpDescriptorMatchResult&>(mPrevAndCurFrameMatchResult).deleteMatch(nVecMatchIndexDuplicatedToDelete);
        
        
        // 2. count their co-vis kpt number, and build co-vis.
        if(nCountCoVisID>0){
            // the assert isnot true now, due to the wrong corresponding between KF1vFm2 and KF1vFm3;
            //assert(nCountCoVisID ==  mKeyPointIDManager.sizeCoVisKptIDs(nPrevFrameID, nCurFrameID));
            nVecCoVisFramePairAndWeight.push_back(CoVisFramePairAndWeight(nPrevFrameID, nCurFrameID, nCountCoVisID));
            
            nVecFrameIDsDirectAdjoin.push_back(nPrevFrameID);
        }
        nCountSumTrackKpts += nCountCoVisID;
    }
    
    // Method 2: directly find co-vis Keypoint between current frame and KF which co-vis KptID and MpID,
    //           which is different to find co-vis by descriptor in KeyPointManager::track;
    if(true && mFuncGetCoVisKFFrameID){
        TpVecFrameID nVecTryToCoVisKFFrmID = mFuncGetCoVisKFFrameID(nCurFrameID); // how to get it.
        for(size_t nIdx=0,nSz = nVecTryToCoVisKFFrmID.size();nIdx<nSz;++nIdx){
            TpFrameID nFrmIDKF = nVecTryToCoVisKFFrmID[nIdx];
            if(std::find(nVecFrameIDsDirectAdjoin.begin(), nVecFrameIDsDirectAdjoin.end(), nFrmIDKF)==nVecFrameIDsDirectAdjoin.end()){
                int nCountCoVisID =  mKeyPointIDManager.sizeCoVisKptIDs(nFrmIDKF, nCurFrameID);
                if(nCountCoVisID>0){
                    nVecCoVisFramePairAndWeight.push_back(CoVisFramePairAndWeight(nFrmIDKF, nCurFrameID, nCountCoVisID));
                    nVecFrameIDsDirectAdjoin.push_back(nFrmIDKF);
                }
            }
        }
    }
    
    cout << "add CoVis: By Frame|KeyFrame..." <<endl;
    
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
    cout << "add CoVis: By CoVisGraph BFS..." <<endl;
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
    cout << "add CoVis: By CoVisGraph BFS" <<endl;
    
    TpSetFrameID nSetFrameIDsDirectAdjoinToCurFrame(nVecFrameIDsDirectAdjoin.begin(),nVecFrameIDsDirectAdjoin.end());
    auto FuncVisitNode = [&](CoVisGraph::TpPtrNode& pNodeFrom, CoVisGraph::TpPtrNode& pNodeTo){
        bool nbIsNodeCurFrame = (pNodeFrom == nullptr);
        if(nbIsNodeCurFrame) return;
        
        const TpFrameID nFrameIDCur = getFrameIDTemplate(pNodeCurFrame->getData());
        const TpFrameID nFrameIDTo  = getFrameIDTemplate(pNodeTo->getData());
        const TpFrameID nFrameIDFrom= getFrameIDTemplate(pNodeFrom->getData());
        
        //cout << "add covis between "<< nFrameIDTo << "->" << nFrameIDCur << "..." <<endl;
        // TODO: need optimize
        const int nSzCosVisKptIDs   = mKeyPointIDManager.sizeCoVisKptIDs(nFrameIDCur, nFrameIDTo);
        const int nSzCosVisKptIDsFT = mKeyPointIDManager.sizeCoVisKptIDs(nFrameIDFrom, nFrameIDTo);
        
        // not visit pNodeTo from this edge between pNodeFrom and pNodeTo;
        if(nSetFrameIDsDirectAdjoinToCurFrame.count(nFrameIDTo))return;
        //if((nSzCosVisKptIDsFT<60 || nSzCosVisKptIDs<50)){ return; }
        
        //cout << "add covis(w)"<< nSzCosVisKptIDs <<" between "<< nFrameIDCur << "->" << nFrameIDTo << " via " << nFrameIDFrom << "->"<< nFrameIDTo <<endl;
        mPtrCoVisGraph->buildCoVisBetween(pNodeTo, pNodeCurFrame, CoVisFramePairAndWeight(nFrameIDTo, nFrameIDCur, nSzCosVisKptIDs));
        
    };
    auto FuncSkipNode = [&](CoVisGraph::TpPtrNode& pNodeFrom, CoVisGraph::TpPtrNode& pNodeTo){
        const TpFrameID nFrameIDCur = getFrameIDTemplate(pNodeCurFrame->getData());
        const TpFrameID nFrameIDTo = pNodeTo->getData().FrameID();
        // always skip some node.
        if(nFrameIDCur - nFrameIDTo>DebugManager::DebugControl().mCountMaxCoVisFrame)
            return true;
        return false;
    };
    mPtrCoVisGraph->BreadthFristSearch(pNodeCurFrame, DebugManager::DebugControl().mMaxGraphDepthToBuildCoVis, FuncVisitNode, FuncSkipNode);    
}

TpVecCoVisFrameIDs CoVisManager::getCoVisFrameIDs(const Type::TpFrameID nFrameIDQuery) const {
    TpVecCoVisFrameIDs vCoVisFrameIDs;
    //TODO
    throw;
    return vCoVisFrameIDs;
}

void CoVisManager::getCoVis(const Type::TpFrameID nFrameIDFrom, const Type::TpFrameID nFrameIDTo,
                            Type::TpVecKeyPointID& nCovisKptID, Type::TpVecMatchResult& nCoVisMatch) 
{
    TpVecKeyPointID nVecKptIDsFrom; TpVecKeyPointIndex nVecKptIndexsFrom;
    OneFrameKptIDMgrByFrameID(nFrameIDFrom).getAllKptIDsAndIdexs(nVecKptIDsFrom, nVecKptIndexsFrom);
    
    TpVecKeyPointID nVecKptIDsTo; TpVecKeyPointIndex nVecKptIndexsTo;
    OneFrameKptIDMgrByFrameID(nFrameIDTo).getAllKptIDsAndIdexs(nVecKptIDsTo, nVecKptIndexsTo);
    
    auto nMapKptID2KptIdxFrom = Tools::buildMap(nVecKptIDsFrom, nVecKptIndexsFrom);
    for(int nIdx=0,nSz= nVecKptIDsTo.size();nIdx<nSz;++nIdx){
        auto nKptIDTo = nVecKptIDsTo[nIdx];
        auto Iter = nMapKptID2KptIdxFrom.find(nKptIDTo);
        if(Iter == nMapKptID2KptIdxFrom.end())
            continue;
        nCovisKptID.push_back(nKptIDTo);
        nCoVisMatch.push_back(cv::DMatch(Iter->second, nVecKptIndexsTo[nIdx], -1));
    }
}


void CoVisManager::getCoVis(const Type::TpFrameID nFrameIDFrom, const Type::TpFrameID nFrameIDTo,
                            Type::TpVecMatchResult& nCoVisMatch) 
{
    Type::TpVecKeyPointID nCovisKptID;
    getCoVis(nFrameIDFrom, nFrameIDTo, nCovisKptID, nCoVisMatch);
}


Type::TpVecMatchResult CoVisManager::getCoVis(const Type::TpFrameID nFrameIDFrom, const Type::TpFrameID nFrameIDTo) 
{
    Type::TpVecMatchResult nCoVisMatch;
    getCoVis(nFrameIDFrom, nFrameIDTo, nCoVisMatch);
    return nCoVisMatch;
}


std::vector<CoVisManager::TpVecFrameIDKptIndexPair> 
CoVisManager::getCoVis(const Type::TpFrameID nFrameIDFrom, const Type::TpVecKeyPointID& nVecKptID) 
{
    int nSzKptID = nVecKptID.size();
    std::vector<TpVecFrameIDKptIndexPair>  nCoVisResult(nVecKptID.size(), TpVecFrameIDKptIndexPair());
    const TpFrameIndex nFrameIndexFrom = mKeyPointIDManager.getFrameIndex(nFrameIDFrom);
    TpVecKeyPointID nCovisKptID = nVecKptID;
    for(int nIdxFrm=nFrameIDFrom;nIdxFrm>=0;--nIdxFrm){
        KeyPointManager::TpOneFrameIDManager& nFrmIDMgr = mKeyPointIDManager.OneFrameIDManagerByFrameIndex(nIdxFrm);
        TpFrameID nFrmID = mKeyPointIDManager.getFrameID(nIdxFrm);
        TpVecKeyPointIndex nVecKptIdx = nFrmIDMgr.getKptIndex(nCovisKptID);
        vector<bool> nVecBoolCoVisKptIdx(nVecKptIdx.size(), false);
        for(int nIdxKptID=0;nIdxKptID<nVecKptIdx.size();++nIdxKptID){
            TpKeyPointIndex& nKptIdx = nVecKptIdx[nIdxKptID];
            if(Type::isValideKeyPointIndex(nKptIdx)){
                nCoVisResult[nIdxKptID].push_back(std::make_pair(nFrmID, nKptIdx));
                nVecBoolCoVisKptIdx[nIdxKptID] = true;
            }
        }
        
        if(DebugManager::DebugControl().mBoolTrackByOpticalFlow){
            Tools::filter(nCovisKptID, nVecBoolCoVisKptIdx);
            if(nCovisKptID.size()==0)
                break;
        }
    }
    return nCoVisResult;
}

}

}

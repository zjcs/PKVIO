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
    Tools::Timer tTimer("CoVis Whole");
    
    const TpFrameID nCurFrameID = fFrame.FrameID();
    int nCountKptsOnThisFrame = mFrameMatchResult.getCountKptsOnThisFrame();
    mKeyPointIDManager.addOneFrameIDManager(fFrame.getFrameIndex(), nCurFrameID, nCountKptsOnThisFrame);
    
    int nCountSumTrackKpts = copyIDToCurFrameOrGenerateIDForBothMatchFrames(mFrameMatchResult);
    
    updateCoVisGraph();
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
    if(nSzMatch<1 || (nSzMatch == 1 && mFrameMatchResult.isExistInnerFrameDescriptorMatchResult()) )
        return nCountSumTrackKpts;
    
    // about 0.05ms for one Co-Vis Frame Pair.
    //Tools::Timer CoVisCopyIDTimer("CoVis: copyID");
    
    TpMapFrameID2FrameIndex mMapFrameID2FrameIndex = initFrameID2FrameIndexOfMatchResult(mFrameMatchResult);
    
    
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
                    cout << "Warning: need merge keypoint or bug in code.exit"<<endl;
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
            nVecCoVisFramePairAndWeight.push_back(CoVisFramePairAndWeight(nPrevFrameID, nCurFrameID, nCountCoVisID));
        }
        nCountSumTrackKpts += nCountCoVisID;
    }
    
    
    return nCountSumTrackKpts;
}


void CoVisManager::updateCoVisGraph() {
    //TODO
}

TpVecCoVisFrameIDs CoVisManager::getCoVisFrameIDs(const Type::TpFrameID nFrameIDQuery) const {
    TpVecCoVisFrameIDs vCoVisFrameIDs;
    //TODO
    throw;
    return vCoVisFrameIDs;
}


}
}

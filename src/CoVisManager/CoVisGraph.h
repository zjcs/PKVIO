#ifndef __COVISGRAPH_H__
#define __COVISGRAPH_H__

#include "../Type/type.h"
#include <memory>
#include <set>

namespace PKVIO
{
    
namespace GraphNode{
template<typename T>
const Type::TpFrameID getFrameIDTemplate(const T& nNodeData){
    return nNodeData.FrameID();
}
// function template full speciatialize should seperate the declare and defination.
}    
    
namespace CoVisManager {
    
class CoVisGraph;
class CoVisFramePairAndWeight
{
public:
    CoVisFramePairAndWeight(const TpFrameID nFrmIDMaster, const TpFrameID nFrmIDSlave, const int nWeightCountCosVisPts)
    :mFrmIDMaster(nFrmIDMaster),mFrmIDSlave(nFrmIDSlave), mWeightCountCosVisPts(nWeightCountCosVisPts) {}
    
    const TpFrameID& getFrameIDMaster(void)const { return mFrmIDMaster;}
    const TpFrameID& getFrameIDSlaver(void)const { return mFrmIDSlave;}
    const int        getSizeCoVisKpts(void)const { return mWeightCountCosVisPts;}
    const CoVisFramePairAndWeight   getWise(void) const { return CoVisFramePairAndWeight(mFrmIDSlave, mFrmIDMaster, mWeightCountCosVisPts);}
    
    const bool       operator==(const CoVisFramePairAndWeight& nCoVis)const{
        return this->isEqualStrict(nCoVis) || isEqualStrict(nCoVis.getWise());
    }
protected:
    bool            isEqualStrict(const CoVisFramePairAndWeight& nCoVis2) const {
        const CoVisFramePairAndWeight& nCoVis1 = *this;
        bool bEqualByFrameID            = nCoVis1.mFrmIDMaster == nCoVis2.mFrmIDMaster && nCoVis1.mFrmIDSlave == nCoVis2.mFrmIDSlave;
        bool bEqualByFrameIDAndWeight   = bEqualByFrameID & (nCoVis1.mWeightCountCosVisPts == nCoVis2.mWeightCountCosVisPts);
        assert(bEqualByFrameID == bEqualByFrameIDAndWeight);
        return bEqualByFrameIDAndWeight;
    }
private:
    TpFrameID       mFrmIDMaster, mFrmIDSlave;
    int             mWeightCountCosVisPts;
    
friend class        CoVisGraph;
    
};

typedef vector<CoVisFramePairAndWeight> TpVecCoVisFramePairAndWeight;

class TpCoVisFramePairAndWeightNodeData
{
public:
    TpCoVisFramePairAndWeightNodeData(const TpFrameID nFrameID, const CoVisFramePairAndWeight& nCoVisFramePairAndWeight)
    : mFrameID(nFrameID), mVecCosVisFramePairAndWeight(1, nCoVisFramePairAndWeight){}
    
    TpCoVisFramePairAndWeightNodeData(const TpFrameID nFrameID) 
    : mFrameID(nFrameID), mVecCosVisFramePairAndWeight(){}
    
    bool operator==(const TpFrameID& nFrameID) const {return mFrameID == nFrameID;}
    bool operator==(const TpCoVisFramePairAndWeightNodeData& nCoVisNodeData) const {
        bool bEqualByFrameID = mFrameID == nCoVisNodeData.mFrameID;
        bool bEqualByCoVisCount = mVecCosVisFramePairAndWeight.size() == nCoVisNodeData.mVecCosVisFramePairAndWeight.size();
        bool bEqualByEachData = true & bEqualByCoVisCount;
        for(int nIdxCoVis = 0,nSzCoVis = std::min(mVecCosVisFramePairAndWeight.size(),
            nCoVisNodeData.mVecCosVisFramePairAndWeight.size()); nIdxCoVis<nSzCoVis; ++nIdxCoVis){
            bool bEqualByOneData = mVecCosVisFramePairAndWeight[nIdxCoVis] == nCoVisNodeData.mVecCosVisFramePairAndWeight[nIdxCoVis];
            bEqualByEachData &= bEqualByOneData;
        }
        assert(bEqualByFrameID == bEqualByCoVisCount && bEqualByCoVisCount == bEqualByEachData);
        return bEqualByFrameID;
    }
    
    TpVecCoVisFramePairAndWeight&    VecCosVisFramePairAndWeight(void){return mVecCosVisFramePairAndWeight;}
    
    inline const TpFrameID  FrameID(void) const {return mFrameID;}
    
    void                    addCoVisFramePairAndWeight(const CoVisFramePairAndWeight& nCoVisFramePairAndWeight){
        if(nCoVisFramePairAndWeight.getFrameIDMaster() == mFrameID)
            mVecCosVisFramePairAndWeight.push_back(nCoVisFramePairAndWeight);
        else{
            assert(nCoVisFramePairAndWeight.getFrameIDSlaver() == mFrameID);
            mVecCosVisFramePairAndWeight.push_back(nCoVisFramePairAndWeight.getWise());
        }
    }
private:
    TpFrameID                       mFrameID;
    TpVecCoVisFramePairAndWeight    mVecCosVisFramePairAndWeight;
};

class TpCoVisFrameNodeData {
public:
    TpCoVisFrameNodeData(const TpFrameID nFrameID, const int nSzCosVisKptIDs)
    : mFrameID(nFrameID), mSzCosVisKptIDs(nSzCosVisKptIDs) {}
    
    // Error: operator== only need one parameter.
    //bool operator==(const TpCoVisFrameNodeData& nNodeData, const TpFrameID nFrameID){return nNodeData.mFrameID == nFrameID;}
    // the 'const' bellow is necesary.
    bool operator==(const TpFrameID& nFrameID) const {return mFrameID == nFrameID;}
    bool operator==(const TpCoVisFrameNodeData& nNodeData) const {return nNodeData.mFrameID == mFrameID;}
    
    inline const TpFrameID  FrameID(void) const {return mFrameID;}
private:
    TpFrameID   mFrameID;
    int         mSzCosVisKptIDs;
};

typedef Type::Graph<TpCoVisFrameNodeData>::Node TpCoVisFrameNode;
typedef Type::Graph<TpCoVisFrameNodeData>::Node* TpPtrCoVisFrameNode;

template<typename T>
const T         createCoVisNode(const TpFrameID nFrameID, const CoVisFramePairAndWeight& nCoVis){
    return T(nFrameID, nCoVis);
}
template<typename T>
const T         createCoVisNode(const TpFrameID nFrameID){
    return T(nFrameID);
}

template<>
const TpFrameID createCoVisNode(const TpFrameID nFrameID, const CoVisFramePairAndWeight& nCoVis);

//class CoVisFrameNode:pulic Type::Graph<CoVisFrameNodeData>::Node{ };

   
class CoVisGraph : public Type::Graph<TpCoVisFramePairAndWeightNodeData>
{
public:
    /*
    void buildCoVisBetween(const TpFrameID nFrmIDMaster, const TpFrameID nFrmIDSlave){
        auto pFrameNodeMaster = QueryAndAddIfNotInGraph(nFrmIDMaster);
        auto pFrameNodeSlave = QueryAndAddIfNotInGraph(nFrmIDSlave);
        this->addEdgeBidirectional(pFrameNodeMaster, pFrameNodeSlave);
    }
    */
    void buildCoVisBetween(TpPtrNode pNodeMaster, TpPtrNode pNodeSlaver, const CoVisFramePairAndWeight& nCoVisFramePairAndWeight){
        this->addEdgeBidirectional(pNodeMaster, pNodeSlaver);
   
        pNodeMaster->Data().addCoVisFramePairAndWeight(nCoVisFramePairAndWeight);
        pNodeSlaver->Data().addCoVisFramePairAndWeight(nCoVisFramePairAndWeight);
    }
    void buildCoVisBetween(const CoVisFramePairAndWeight& nCoVisFramePairAndWeight){
        
        auto pFrameNodeMaster = QueryAndAddIfNotInGraph(
            createCoVisNode<TpNodeData>(nCoVisFramePairAndWeight.mFrmIDMaster));
        auto pFrameNodeSlaver = QueryAndAddIfNotInGraph(
            createCoVisNode<TpNodeData>(nCoVisFramePairAndWeight.mFrmIDSlave));
        buildCoVisBetween(pFrameNodeMaster, pFrameNodeSlaver, nCoVisFramePairAndWeight);
    }
    void buildCoVisBetween(const TpVecCoVisFramePairAndWeight& nVecCoVisFramePairAndWeight){
        int nSzCoVis=nVecCoVisFramePairAndWeight.size();
        for(int nIdxCoVis=0;nIdxCoVis<nSzCoVis;++nIdxCoVis){
            buildCoVisBetween(nVecCoVisFramePairAndWeight[nIdxCoVis]);
        }
        //TODO : try to optimize the speed, if necessary while the graph is big.
        
        //set<TpFrameID> nSetFrameIDMasterToUse; 
        //set<TpFrameID> nSetFrameIDSlaveToUse; 
        //TpFrameID nFrameIDLastMaster = Type::INVALIDFRAMEID, nFrameIDLastSlave = Type::INVALIDFRAMEID;
        //for(int nIdxCoVis=0;nIdxCoVis<nSzCoVis;++nIdxCoVis){
        //    const auto& nCoVis = nVecCoVisFramePairAndWeight[nIdxCoVis];
        //    if(nCoVis.mFrmIDMaster!=nFrameIDLastMaster){
        //        nFrameIDLastMaster = nCoVis.mFrmIDMaster;
        //        nSetFrameIDMasterToUse.insert(nCoVis.mFrmIDMaster);
        //    }
        //    if(nCoVis.mFrmIDSlave!=nFrameIDLastSlave){
        //        nFrameIDLastSlave = nCoVis.mFrmIDSlave;
        //        nSetFrameIDSlaveToUse.insert(nCoVis.mFrmIDSlave);
        //    }
        //}
        //
        //TpVecFrameID nVecCoVisIndex2FrameIDMaster, nVecCoVisIndex2FrameIDSlave;
        //nVecCoVisIndex2FrameIDMaster.reserve(nSzCoVis), nVecCoVisIndex2FrameIDSlave.reserve(nSzCoVis);
        //auto Func = [&](TpPtrNode nPtrNode)->Type::EnForFlag{
        //    
        //    if(nSetFrameIDMasterToUse.count(nPtrNode->getData())){
        //        //nVecCoVisIndex2FrameIDMaster.push_back();
        //    }
        //    return Type::EnExec;
        //};
    }
    
    TpPtrNode getFrameByFrameID(const TpFrameID nFrameID){
        TpPtrNode pFrameNode = nullptr;
        auto Func = [&](TpPtrNode nPtrNode)->Type::EnForFlag{
            
            if(nPtrNode->getData() == nFrameID){
                pFrameNode = nPtrNode;
                return Type::EnBreak;
            }
            return Type::EnExec;
        };
        bool bFind = this->QueryNodeByFunc(Func);
        assert(bFind == (pFrameNode != nullptr));
        return pFrameNode;
        
    }
    
    TpPtrNode QueryAndAddIfNotInGraph(const TpNodeData& nNodeData){
        auto pFrameNode = getFrameByFrameID(getFrameIDTemplate(nNodeData));
        if(isNodeNull(pFrameNode)){
            pFrameNode = generateNodeAndAddIntoGraph(nNodeData);
        }
        return pFrameNode;
    }
};

typedef std::shared_ptr<CoVisGraph> TpPtrCoVisGraph;
    
}
}

#endif

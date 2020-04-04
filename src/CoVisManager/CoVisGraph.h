#ifndef __COVISGRAPH_H__
#define __COVISGRAPH_H__

#include "../Type/type.h"
#include <memory>
#include <set>

namespace PKVIO
{
namespace CoVisManager {
    
class CoVisGraph;
class CoVisFramePairAndWeight
{
public:
    CoVisFramePairAndWeight(const TpFrameID nFrmIDMaster, const TpFrameID nFrmIDSlave, const int nWeightCountCosVisPts)
    :mFrmIDMaster(nFrmIDMaster),mFrmIDSlave(nFrmIDSlave), mWeightCountCosVisPts(nWeightCountCosVisPts) {}
private:
    TpFrameID       mFrmIDMaster, mFrmIDSlave;
    int             mWeightCountCosVisPts;
    
friend class        CoVisGraph;
    
};

typedef vector<CoVisFramePairAndWeight> TpVecCoVisFramePairAndWeight;
   
class CoVisGraph : public Type::Graph<TpFrameID>
{
public:
    void buildCoVisBetween(const TpFrameID nFrmIDMaster, const TpFrameID nFrmIDSlave){
        
    }
    void buildCoVisBetween(const CoVisFramePairAndWeight& nCoVisFramePairAndWeight){
        auto pFrameNodeMaster = QueryAndAddIfNotInGraph(nCoVisFramePairAndWeight.mFrmIDMaster);
        auto pFrameNodeSlave = QueryAndAddIfNotInGraph(nCoVisFramePairAndWeight.mFrmIDSlave);
        this->addEdgeBidirectional(pFrameNodeMaster, pFrameNodeSlave);
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
        if(bFind != (pFrameNode == nullptr))
            throw;
        return pFrameNode;
        
    }
    
    TpPtrNode QueryAndAddIfNotInGraph(const TpFrameID nFrameID){
        auto pFrameNode = getFrameByFrameID(nFrameID);
        if(isNodeNull(pFrameNode)){
            pFrameNode = generateNodeAndAddIntoGraph(nFrameID);
        }
        return pFrameNode;
    }
};

typedef std::shared_ptr<CoVisGraph> TpPtrCoVisGraph;
    
}
}

#endif

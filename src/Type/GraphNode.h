#ifndef __GRAPHNODE_H__
#define __GRAPHNODE_H__

#include<vector>
#include "Frame.h"

using namespace std;


namespace PKVIO
{
namespace GraphNode 
{

typedef enum{
    EnExec,
    EnContinue,
    EnBreak,
    EnReturn
}EnForFlag;   

//template<typename T>
//const Type::TpFrameID getFrameIDTemplate(const T& nNodeData){
//    return nNodeData.FrameID();
//}
    
template<typename T>
class Graph
{
public:
    
typedef T                   TpNodeData;

class Node;
typedef Node*               TpPtrNode;
typedef vector<TpPtrNode>   TpVecPtrNodes;


inline bool isNodeNull(TpPtrNode p){return p==nullptr;}

virtual             ~Graph(){clearGraph();}

class Node
{
public:
                        Node(const T& nData):mPtrData(nData), mIndexInGraph(-1){}
    virtual             ~Node(){}

    //template<typename T>            friend typename Graph<T>::TpPtrNode           Graph<T>::generateNode(const T& nData);
    //friend              void                clearGraph(void);

public:
    void                initNodeIndexInGraph(const int nIndexInGraph){mIndexInGraph = nIndexInGraph; }
    const int           getNodeIndexInGraph(void) const {return mIndexInGraph;}

    //bool                operator==(const T& n){return mPtrData == n;}
    //const Type::TpFrameID     getFrameID(void){ return getFrameIDTemplate(mPtrData); }

    const int           sizeAdjoinNodes(void) const {return (int)mVecAdjoinNodes.size();}
    TpPtrNode           NodeByIndex(const int nAdjoinIndex){return mVecAdjoinNodes[nAdjoinIndex];}
    const TpPtrNode     getNodeByIndex(const int nAdjoinIndex)const{return mVecAdjoinNodes[nAdjoinIndex];}
    T&                  Data(void){return mPtrData;}
    const T&            getData(void)const {return mPtrData;}
    
    const bool          isEqualTo(const TpPtrNode pNode) const {
                            bool nbAjointByPtr      = this == pNode;
                            bool nbAjointByIndex    = this->mIndexInGraph == pNode->mIndexInGraph;
                            bool nbAjointByData     = this->getData() == pNode->getData();
                            
                            assert(nbAjointByPtr == nbAjointByIndex && nbAjointByIndex == nbAjointByData);
                            return nbAjointByData;
                            
                        }
                        
    const bool          isAdjoinTo(const TpPtrNode pNode)const{
                            return std::count_if(mVecAdjoinNodes.begin(), mVecAdjoinNodes.end(),
                                [&](const TpPtrNode& pNodeAdjoin){ return pNodeAdjoin->isEqualTo(pNode); });
                        };
    
    void                addAdjoin(TpPtrNode pAdjoint, bool bForce=false){
                            assert(isAdjoinTo(pAdjoint) == false);
                            mVecAdjoinNodes.push_back(pAdjoint);
                        }

    void                clear(void){ mIndexInGraph = -1; mVecAdjoinNodes.clear();}

    private:
    TpVecPtrNodes       mVecAdjoinNodes;
    T                   mPtrData;
    int                 mIndexInGraph;
};

protected:
TpPtrNode           generateNode(const T& nData);
void                clearGraph(void);
void                addIntoGraph(TpPtrNode pNode){pNode->initNodeIndexInGraph(sizeNodeInGraph()); mNodes.push_back(pNode);}
public:
TpPtrNode           generateNodeAndAddIntoGraph(const T& nData){ auto pNode = generateNode(nData); addIntoGraph(pNode); return pNode;}

void                addEdgeUndirectional(TpPtrNode pFromNode, TpPtrNode pToNode){ pFromNode->addAdjoin(pToNode); }
void                addEdgeBidirectional(TpPtrNode pNodeA, TpPtrNode pNodeB){
                        pNodeA->addAdjoin(pNodeB);
                        pNodeB->addAdjoin(pNodeA);
                    }

const int           sizeNodeInGraph(void){return mNodes.size();}

TpPtrNode           NodeByIndex (const int nNodeIndexInGraph) {return mNodes[nNodeIndexInGraph];}
const TpPtrNode     getNodeByIndex (const int nNodeIndexInGraph)const  {return mNodes[nNodeIndexInGraph];}

// nMaxDepth: -1 means no depth limitation, all node will be visited while 0 means no node will be visited.
template<typename FuncVisit, typename FuncToSkipSpecifiedNode>
void                BreadthFristSearch(TpPtrNode pFistNode, int nMaxDepth, FuncVisit fVisit, FuncToSkipSpecifiedNode fSkip){
    vector<bool> nVecFlagVisited(sizeNodeInGraph(), false);
    widthSearchFirst(nullptr, pFistNode, nMaxDepth, fVisit, fSkip, nVecFlagVisited);
}

template<typename FuncVisit, typename FuncToSkipSpecifiedNode>
void                widthSearchFirst(TpPtrNode pNodeFrom, TpPtrNode pNodeTo, int nMaxDepth, FuncVisit fVisit, FuncToSkipSpecifiedNode fSkip,  
                                     vector<bool>&nVecFlagVisited){
    
    // nMaxDepth: -1 means no depth limitation, all node will be visited while 0 means no node will be visited.
    if(nMaxDepth==0)return;
    
    int nIdxNodeInGraph = pNodeTo->getNodeIndexInGraph();
    if(nVecFlagVisited[nIdxNodeInGraph])
        return;
    
    if(!fSkip(pNodeFrom, pNodeTo)){
        fVisit(pNodeFrom,pNodeTo);
        nVecFlagVisited[nIdxNodeInGraph] = true;
    }
    
    for(int nIdxAdjoin=0,nSzAdjoin=pNodeTo->sizeAdjoinNodes();nIdxAdjoin<nSzAdjoin;++nIdxAdjoin){
        TpPtrNode pNodeNextTo = pNodeTo->getNodeByIndex(nIdxAdjoin);
        widthSearchFirst(pNodeTo,pNodeNextTo, nMaxDepth-1, fVisit, fSkip, nVecFlagVisited);
    }
}

//template<typename EqualFunc>
//bool                QueryNodeByFunc(EqualFunc f);

template<typename EqualFunc>
bool                QueryNodeByFunc(EqualFunc f){
                        for(int nIdxNode=0,nSzNodes= sizeNodeInGraph();nIdxNode<nSzNodes;++nIdxNode){
                            TpPtrNode nPtrNode = NodeByIndex(nIdxNode);
                            auto eResult = f(nPtrNode);
                            if(eResult == EnContinue){
                                continue;
                            }else if(eResult == EnBreak || eResult == EnReturn) {
                                return true;
                            }else if(eResult != EnExec){
                                throw;
                            }
                        }
                        return false;
                    }

private:
TpVecPtrNodes       mNodes;
    
};


template<typename T> 
typename Graph<T>::TpPtrNode    Graph<T>::generateNode(const T& nData)
{ 
    return new Node(nData);
}

template<typename T> 
void                            Graph<T>::clearGraph(void)
{
    for(int nIdxNode =0,nSzNodes = this->sizeNodeInGraph(); nIdxNode<nSzNodes;++nIdxNode){
        TpPtrNode pNode = this->NodeByIndex(nIdxNode);
        pNode->clear();
    }
    this->mNodes.clear();
}

//template<typename T, typename EqualFunc>
//bool                Graph<T>::QueryNodeByFunc<EqualFunc>(EqualFunc f){
//        for(int nIdxNode=0,nSzNodes= this->sizeNodeInGraph();nIdxNode<nSzNodes;++nIdxNode){
//            TpPtrNode nPtrNode = this->getNodeByIndex(nIdxNode);
//            auto eResult = f(nPtrNode);
//            if(eResult == EnContinue){
//                continue;
//            }else if(eResult == EnBreak || eResult == EnReturn) {
//                return true;
//            }else {
//                throw;
//            }
//        }
//        return false;
//}
}
}


#endif

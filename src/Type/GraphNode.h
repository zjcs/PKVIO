#ifndef __GRAPHNODE_H__
#define __GRAPHNODE_H__

#include<vector>

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
    
template<typename T>
class Graph
{
public:
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

    //bool                operator<( T& n){return false;}

    const int           sizeAdjoinNodes(void) const {return (int)mVecAdjoinNodes.size();}
    TpPtrNode           NodeByIndex(const int nAdjoinIndex){return mVecAdjoinNodes[nAdjoinIndex];}
    const TpPtrNode     getNodeByIndex(const int nAdjoinIndex)const{return mVecAdjoinNodes[nAdjoinIndex];}
    T&                  Data(void){return mPtrData;}
    const T&            getData(void)const {return mPtrData;}
    void                addAdjoin(TpPtrNode pAdjoint){mVecAdjoinNodes.push_back(pAdjoint);}

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
void                addEdgeBidirectional(TpPtrNode pNodeA, TpPtrNode pNodeB){ pNodeA->addAdjoin(pNodeB); pNodeB->addAdjoin(pNodeA); }

const int           sizeNodeInGraph(void){return mNodes.size();}

TpPtrNode           NodeByIndex (const int nNodeIndexInGraph) {return mNodes[nNodeIndexInGraph];}
const TpPtrNode     getNodeByIndex (const int nNodeIndexInGraph)const  {return mNodes[nNodeIndexInGraph];}

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

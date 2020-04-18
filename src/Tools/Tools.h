#ifndef __TOOLS_H__
#define __TOOLS_H__

#include "Draw.h"
#include "Compare.h"
#include "Timer.h"

namespace PKVIO{
namespace Tools{
    
using namespace Draw;
using namespace Compare;
using namespace Timer;

void getScreenWindowSize(int& nWidth, int& nHeight);
inline cv::Size getScreenWindowSize(void){ int nWidth,nHeight; Tools::getScreenWindowSize(nWidth, nHeight); return cv::Size(nWidth,nHeight); };

template<typename T>
void filter(vector<T>& nVecInput, const vector<bool>& nFilterTrueSave){
    int nIdxToWrite=0;
    for(int nIdx=0,nSz=nVecInput.size();nIdx<nSz;++nIdx){
        if(nFilterTrueSave[nIdx]){
            if(nIdx==nIdxToWrite){
                ++nIdxToWrite; continue;
            }else{
                nVecInput[nIdxToWrite++] = nVecInput[nIdx];
            }
        }
    }
    auto nIterRemove = nVecInput.begin();
    std::advance(nIterRemove, nIdxToWrite);
    nVecInput.erase(nIterRemove, nVecInput.end());
    
    //for(int nIdx=0,nSz=nVecInput.size();nIdx<nSz;++nIdx){ cout << "nCoVis3:" << nIdx << " - " << nVecInput[nIdx] <<endl; }
}

bool triangulation(const cv::Point2f& Pl2D, const cv::Point2f& Pr2D, const float fx, const float& nBaseline, float& nDepthInLeftView);
bool triangulation(const cv::Point2f& Pl2D, const cv::Point2f& Pr2D, const cv::Matx44f& nPrTPl, cv::Vec3f& Pl3D);


template<typename TpKey, typename TpValue>
map<TpKey, TpValue> buildMap(const vector<TpKey>& vKeys,const vector<TpValue>& vValues)
{
    map<TpKey, TpValue> nMap;   
    for(int nIdx=0,nSz=vKeys.size();nIdx<nSz;++nIdx){
        nMap[vKeys[nIdx]] = vValues[nIdx];
    }
    return nMap;
}

}
}

#endif

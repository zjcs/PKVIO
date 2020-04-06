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
            for(int nIdx=0,nIdxToWrite=0,nSz=nVecInput.size();nIdx<nSz;++nIdx){
                if(nFilterTrueSave[nIdx]){
                    if(nIdx==nIdxToWrite){
                        ++nIdxToWrite; continue;
                    }else{
                        nVecInput[nIdxToWrite++] = nVecInput[nIdx];
                    }
                }
            }
        }
    }
}

#endif

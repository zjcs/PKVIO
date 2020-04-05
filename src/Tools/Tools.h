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
    }
}

#endif

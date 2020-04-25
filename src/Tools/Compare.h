#ifndef __COMPARE_H__
#define __COMPARE_H__

#include <opencv2/opencv.hpp>

namespace PKVIO{
namespace Compare{
    template<typename T>
    inline bool isInRange(const T& t, const T& low, const T& high){return (t>=low) && (t<=high);}
    
    inline bool isInWindow (const cv::Point& p, const cv::Size& nWinSz){
        return isInRange<float>(p.x, 0, nWinSz.width) && isInRange<float>(p.y, 0, nWinSz.height);
    }
    
    inline bool isInWindow (const cv::Point& p, const cv::Rect& nRect){
        return isInRange<float>(p.x, nRect.x, nRect.x+nRect.width) && isInRange<float>(p.y, nRect.y, nRect.y+nRect.height);
    }
    
}
}


#endif

#include "type.h"
#include <stdio.h>

namespace PKVIO{
namespace Type{
    
const TpFrameID     INVALIDFRAMEID      = -1;     
const TpKeyPointID  INVALIDKEYPOINTID   = -1;     

string cvtTimeStampToString(const TpTimeStamp& t){
    char a[100];
    sprintf(a,"%19.0f",t);
    return string(a);
}
    
TpMatchPair cvtMatchToMatchPair(const cv::DMatch& m){
    return std::make_pair(m.queryIdx, m.trainIdx);
}


}
}

#include "type.h"
#include <stdio.h>

namespace PKVIO{
namespace Type{
    
const TpFrameID     INVALIDFRAMEID      = -1;     
const TpKeyPointID  INVALIDKEYPOINTID   = -1;     
const TpMapPointID  INVALIDMAPPOINTID   = -1;


namespace TypeConvertor{
    
string cvtTimeStampToString(const TpTimeStamp& t){
    char a[100];
    sprintf(a,"%19.0f",t);
    return string(a);
}
    
TpMatchPair cvtMatchToMatchPair(const cv::DMatch& m){
    return std::make_pair(m.queryIdx, m.trainIdx);
}
}

void cvtMatx44fToR33T31(const cv::Matx44f&T, cv::Matx33f& R, cv::Matx31f&t)
{
    R = cv::Mat(T)(cv::Rect(0,0,3,3));
    t = cv::Mat(T)(cv::Rect(3,0,1,3));
}

void cvtMatx44fToPosition(const cv::Matx44f&T, cv::Vec3f& p)
{
    p = cvtMatx44fToPosition(T);
}

cv::Vec3f cvtMatx44fToPosition(const cv::Matx44f&T)
{
    cv::Mat R = cv::Mat(T)(cv::Rect(0,0,3,3));
    cv::Mat t = cv::Mat(T)(cv::Rect(3,0,1,3));
    cv::Matx31f mp = (cv::Mat(-R.t()*t));
    return cv::Vec3f(mp(0), mp(1), mp(2));
}

}
}

#ifndef __TYPE__
#define __TYPE__
#include <string>
#include <queue>
#include <list>
#include <functional>
#include <map>
#include <opencv/cv.h>
#include "Frame.h"
#include "IDGenerator.h"
#include "GraphNode.h"
#include "FixLengthQueue.h"
#include "DataHistoryTemplate.h"
#include "KeyFrameNonKeyFrame.h"

using namespace std;

namespace PKVIO{
namespace Type
{

using namespace PKVIO::GraphNode;
using namespace PKVIO::IDGenerator;
   

typedef     std::vector<cv::KeyPoint>   TpVecKeyPoints;
typedef     cv::Mat                     TpVecDescriptor;

typedef cv::DMatch                      TpOneMatchResult;
typedef vector<cv::DMatch>              TpVecMatchResult;
typedef pair<int,int>                   TpMatchPair;
typedef vector<TpMatchPair>             TpVecMatchPairs;


typedef enum{
    EnNonKF,
    EnNeedKF,
    EnLost,
    EnLoop,
    EnReloc
}EnSLAMState;


namespace TypeConvertor
{
    
template<typename T> string             toString(T& t)
{
    stringstream sStrStream;
    sStrStream<<t;
    return sStrStream.str();
}
string                                  cvtTimeStampToString(const TpTimeStamp& t);
TpMatchPair                             cvtMatchToMatchPair(const cv::DMatch & m);

}

using namespace TypeConvertor;


}

using namespace Type;

}

#endif

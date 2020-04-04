#ifndef __TYPE__
#define __TYPE__
#include <string>
#include <opencv/cv.h>
#include <queue>
#include <list>
#include <functional>
#include <map>
#include "Frame.h"
#include "IDGenerator.h"
#include "GraphNode.h"
#include "FixLengthQueue.h"
#include "DataHistoryTemplate.h"

using namespace std;

namespace PKVIO{
namespace Type
{

using namespace PKVIO::GraphNode;
using namespace PKVIO::IDGenerator;
   

typedef     std::vector<cv::KeyPoint>   TpVecKeyPoints;
typedef     cv::Mat                     TpVecDescriptor;

typedef vector<cv::DMatch>              TpVecMatchResult;
typedef pair<int,int>                   TpMatchPair;
typedef vector<TpMatchPair>             TpVecMatchPairs;

TpMatchPair                             cvtMatchToMatchPair(const cv::DMatch& m);

string                                  cvtTimeStampToString(const TpTimeStamp& t);

}

using namespace Type;

}

#endif

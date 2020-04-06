#include "DebugManager.h"
#include <iostream>

using namespace std;

namespace PKVIO
{
namespace DebugManager 
{
    
void logDebugMatchInfo(const DebugMatchInfo& nDebugMatchInfo)
{
    nDebugMatchInfo.log();
}

void logDebugKeyFrameGenerationInfo(const DebugKeyFrameGenerationInfo& nDebugKeyFrameGenerationInfo)
{
    nDebugKeyFrameGenerationInfo.log();
}

DebugInfoConfig& getDebugInfoConfig() 
{
    static DebugInfoConfig nDebugInfoConfig;
    return nDebugInfoConfig;
}

}
}

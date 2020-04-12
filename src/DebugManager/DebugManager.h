#ifndef __DEBUGMANAGER_H__
#define __DEBUGMANAGER_H__

#include "DebugInfoConfig.h"

namespace PKVIO
{
namespace DebugManager 
{
    
DebugInfoConfig&    getDebugInfoConfig(void);
void                logDebugMatchInfo(const DebugMatchInfo& nDebugMatchInfo);
void                logDebugKeyFrameGenerationInfo(const DebugKeyFrameGenerationInfo& nDebugKeyFrameGenerationInfo);

int                 getMinimumKptNumberToKeepTrackingWell(void);
int                 getMinimumKptNumberToCreateKFOtherwiseLost(void);

std::vector<cv::Vec3d> getMapPointUsedInSimulator(void);


class DebugManager 
{
    
};
}
}

#endif

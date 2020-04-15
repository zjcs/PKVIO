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

std::vector<cv::Vec3d> getVirtualPointInSimulator(void);

int                 getMaxCoVisLength(void);

void                setUseSimulator(bool bUseSimulator);
bool                getUseSimulator(void);


class TpDebugControl{
public:
    int  mCountSegment              = 20;
    int  mCountCoVis                = 1;
    bool mBoolUseSimulator          = true;
    bool mBoolUseG2OSolver          = true;
    bool mBoolUseCoVisMgr           = false;
    bool mBoolMapPointFixed         = false;    // fix mappoint to not optimizate.
    bool mBoolUpdateMapPoint        = true;     // synchronous the mappoint optimization result while mappoint not fixed.
    const std::string str(void)const{
        stringstream s;
        s << endl << "Debug Control: " <<endl
        << " UseSimulator       : " << mBoolUseSimulator << endl
        << " CountSegment       : " << mCountSegment <<endl
        << " UseG2OSolver       : " << mBoolUseG2OSolver << endl
        << " UsePnPSolver       : " << !mBoolUseCoVisMgr << endl
        << " MapPointFixed      : " << mBoolMapPointFixed << endl <<endl;
        return s.str();
    }
    
    std::vector<cv::Vec3d> getVirtualPointInSimulator(void){return DebugManager::getVirtualPointInSimulator();}
};

TpDebugControl& DebugControl(void);


class DebugManager 
{
    
};
}
}

#endif

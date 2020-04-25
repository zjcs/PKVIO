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

class TpDebugControl{
public:
    int  mCountSegment              = 20;
    
    int  mCountCoVis                = 2;        // 1, covis depth 1, means only self camera pose, no other covis frame, to be optimizated.
                                                // 2, covis depth 1, means self and directly co-vis(track) frames camera pose will be optimizated.
    int  mCountMaxCoVisFrame        = 0; //100;     // only the nearest frame camera pose will be optimizated, 0 means no limitation.
    
    bool mBoolUseSimulator          = false;
    bool mBoolTrackByOpticalFlow    = true;
    
    bool mBoolUseG2OSolver          = true;
    bool mBoolUseCoVisMgr           = true;
    
    bool mBoolMapPointFixed         = false;    // fix mappoint to not optimizate.
    bool mBoolUpdateMapPoint        = true;     // synchronous the mappoint optimization result while mappoint not fixed.
    
    int  mMaxFramesToMatchInTrack   = 1;        // 1 means only previous and current frame.
    int  mMaxKeyFramesToMatchInTrack= 0;        // 1 means will track from the nearest keyframe. 
                                                // too many wrong match, stop.
    
    int  mMaxGraphDepthToBuildCoVis = 2;        // 1 or 2 means only self and track frames relationship;
    
    bool mBoolShowMatchResult       = false;
    bool mBoolShowMatchResultWaitKey= false;
    
    bool mBoolLogSolverVtxEdgInfo   = false;
    bool mBoolLogStereoTriangular   = false;
    
    bool mBoolTriangularByInverseDepth = false;
    bool mBoolAddStereoMatchInSolver   = true;
    
    bool mBoolSolverUseOnlyCoVisKpt    = false;
    
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

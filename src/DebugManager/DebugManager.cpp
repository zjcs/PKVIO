#include "DebugManager.h"
#include <iostream>

using namespace std;

namespace PKVIO
{
namespace DebugManager 
{
    
static bool gBoolUseSimulator = false;
static TpDebugControl gDebugControl;
    
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

int getMinimumKptNumberToKeepTrackingWell(void){
    if(getUseSimulator())
        return 1;
    return 30;
}

int getMinimumKptNumberToCreateKFOtherwiseLost(void){
    if(getUseSimulator())
        return getVirtualPointInSimulator().size();
    return 50;
}

std::vector<cv::Vec3d> getVirtualPointInSimulator(void){
    typedef cv::Vec3d TpDataPt;
    int nZ1 = 100;
    int nZ2 = 200;
    std::vector<cv::Vec3d> nVecMapPt3D = {
        TpDataPt(0,0,nZ1), TpDataPt(100,0,nZ1) , TpDataPt(0,100,nZ1) 
        , TpDataPt(-50,-50,nZ1) ,TpDataPt(50,-50,nZ1),TpDataPt(50,50,nZ1),TpDataPt(-50,50,nZ1)
        , TpDataPt(20,20,nZ2),TpDataPt(80,20,nZ2),TpDataPt(80,80,nZ2),TpDataPt(20,80,nZ2),
    };
    return nVecMapPt3D;
}

int getMaxCoVisLength(void){
    return 10;
}

void setUseSimulator(bool bUseSimulator){
    gBoolUseSimulator = bUseSimulator;
}
bool getUseSimulator(void){
    return gBoolUseSimulator;
}

TpDebugControl& DebugControl(void){
    return gDebugControl;
}

}
}

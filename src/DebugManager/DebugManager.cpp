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

int getMinimumKptNumberToKeepTrackingWell(void){
    return 1;
    // return 30;
}

int getMinimumKptNumberToCreateKFOtherwiseLost(void){
    //return 50;
    return 0;
}

std::vector<cv::Vec3d> getMapPointUsedInSimulator(void){
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

}
}

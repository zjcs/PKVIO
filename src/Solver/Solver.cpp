#include "Solver.h"


namespace PKVIO {
namespace Solver {
Solver::Solver() 
{
    initialization();
}

void Solver::initialization() 
{
    
}

void Solver::initCamerPoses(const std::map< PKVIO::Type::TpFrameID, TpPtrCameraPose>& nMapFrameID2CameraPose) 
{
    
}

void Solver::initMapPoints(const std::map< PKVIO::Type::TpMapPointID, PKVIO::Type::TpPtrMapPoint3D >& nMapMapPointID2MapPoint3D) 
{
    
}

void Solver::addMeasurement(const TpVisualMeasurement& nVisualMeasurement) 
{
    
}

void Solver::solve(std::map< Type::TpFrameID, TpPtrCameraPose>& nMapFrameID2CameraPose,
                   std::map<Type::TpMapPointID, Type::TpPtrMapPoint3D >& nMapMapPointID2MapPoint3D) 
{
    
}

}
}


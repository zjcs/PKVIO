#ifndef __SOLVER_H__    
#define __SOLVER_H__

#include "../Type/type.h"

namespace PKVIO
{
namespace Solver
{
    class TpVisualMeasurement{
    public:
        TpFrameID           mFrameID;
        TpMapPointID        mMapPointID;
        
        TpKeyPoint          mKeyPoint;
        TpPtrMapPoint3D     mMapPoint3D;
        
        bool                mBoolRight;
    };
    
    typedef vector<TpVisualMeasurement> TpVecVisualMeasurement;
    
    class Solver {
    public:
        Solver();
        void initCamerPoses(const map<TpFrameID, TpPtrCameraPose>& nMapFrameID2CameraPose);
        void initMapPoints(const map<TpMapPointID, TpPtrMapPoint3D>& nMapMapPointID2MapPoint3D);
        void addMeasurement(const TpVisualMeasurement& nVisualMeasurement);
        void solve(map<TpFrameID, TpPtrCameraPose>& nMapFrameID2CameraPose, map<TpMapPointID, TpPtrMapPoint3D>& nMapMapPointID2MapPoint3D, const TpVecVisualMeasurement& nVecVisualMeasurement, const TpPtrCameraStereo nPtrCameraStereo);
        
        
        void solveBySimulator(cv::Mat nInnerMat, TpPtrCameraStereo nPtrCameraStereo);
    protected:
        void initialization(void);
    };
}
}

#endif

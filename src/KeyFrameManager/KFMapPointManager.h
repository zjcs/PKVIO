#ifndef __KFMAPPOINTMANAGER_H__
#define __KFMAPPOINTMANAGER_H__

#include <opencv2/core/types.hpp>
#include "../Type/type.h"

namespace PKVIO
{
namespace KeyFrameManager
{
    
    class TpMapPoint
    {
    public:
        TpMapPoint(TpKeyPointID nKptID):mKptID(nKptID){}
        inline void             addMeasurement(TpFrameID nFrameID, TpKeyPointIndex nKptIndex){ mVecMeasurements.push_back(std::make_pair(nFrameID, nKptIndex)); }
        inline TpKeyPointID     getKeyPointID(void){return mKptID;}
    private:
        cv::Point3f     mMapPoint3D;
        TpKeyPointID    mKptID;
        vector<pair<TpFrameID, TpKeyPointIndex>>    mVecMeasurements;
    };
    
}
}

#endif

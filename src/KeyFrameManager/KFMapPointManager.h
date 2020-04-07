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
        TpMapPoint(TpKeyPointID nKptID):mKptID(nKptID), mMapPoint3D(std::make_shared<TpMapPoint3D>())
        {}
        
        inline void             addMeasurement(TpFrameID nFrameID, TpKeyPointIndex nKptIndex){
                                    mVecMeasurements.push_back(std::make_pair(nFrameID, nKptIndex)); 
                                }
                                
        inline TpKeyPointID     getKeyPointID(void){return mKptID;}
        
        TpPtrMapPoint3D         getMapPoint3D(void){return mMapPoint3D;}
    private:
        TpPtrMapPoint3D         mMapPoint3D;
        TpKeyPointID            mKptID;
        vector<pair<TpFrameID, TpKeyPointIndex>>    mVecMeasurements;
    };
    
}
}

#endif

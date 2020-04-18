#ifndef __KFMAPPOINTMANAGER_H__
#define __KFMAPPOINTMANAGER_H__

#include <opencv2/core/types.hpp>
#include "../Type/type.h"

namespace PKVIO
{
namespace KeyFrameManager
{
    float RandomData();
    class TpMapPoint
    {
    public:
        TpMapPoint(TpMapPointID nMapPointID, TpKeyPointID nKptID)
        : mMapPointID(nMapPointID), mKptID(nKptID), mBoolMpValid(false)
        , mMapPoint3D(std::make_shared<TpMapPoint3D>(cv::Point3f(RandomData(),RandomData(),RandomData())))
        {
            //cout << "New MpPt: KptID|MpID-" << nKptID<<"|"<<nMapPointID<<endl;
        }
        
        inline void             initMapPoint3D(const TpMapPoint3D& nMp3D){*mMapPoint3D = nMp3D; mBoolMpValid = true;}
        inline bool             getMapPoint3DValid(void)const{return mBoolMpValid;}
        
        inline void             addMeasurement(TpFrameID nFrameID, TpKeyPointIndex nKptIndex){
                                    mVecMeasurements.push_back(std::make_pair(nFrameID, nKptIndex)); 
                                }
                                
        inline TpKeyPointID     getKeyPointID(void)const{return mKptID;}
        inline TpMapPointID     getMapPointID(void)const{return mMapPointID;}
        inline TpPtrMapPoint3D  getMapPoint3D(void)const{return mMapPoint3D;}
        
        vector<pair<TpFrameID, TpKeyPointIndex>>&       VecMeasurments(void){return mVecMeasurements;}
        const vector<pair<TpFrameID, TpKeyPointIndex>>& getVecMeasurments(void)const{return mVecMeasurements;}
        
    private:
        bool                    mBoolMpValid;
        TpPtrMapPoint3D         mMapPoint3D;
        TpMapPointID            mMapPointID;
        TpKeyPointID            mKptID;
        vector<pair<TpFrameID, TpKeyPointIndex>>    mVecMeasurements;
    };
    
}
}

#endif

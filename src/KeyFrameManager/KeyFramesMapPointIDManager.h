

#ifndef __KEYFRAMESMAPPOINTIDMANAGER_H__
#define __KEYFRAMESMAPPOINTIDMANAGER_H__

#include <vector>
#include <map>
#include "../Type/type.h"

using namespace std;

namespace PKVIO
{
namespace KeyFrameManager
{
/*
class TpOneKeyFrameMapPointIDManager
{
public:
    TpOneKeyFrameMapPointIDManager(int nCountMapPoints = 0):mVecMapPointIDs(nCountMapPoints, INVALIDKEYPOINTID){}
    TpOneKeyFrameMapPointIDManager(const TpOneKeyFrameMapPointIDManager& cp):mVecMapPointIDs(cp.mVecMapPointIDs){}
    
    inline TpMapPointID&                MapPointID(const TpKeyPointID nKeyPointID){ return mVecKeyPointIDs[nKeyPointIndexInThisFrame]; }
    inline const TpMapPointID           getMapPointID(const TpKeyPointIndex nKeyPointIndexInThisFrame) const { return mVecKeyPointIDs[nKeyPointIndexInThisFrame]; }
    
    inline void                         InitializeMapPointID(TpKeyPointID& nNewKptIDToThisID, TpKeyPointID nNewKptID){ nNewKptIDToThisID = nNewKptID; }
    inline void                         InitializeKptIDByKptIdx(TpKeyPointIndex& nNewKptIDToThisIndex, TpKeyPointID nNewKptID){ KeyPointID(nNewKptIDToThisIndex) = nNewKptID; }
    
    inline int                          sizeKeyPoints(void){return mVecKeyPointIDs.size();}
    inline int                          sizeKeyPointsWithID(void){ 
        return sizeKeyPoints() - std::count(mVecKeyPointIDs.begin(),mVecKeyPointIDs.end(),INVALIDKEYPOINTID); }
    
    TpVecKeyPointID                     getAllKeyPointIDs(void){
        int nSzKptIDs = sizeKeyPointsWithID();
        TpVecKeyPointID nVecKptIDs;
        nVecKptIDs.resize(nSzKptIDs);
        std::copy_if(mVecKeyPointIDs.begin(),mVecKeyPointIDs.end(), nVecKptIDs.begin(),Type::isInvalideKeyPointID);
        return nVecKptIDs;
    }
    
private:
    map<TpMapPointID, TpKeyPointID>     mMapPointID2KeyPointID;
};
    
class KeyFramesMapPointIDManager
{
public:
    inline TpOneKeyFrameMapPointIDManager&         OneKeyFrameMapPointIDManager(const TpFrameID nFrameID){return OneKeyFrameMapPointIDManagerByFrameIndex(getFrameIndex(nFrameID));}
    inline const TpOneKeyFrameMapPointIDManager&   getOneKeyFrameMapPointIDManager(const TpFrameID nFrameID)const{return getOneKeyFrameMapPointIDManagerByFrameIndex(getFrameIndex(nFrameID));}
    inline TpOneKeyFrameMapPointIDManager&         OneKeyFrameMapPointIDManagerByFrameIndex(const TpFrameIndex nFrameIndex){ return mSetFrameIndex2OneKeyFrameMapPointIDManager[nFrameIndex]; }
    inline const TpOneKeyFrameMapPointIDManager&   getOneKeyFrameMapPointIDManagerByFrameIndex(const TpFrameIndex nFrameIndex)const{ return mSetFrameIndex2OneKeyFrameMapPointIDManager[nFrameIndex]; }
    
    inline TpKeyPointID&                KeyPointID(const TpFrameID nFrameID, const TpKeyPointIndex nKeyPointIndexInThisFrame){ return OneKeyFrameMapPointIDManager(nFrameID).KeyPointID(nKeyPointIndexInThisFrame); }
    inline const TpKeyPointID           getKeyPointID(const TpFrameID nFrameID, const TpKeyPointIndex nKeyPointIndexInThisFrame) const 
    {return getOneKeyFrameMapPointIDManager(nFrameID).getKeyPointID(nKeyPointIndexInThisFrame); 
    }
    inline TpKeyPointID&                KeyPointIDByFrameIndex(const TpFrameIndex nFrameIndex, const TpKeyPointIndex nKeyPointIndexInThisFrame){ return OneKeyFrameMapPointIDManagerByFrameIndex(nFrameIndex).KeyPointID(nKeyPointIndexInThisFrame); }
    inline const TpKeyPointID           getKeyPointIDByFrameIndex(const TpFrameIndex nFrameIndex, const TpKeyPointIndex nKeyPointIndexInThisFrame) const 
    {return getOneKeyFrameMapPointIDManagerByFrameIndex(nFrameIndex).getKeyPointID(nKeyPointIndexInThisFrame); 
    }
    
    inline const TpFrameIndex           getFrameIndex(const TpFrameID nFrameID)const{
        auto FindIter = mSetFrameID2FrameIndex.find(nFrameID);
        if(FindIter == mSetFrameID2FrameIndex.end())
            throw;
        return FindIter->second;
    }
    
    void                                addOneKeyFrameMapPointIDManager(const TpFrameIndex nFrameIndex, const TpFrameID nFrameID, const int nCountKpts);
    
    inline TpKeyPointID                 GenerateKeyPointID(void){ return mKeyPointIDGeneration.create();}
private:
    Type::IDGenerator<TpKeyPointID>     mKeyPointIDGeneration;
    
    TpMapFrameID2FrameIndex             mSetFrameID2FrameIndex;
    vector<TpFrameID>                   mSetFrameIndex2FrameID;
    
    vector<TpOneKeyFrameMapPointIDManager>         mSetFrameIndex2OneKeyFrameMapPointIDManager;
    
    
};
*/
}
}

#endif

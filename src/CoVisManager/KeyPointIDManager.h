#ifndef __KEYPOINTIDMANAGER_H__
#define __KEYPOINTIDMANAGER_H__

#include <vector>
#include <map>
#include "../Type/type.h"

using namespace std;

namespace PKVIO
{
namespace KeyPointManager
{

class TpOneFrameIDManager
{
public:
    TpOneFrameIDManager(int nCountKpts = 0):mVecKeyPointIDs(nCountKpts, -1){}
    TpOneFrameIDManager(const TpOneFrameIDManager& cp):mVecKeyPointIDs(cp.mVecKeyPointIDs){}
    
    inline TpKeyPointID&                KeyPointID(const TpKeyPointIndex nKeyPointIndexInThisFrame){ return mVecKeyPointIDs[nKeyPointIndexInThisFrame]; }
    inline const TpKeyPointID           getKeyPointID(const TpKeyPointIndex nKeyPointIndexInThisFrame) const { return mVecKeyPointIDs[nKeyPointIndexInThisFrame]; }
    
    inline void                         InitializeKptID(TpKeyPointID& nNewKptIDToThisID, TpKeyPointID nNewKptID){ nNewKptIDToThisID = nNewKptID; }
    inline void                         InitializeKptIDByKptIdx(TpKeyPointIndex& nNewKptIDToThisIndex, TpKeyPointID nNewKptID){ KeyPointID(nNewKptIDToThisIndex) = nNewKptID; }
private:
    vector<TpKeyPointID>                mVecKeyPointIDs;
};
    
class KeyPointIDManager
{
public:
    inline TpOneFrameIDManager&         OneFrameIDManager(const TpFrameID nFrameID){return OneFrameIDManagerByFrameIndex(getFrameIndex(nFrameID));}
    inline const TpOneFrameIDManager&   getOneFrameIDManager(const TpFrameID nFrameID)const{return getOneFrameIDManagerByFrameIndex(getFrameIndex(nFrameID));}
    inline TpOneFrameIDManager&         OneFrameIDManagerByFrameIndex(const TpFrameIndex nFrameIndex){ return mSetFrameIndex2OneFrameIDManager[nFrameIndex]; }
    inline const TpOneFrameIDManager&   getOneFrameIDManagerByFrameIndex(const TpFrameIndex nFrameIndex)const{ return mSetFrameIndex2OneFrameIDManager[nFrameIndex]; }
    
    inline TpKeyPointID&                KeyPointID(const TpFrameID nFrameID, const TpKeyPointIndex nKeyPointIndexInThisFrame){ return OneFrameIDManager(nFrameID).KeyPointID(nKeyPointIndexInThisFrame); }
    inline const TpKeyPointID           getKeyPointID(const TpFrameID nFrameID, const TpKeyPointIndex nKeyPointIndexInThisFrame) const 
    {return getOneFrameIDManager(nFrameID).getKeyPointID(nKeyPointIndexInThisFrame); 
    }
    inline TpKeyPointID&                KeyPointIDByFrameIndex(const TpFrameIndex nFrameIndex, const TpKeyPointIndex nKeyPointIndexInThisFrame){ return OneFrameIDManagerByFrameIndex(nFrameIndex).KeyPointID(nKeyPointIndexInThisFrame); }
    inline const TpKeyPointID           getKeyPointIDByFrameIndex(const TpFrameIndex nFrameIndex, const TpKeyPointIndex nKeyPointIndexInThisFrame) const 
    {return getOneFrameIDManagerByFrameIndex(nFrameIndex).getKeyPointID(nKeyPointIndexInThisFrame); 
    }
    
    inline const TpFrameIndex           getFrameIndex(const TpFrameID nFrameID)const{
        auto FindIter = mSetFrameID2FrameIndex.find(nFrameID);
        if(FindIter == mSetFrameID2FrameIndex.end())
            throw;
        return FindIter->second;
    }
    
    inline void                         add(const TpFrameIndex nFrameIndex, const TpFrameID nFrameID, const int nCountKpts){
        
        // allocate memory.
        mSetFrameIndex2FrameID.resize(nFrameIndex+1);
        mSetFrameIndex2OneFrameIDManager.resize(nFrameIndex+1);
        
        // initialize data.
        mSetFrameID2FrameIndex[nFrameID] = nFrameIndex;
        mSetFrameIndex2FrameID[nFrameIndex] = nFrameID;
        mSetFrameIndex2OneFrameIDManager[nFrameIndex] = TpOneFrameIDManager(nCountKpts);
    }
    
    
    
    inline TpKeyPointID                 GenerateKeyPointID(void){ return mKeyPointIDGeneration.create();}
private:
    Type::IDGenerator<TpKeyPointID> mKeyPointIDGeneration;
    
    TpMapFrameID2FrameIndex mSetFrameID2FrameIndex;
    vector<TpFrameID> mSetFrameIndex2FrameID;
    
    vector<TpOneFrameIDManager> mSetFrameIndex2OneFrameIDManager;
    
    
};

}
}

#endif

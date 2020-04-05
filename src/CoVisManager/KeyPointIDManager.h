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
    TpOneFrameIDManager(int nCountKpts = 0):mVecKeyPointIDs(nCountKpts, INVALIDKEYPOINTID){}
    TpOneFrameIDManager(const TpOneFrameIDManager& cp):mVecKeyPointIDs(cp.mVecKeyPointIDs){}
    
    inline TpKeyPointID&                KeyPointID(const TpKeyPointIndex nKeyPointIndexInThisFrame){ return mVecKeyPointIDs[nKeyPointIndexInThisFrame]; }
    inline const TpKeyPointID           getKeyPointID(const TpKeyPointIndex nKeyPointIndexInThisFrame) const { return mVecKeyPointIDs[nKeyPointIndexInThisFrame]; }
    
    inline void                         InitializeKptID(TpKeyPointID& nNewKptIDToThisID, TpKeyPointID nNewKptID){ nNewKptIDToThisID = nNewKptID; }
    inline void                         InitializeKptIDByKptIdx(TpKeyPointIndex& nNewKptIDToThisIndex, TpKeyPointID nNewKptID){ KeyPointID(nNewKptIDToThisIndex) = nNewKptID; }
    
    inline void                         setKptIDIsFirstDetectedDueToCurrentFrame(TpKeyPointIndex& nKptIndexInThisFrame, TpKeyPointID nKptID){
        mLstFirstDetectedKptIndex.push_back(nKptIndexInThisFrame);
    }
    
    inline TpVecKeyPointID              getFirstDetectedKptIDs(void){
        TpVecKeyPointID nVecKptID;
        nVecKptID.reserve(sizeFirstKptIDsDetected());
        for(auto Iter = mLstFirstDetectedKptIndex.begin(),EndIter = mLstFirstDetectedKptIndex.end();Iter!=EndIter;++Iter){
            nVecKptID.push_back(mVecKeyPointIDs[*Iter]);
        }
        return nVecKptID;
    }
    inline void                         getFirstDetectedKptIDsAndIdexs(TpVecKeyPointID& nVecKptIDs, TpVecKeyPointIndex& nVecKptIndexs){
        // copy Index;
        nVecKptIndexs = TpVecKeyPointIndex(mLstFirstDetectedKptIndex.begin(), mLstFirstDetectedKptIndex.end());
        // copy ID;
        nVecKptIDs = getFirstDetectedKptIDs();
    }
    
    inline int                          sizeKeyPoints(void){return mVecKeyPointIDs.size();}
    inline int                          sizeKeyPointsWithID(void){ 
        return sizeKeyPoints() - std::count(mVecKeyPointIDs.begin(),mVecKeyPointIDs.end(),INVALIDKEYPOINTID); }
    inline int                          sizeFirstKptIDsDetected(void){return mLstFirstDetectedKptIndex.size();}
    
    TpVecKeyPointID                     getAllKeyPointIDs(void){
        int nSzKptIDs = sizeKeyPointsWithID();
        TpVecKeyPointID nVecKptIDs;
        nVecKptIDs.resize(nSzKptIDs);
        std::copy_if(mVecKeyPointIDs.begin(),mVecKeyPointIDs.end(), nVecKptIDs.begin(),Type::isValideKeyPointID);
        return nVecKptIDs;
    }
    TpVecKeyPointIndex                   getAllKeyPointIndexsWithID(void){
        int nSzKptIDs = sizeKeyPointsWithID();
        TpVecKeyPointIndex nVecKptIndexs;
        nVecKptIndexs.reserve(nSzKptIDs);
        for(int nKptIdx = 0,nSzKpts = (int)mVecKeyPointIDs.size();nKptIdx!=nSzKpts;++nKptIdx){
            if(Type::isValideKeyPointID(mVecKeyPointIDs[nKptIdx])){
                nVecKptIndexs.push_back(nKptIdx);
            }
        }
        return nVecKptIndexs;
    }
    
    string                              str(void){
        stringstream sStrStream;
        sStrStream << "L+R Kpts | KptIDs | NewKptIDs - "<<   sizeKeyPoints() << " | " << sizeKeyPointsWithID() << " | " << sizeFirstKptIDsDetected();
        return sStrStream.str();
    }
private:
    TpVecKeyPointID                     mVecKeyPointIDs;
    list<TpKeyPointIndex>               mLstFirstDetectedKptIndex;
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
    
    void                                addOneFrameIDManager(const TpFrameIndex nFrameIndex, const TpFrameID nFrameID, const int nCountKpts);
    
    inline TpKeyPointID                 GenerateKeyPointID(void){ return mKeyPointIDGeneration.create();}
private:
    Type::IDGenerator<TpKeyPointID>     mKeyPointIDGeneration;
    
    TpMapFrameID2FrameIndex             mSetFrameID2FrameIndex;
    vector<TpFrameID>                   mSetFrameIndex2FrameID;
    
    vector<TpOneFrameIDManager>         mSetFrameIndex2OneFrameIDManager;
    
    
};

}
}

#endif

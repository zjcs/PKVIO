#ifndef __DEBUGINFOCONFIG_H__
#define __DEBUGINFOCONFIG_H__

#include "../Type/type.h"
#include <iostream>

using namespace std;

namespace PKVIO
{
namespace DebugManager 
{
    
#define STR5BEGIN "     "
#define STR4BEGIN "    "
#define STR0l0SEP  "|"
#define STR1l1SEP " | "
    
    
class DebugInfoInterface{
public:
    virtual ~DebugInfoInterface(){}
    virtual const string str() const {return "";}
    virtual bool         isEnableLog(void) const    {return true;}
    inline void          log(void) const            {log(isEnableLog());}
    inline void          log(bool bLog) const       {if(bLog)cout << str() <<endl;}
    inline void          logForce(void) const       {log(true);}
};   

class DebugMatchInfo:public DebugInfoInterface
{
public:
    TpFrameID   mFrameIDMaster;
    TpFrameID   mFrameIDSlaver;
    int         mCountKptsLeft;
    int         mCountKptsRight;
    TpTimeStamp mTimeCost;
    int         mCountKptsMatch;
    int         nCountMasterKptsWithM1vSn;
    int         nCountSlaverKptsWithM1vSn;
    virtual const string str() const override {
        stringstream sStrStream;    
        const int mCountKptsUnduplicate = mCountKptsLeft+mCountKptsRight-mCountKptsMatch;
        sStrStream  <<"MatchInfo - FrameID-M|S|MatchKpt|AllKpts|LKpt|RKpt|M1vSn|TimeCost:"<<endl
                    <<STR5BEGIN
                    <<mFrameIDMaster<<STR0l0SEP<<mFrameIDSlaver<<STR0l0SEP
                    <<mCountKptsMatch<<STR0l0SEP<<mCountKptsUnduplicate<<STR0l0SEP
                    <<mCountKptsLeft<<STR0l0SEP<<mCountKptsRight<<STR0l0SEP<<nCountMasterKptsWithM1vSn<<"v"<<nCountSlaverKptsWithM1vSn<<STR0l0SEP
                    <<mTimeCost<<"ms"<<endl;
        return sStrStream.str();
    }
    //operator<<(){}
private:
    //int         mCountKptsUnduplicate;
};

class DebugKeyPointTrackingInfo:public DebugInfoInterface
{
public:
    TpFrameID       mFrameID;
    TpTimeStamp     mTimeCostWhole;
    
    virtual const string str() const override {
        stringstream sStrStream;    
        sStrStream  <<"KeyPointTrack - FrameID | TimeCost:"<<endl
                    << STR5BEGIN << mFrameID << STR1l1SEP << mTimeCostWhole <<endl;
        return sStrStream.str();
    } 
};

class DebugCoVisInfo:public DebugInfoInterface
{
public:
    TpFrameID       mFrameID;
    TpTimeStamp     mTimeCostWhole;
    
    virtual const string str() const override {
        stringstream sStrStream;    
        sStrStream  <<"CoVisInfo - FrameID | TimeCost:"<<endl
                    << STR5BEGIN << mFrameID << STR1l1SEP << mTimeCostWhole <<endl;
        return sStrStream.str();
    } 
};


class DebugKeyFrameGenerationInfo:public DebugInfoInterface
{
public:
    TpFrameID               mFrameID;
    TpFrameID               mLastKeyFrameID;
    TpTimeStamp             mTimeCostWhole;
    TpTimeStamp             mTimeCostCountTrackedMapPoint;
    TpTimeStamp             mTimeCostAccFirstDetectedKptIDs;
    TpTimeStamp             mTimeCostGenerateMapPoint;
    TpTimeStamp             mTimeCostAddMeasurement;
    string                  mStrKeyFrameGeneration;
    
    int                     nCountTrackedMapPoint;
    int                     nCountTrackedKptIDs;
    int                     nCountNewKptIDs;
    int                     nCountFirstDetectedKptIDs;
    int                     nCountFirstDetectedKptIDToMapIDs;
    
    virtual const string    str() const override {
        stringstream sStrStream;    
        sStrStream  <<"KeyFrameGenerate - FrameID | AccNewKptID | TrackedKptID | TrackedMapPoint | CreateNewMapPointID "<<endl
                    << mStrKeyFrameGeneration << " LastKeyFrameID: " << mLastKeyFrameID << endl
                    << STR5BEGIN <<mFrameID <<STR1l1SEP<<nCountFirstDetectedKptIDs << STR1l1SEP << nCountTrackedKptIDs << STR1l1SEP << nCountTrackedMapPoint 
                    << STR1l1SEP << nCountFirstDetectedKptIDToMapIDs  <<endl
                    << STR4BEGIN << "TimeCost-Whole|CountTrack|AccNewKptID|Generate|AddMeasurement:"<<endl
                    << STR5BEGIN <<mTimeCostWhole<<STR0l0SEP <<mTimeCostCountTrackedMapPoint <<STR0l0SEP<< mTimeCostAccFirstDetectedKptIDs
                              <<STR0l0SEP << mTimeCostGenerateMapPoint <<STR0l0SEP<<mTimeCostAddMeasurement<<endl;
        return sStrStream.str();
    }
 };
    
class DebugInfoConfig 
{
public:
    bool getEnableOutputMatchInfo(void){return true;}
    
private:
};
}
}

#endif


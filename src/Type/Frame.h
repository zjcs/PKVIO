#ifndef __FRAME_H__
#define __FRAME_H__

#include <map>
#include <set>
#include <memory>
#include "Convertor.h"

using namespace std;

namespace PKVIO
{
namespace Type
{

typedef double  TpTimeStamp;
    
typedef int     TpFrameID;
typedef int     TpFrameIndex;
typedef int     TpKeyFrameID;

typedef int     TpKeyPointID;
typedef int     TpKeyPointIndex;

typedef vector<TpKeyPointID>    TpVecKeyPointID;
typedef set<TpKeyPointID>       TpSetKeyPointID;
typedef list<TpKeyPointID>      TpLstKeyPointID;
typedef vector<TpKeyPointIndex> TpVecKeyPointIndex;
typedef list<TpKeyPointIndex>   TpLstKeyPointIndex;
typedef vector<TpKeyFrameID>    TpVecKeyFrameID;
typedef list<TpKeyFrameID>      TpLstKeyFrameID;

extern const TpFrameID          INVALIDFRAMEID;     
extern const TpKeyPointID       INVALIDKEYPOINTID;     
extern const TpKeyPointIndex    INVALIDKEYPOINTINDEX;     

inline bool                 isInvalideFrameID(const TpFrameID nFrameID){return nFrameID==INVALIDFRAMEID;}
inline bool                 isInvalideKeyPointID(const TpKeyPointID nKptID){return nKptID==INVALIDKEYPOINTID;}
inline bool                 isValideKeyPointID(const TpKeyPointID nKptID){return nKptID!=INVALIDKEYPOINTID;}
inline bool                 isValideKeyPointIndex(const TpKeyPointIndex nKptIndex){return nKptIndex!=INVALIDKEYPOINTINDEX;}

typedef vector<TpFrameID>               TpVecFrameID;
typedef set<TpFrameID>                  TpSetFrameID;
typedef map<TpFrameID, TpFrameIndex>    TpMapFrameID2FrameIndex;
 
typedef enum {
    TpMono			= 0x01<<0,
    TpRight			= 0x01<<1,
    TpDepth     	= 0x01<<2,

    TpStereo		= TpMono | TpRight,
    TpRGBD			= TpMono | TpDepth,
    TpStereoDepth 	= TpStereo | TpDepth
} TpFrame;

class FrameInfo{
public:
    string          mStrFileName;
    string          mStrFileAbsName;
    TpTimeStamp     mTimeStamp;
    TpFrameID       mFrameID;
    TpFrameIndex    mFrameIndex;
};

cv::Vec3f project(const cv::Matx44f& T, const cv::Vec3f& pt);
cv::Vec2f project(const cv::Matx33f& P, const cv::Matx44f& T, const cv::Vec3f& pt);
cv::Vec2f projectError(const cv::Matx33f& P, const cv::Matx44f& T, const cv::Vec3f& pt, const cv::Vec2f& pt2d);

class Frame{
public:
    virtual                     ~Frame(){}
    virtual TpFrame             type(void) const {return TpMono;}
    inline const TpFrameID&     FrameID(void)const { return mFrameID; }
    inline void                 initFrameID(const TpFrameID& nFrmID){ mFrameID = nFrmID; };
    inline void                 initFrameIndex(const TpFrameIndex& nFrmIndex){ mFrameIndex = nFrmIndex; };
    inline cv::Mat&             Image(void) { return mImage; }
    inline const cv::Mat&       getImage(void) const {return mImage; }
    inline TpFrameIndex&        FrameIndex(void){ return mFrameIndex; }
    inline const TpFrameIndex   getFrameIndex(void)const{ return mFrameIndex; }
protected:
    cv::Mat                     mImage;
    TpFrameID                   mFrameID;
    TpFrameIndex                mFrameIndex;
};

class StereoFrame: public Frame{
public:
    virtual TpFrame             type(void) const override {return TpStereo;}
    inline cv::Mat&             ImageLeft(void){ return Image(); }
    inline cv::Mat&             ImageRight(void){ return mImageRight; }
    inline const cv::Mat&       getImageLeft(void) const {return getImage(); }
    inline const cv::Mat&       getImageRight(void) const {return mImageRight; }
protected:
    cv::Mat                     mImageRight;
};

class TpCameraPose
{
public:
    TpCameraPose(const cv::Matx44f& nT){setMatx44f(nT);}
    TpCameraPose(const TpCameraPose& nT){setMatx44f(nT.getMatx44f());}
    
    void                setMatx44f(const cv::Matx44f& nT){mPoseMatx44f = nT;}
    
    cv::Matx44f&        Matx44f(void){return mPoseMatx44f;}
    const cv::Matx44f&  getMatx44f(void)const{return mPoseMatx44f;}
    
    cv::Vec3f           getPosition(void){ return Type::cvtMatx44fToPosition(getMatx44f()); }
    cv::Vec3f           cvtToCamera(const cv::Vec3f& nPtInWorld)const{return Type::project(mPoseMatx44f, nPtInWorld); };
    cv::Vec3f           cvtToWorld(const cv::Vec3f& nPtInCamera)const{ return Type::project(mPoseMatx44f.inv(), nPtInCamera);};
    
    
private:
    cv::Matx44f     mPoseMatx44f;
};

typedef std::shared_ptr<TpCameraPose> TpPtrCameraPose;
 
}
}

#endif

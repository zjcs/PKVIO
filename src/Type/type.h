#ifndef __TYPE__
#define __TYPE__
#include <string>
#include <opencv/cv.h>

using namespace std;

namespace PKVIO{
namespace Type{
typedef int     TpFrameID;
typedef double  TpTimeStamp;

typedef enum {
    TpMono			= 0x01<<0,
    TpRight			= 0x01<<1,
    TpDepth     	= 0x01<<2,

    TpStereo		= TpMono | TpRight,
    TpRGBD			= TpMono | TpDepth,
    TpStereoDepth 	= TpStereo | TpDepth
} TpFrame;

class Frame{
public:
    virtual                 ~Frame(){}
    virtual TpFrame         type(void) {return TpMono;}
    inline const TpFrameID& FrameID(void){ return mFrmID; }
    inline void             initFrameID(const TpFrameID& nFrmID){ mFrmID = nFrmID; };
    inline cv::Mat&         getImage(void){ return mImage; }
protected:
    cv::Mat mImage;
    TpFrameID mFrmID;
};

class StereoFrame: public Frame{
public:
    virtual TpFrame         type(void) override { return TpStereo; }
    inline cv::Mat&         getImageLeft(void){ return getImage(); }
    inline cv::Mat&         getImageRight(void){ return mImageRight; }
protected:
    cv::Mat mImageRight;
};

string cvtTimeStampToString(const TpTimeStamp& t);

}

using namespace Type;
}

#endif

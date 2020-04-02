#ifndef __TYPE__
#define __TYPE__
#include <string>
#include <opencv/cv.h>
#include <queue>
#include <list>

using namespace std;

namespace PKVIO{
namespace Type{
typedef int     TpFrameID;
typedef int     TpFrameIndex;
typedef double  TpTimeStamp;

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

class Frame{
public:
    virtual                 ~Frame(){}
    virtual TpFrame         type(void) const {return TpMono;}
    inline const TpFrameID& FrameID(void)const { return mFrmID; }
    inline void             initFrameID(const TpFrameID& nFrmID){ mFrmID = nFrmID; };
    inline cv::Mat&         getImage(void) { return mImage; }
    inline const cv::Mat&   Image(void) const {return mImage; }
protected:
    cv::Mat mImage;
    TpFrameID mFrmID;
};

class StereoFrame: public Frame{
public:
    virtual TpFrame         type(void) const override {return TpStereo;}
    inline cv::Mat&         getImageLeft(void){ return getImage(); }
    inline cv::Mat&         getImageRight(void){ return mImageRight; }
    inline const cv::Mat&   ImageLeft(void) const {return Image(); }
    inline const cv::Mat&   ImageRight(void) const {return mImageRight; }
protected:
    cv::Mat mImageRight;
};
    
typedef     std::vector<cv::KeyPoint>   TpVecKeyPoints;
typedef     cv::Mat                     TpVecDescriptor;

typedef vector<cv::DMatch>              TpVecMatchResult;
typedef pair<int,int>                   TpMatchPair;
typedef vector<TpMatchPair>             TpVecMatchPairs;

TpMatchPair                             cvtMatchToMatchPair(const cv::DMatch& m);


template<typename T>
class FixLengthQueue: public list<T>
{
public:
    FixLengthQueue(const int nMaxLength = 10):mMaxLength(std::max(nMaxLength, 1)){}
    void            push(const T& t) {
        auto nCurSz = (int)this->size();
        if(nCurSz>=mMaxLength){
            for(int nIdx=0,nPop=nCurSz+1-mMaxLength;nIdx<nPop;++nIdx){
                list<T>::pop_front();
            }
        }
        list<T>::push_back(t);
    }
    inline virtual bool     isFull(void){return mMaxLength>(int)this->size();}
private:
    const int mMaxLength;
};

string cvtTimeStampToString(const TpTimeStamp& t);

}

using namespace Type;
}

#endif

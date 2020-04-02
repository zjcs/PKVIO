#ifndef __TYPE__
#define __TYPE__
#include <string>
#include <opencv/cv.h>
#include <queue>
#include <list>
#include <functional>

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
    FixLengthQueue(const int nMaxLength = 10):mMaxLength(nMaxLength){}
    void            push(const T& t) {
        auto nCurSz = (int)this->size();
        if(isFixedLength() && nCurSz>=mMaxLength){
            for(int nIdx=0,nPop=nCurSz+1-mMaxLength;nIdx<nPop;++nIdx){
                list<T>::pop_front();
            }
        }
        list<T>::push_back(t);
    }
    inline virtual bool     isFull(void){return isFixedLength() && mMaxLength>(int)this->size();}
protected:
    inline bool             isFixedLength(void) {return mMaxLength > 0;}
private:
    const int mMaxLength;       // if is lessequal than 0, means not fixed length and save all values.
};

// // okay
//template<int n>
//class nTTest{
//    
//};
// // wrong, is not a valid type for a template non-type parameter.
//template<typename std::function<bool(int&,const int)> >
//class nTTest2{
//    
//};
//typedef template<typename TpID, typename T > std::function<bool(T&,const TpID)> TpFuncIDAndTMatch;
//template <typename TpID, typename T,  TpFuncIDAndTMatch f>
//template <typename TpID, typename T,  std::function<bool(T&,const TpID)> f>
template <typename T, typename TpID>
class DataHistoryTemplate{
public:
    typedef std::function<bool(T&,const TpID)>  TpFuncIsIDAndTMatch;
    typedef std::vector<T>                      TpVecTs;
    DataHistoryTemplate(TpFuncIsIDAndTMatch/*std::function<bool(T&,const TpID)>*/ fFuncIsIDAndTMatch)
    : mFuncIsIDAndTMatch(fFuncIsIDAndTMatch)
    {
        
    }
    
    //operator= (DescriptorHistory& op){}

    void            push(const T& One){return mHistory.push(One);}
    bool            isExisting(const TpID nID){
                        for(auto Iter = mHistory.begin(), EndIter = mHistory.end(); Iter!=EndIter; ++Iter) {
                            T& h = *Iter;
                            if(mFuncIsIDAndTMatch(h, nID))
                                return true;
                        }
                        return false;
                    }
    T&              get(const TpID nID){
                        for(auto Iter = mHistory.begin(), EndIter = mHistory.end(); Iter!=EndIter; ++Iter) {
                            T& h = *Iter;
                            if(mFuncIsIDAndTMatch(h, nID))
                                return h;
                        }
                        // not found, errro call; should call isExisting() before get();
                        throw;
                    }
    inline T&       back(void){return mHistory.back();}
    inline T&       getLastOne(void){return back();}
    TpVecTs         getLastSeveral(int nSz, std::function<bool(T&)> bFuncIsThisToBeExcluded = [](T&)->bool{return false;}){
                        TpVecTs vTs;
                        vTs.reserve(nSz);
                        int nNotUsed = nSz;
                        for(auto iter = mHistory.rbegin(), EndIter = mHistory.rend(); iter!=EndIter && (nNotUsed--); ++iter) {
                            if(bFuncIsThisToBeExcluded(*iter))
                                continue;
                            vTs.push_back(*iter);
                        }
                        // free the memory of last NotUsed place.
                        auto LastIter = vTs.begin();
                        std::advance(LastIter, vTs.size());
                        vTs.erase(LastIter, vTs.end());
                        // Error as below;
                        //vTs.erase(std::advance(vTs.begin(), vTs.size()), vTs.end());
                        return vTs;
                                            
                    }
    bool            empty(void){return !size();}
    int             size(void){return (int)mHistory.size();}
protected:
   Type::FixLengthQueue<T>  mHistory;
private:
   TpFuncIsIDAndTMatch      mFuncIsIDAndTMatch;
};


string cvtTimeStampToString(const TpTimeStamp& t);

}

using namespace Type;
}

#endif

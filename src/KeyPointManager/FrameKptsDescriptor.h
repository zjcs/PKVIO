#ifndef __FRAMEKPTSDESCRIPTOR_H__
#define __FRAMEKPTSDESCRIPTOR_H__

#include "../Type/type.h"

namespace PKVIO{
namespace KeyPointManager{
    
typedef enum{
    TpStillExisting,
    TpNotHere,
    TpRemoved,
    TpIsComing
} TpHistoryStorageState;

typedef struct{
    TpVecKeyPoints  mKeyPointsLeft; 
    TpVecDescriptor mDescriptorsLeft;
    TpVecKeyPoints  mKeyPointsRight; 
    TpVecDescriptor mDescriptorsRight;
    Type::TpFrameID mFrameIDLeft;
    Type::TpFrameID mFrameIDRight;
    inline const TpFrameID& FrameID(void)const { return FrameIDLeft(); }
    inline const TpFrameID& FrameIDLeft(void)const { return mFrameIDLeft; }
    inline const TpFrameID& FrameIDRight(void)const { return mFrameIDRight; }
    
    void getMatchKpts(const TpVecMatchResult& nMatch, cv::Mat& mKptL, cv::Mat& mKptR){
        size_t nSz = nMatch.size();
        std::vector<cv::Point2f> vKptL, vKptR;
        vKptL.reserve(nSz); vKptR.reserve(nSz);
        for(int nIdx=0;nIdx<nSz;++nIdx){
            const auto& nM = nMatch[nIdx];
            vKptL.push_back(mKeyPointsLeft[nM.queryIdx].pt);
            vKptR.push_back(mKeyPointsRight[nM.trainIdx].pt);
        }
        cv::Mat(vKptL).convertTo(mKptL, CV_32F);
        cv::Mat(vKptR).convertTo(mKptR, CV_32F);
    }
    
    void getMatchKpts(const TpVecMatchResult& nMatch, TpVecKeyPoints& mKptL, TpVecKeyPoints& mKptR){
        size_t nSz = nMatch.size();
        mKptL.reserve(nSz); mKptR.reserve(nSz);
        for(int nIdx=0;nIdx<nSz;++nIdx){
            const auto& nM = nMatch[nIdx];
            mKptL.push_back(mKeyPointsLeft[nM.queryIdx]);
            mKptR.push_back(mKeyPointsRight[nM.trainIdx]);
        }
    }
   
} TpOneFrameKptDescriptor;

typedef std::vector<TpOneFrameKptDescriptor> TpVecFrameKptDescriptors;
    
/*
class DescriptorHistory{
public:
    DescriptorHistory();
    //operator= (DescriptorHistory& op){}

    void                                push(TpOneFrameKptDescriptor& One);

    bool                                isExisting(const TpFrameID nFrmID);
    TpOneFrameKptDescriptor&            get(const TpFrameID nFrmID);
    inline TpOneFrameKptDescriptor&     back(void){return mHistory.back();}
    inline TpOneFrameKptDescriptor&     getLastFrame(void){return back();}
    TpVecFrameKptDescriptors            getLastFrames(int nSz, bool bIncludeKeyFrame = true);
    TpVecFrameKptDescriptors            getLastKeyFrames(int nSz);
    
    bool                                empty(void){return mHistory.size();}
private:
   Type::FixLengthQueue<TpOneFrameKptDescriptor> mHistory;
};
*/

typedef std::function<bool(Type::Frame& fFrame,const TpFrameID nFrameID)>               TpFuncIsIDAndFrameMatch;
// another decleration method.
//typedef bool(*TpFuncIsIDAndFrameMatch)(TpOneFrameKptDescriptor&fFrame,const TpFrameID nFrameID);
typedef std::function<bool(TpOneFrameKptDescriptor&fFrame,const TpFrameID nFrameID)>    TpFuncIsIDAndFrameKptsDescriptorMatch;
typedef std::function<bool(TpOneFrameKptDescriptor&fFrame)>                             TpFuncIsFrameStasify;

extern TpFuncIsIDAndFrameMatch                        FuncIsIDAndFrameMatch;
extern TpFuncIsIDAndFrameKptsDescriptorMatch          FuncIsIDAndFrameKptsDescriptorMatch;
extern TpFuncIsFrameStasify             FuncIsKeyFrame;
extern TpFuncIsFrameStasify             FuncIsNotKeyFrame;


template<typename TpFrameType>
bool FuncIsIDAndFrameDataMatch(TpFrameType& fFrame, const TpFrameID nFrameID)
{
    return fFrame.FrameID() == nFrameID;
}

template<typename TpFrameType>
bool FuncIsKeyFrameTemplate(TpFrameType& fFrame){
    return fFrame.FrameID() == 0;
}

template<typename TpFrameType>
bool FuncIsNotKeyFrameTemplate(TpFrameType& fFrame){
    return !FuncIsKeyFrame(fFrame);
}

template<typename TpFrameType>
class TpFrameDataHistory: public DataHistoryTemplate<TpFrameType, TpFrameID>
{
public:
    // Info: due to template function only have full specification, so the bellow, calling , is wrong.
    // TpFrameDataHistory(TpFuncIsIDAndTMatch fFuncIsIDAndTMatch);
    // TpFrameDataHistory(DataHistoryTemplate::TpFuncIsIDAndTMatch fFuncIsIDAndTMatch);
    // TpFrameDataHistory(DataHistoryTemplate<TpFrameType,TpFrameID>::TpFuncIsIDAndTMatch fFuncIsIDAndTMatch);
    // template<TpFrameType, TpFrameID>
    // TpFrameDataHistory(DataHistoryTemplate::TpFuncIsIDAndTMatch fFuncIsIDAndTMatch);
    
    // only one right way is to define the function template by hand.
    TpFrameDataHistory(std::function<bool(TpFrameType&, const TpFrameID)> fFuncIsIDAndTMatch) 
    : DataHistoryTemplate<TpFrameType, TpFrameID> (fFuncIsIDAndTMatch)
    { }
    
    //DataHistoryTemplate<TpFrameType>::TpVecTs     getLastFrames(int nSz, bool bIncludeKeyFrame = true)
    
    // Info: only this way is okay.
    inline typename DataHistoryTemplate<TpFrameType, TpFrameID>::TpVecTs     getLastFrames(int nSz, bool bIncludeKeyFrame = true)
    {
        return bIncludeKeyFrame ? DataHistoryTemplate<TpFrameType, TpFrameID>::getLastSeveral(nSz, FuncIsKeyFrameTemplate) 
                                : DataHistoryTemplate<TpFrameType, TpFrameID>::getLastSeveral(nSz);
    } 
    
    inline typename DataHistoryTemplate<TpFrameType, TpFrameID>::TpVecTs     getLastKeyFrames(int nSz)
    {
        return DataHistoryTemplate<TpFrameType, TpFrameID>::getLastSeveral(nSz, FuncIsNotKeyFrame);
    }
};

// method#1 using full specification;
// Info: for specificied template, no need with template<> before sub-derived class.
//template<>
//class KptsDescriptorHistory: public DataHistoryTemplate<TpOneFrameKptDescriptor, TpFrameID>
//{
//public:
//    KptsDescriptorHistory();
//    
//    inline TpVecFrameKptDescriptors     getLastFrames(int nSz, bool bIncludeKeyFrame = true);
//    inline TpVecFrameKptDescriptors     getLastKeyFrames(int nSz);
//};
//
//typedef KptsDescriptorHistory DescriptorHistory;

// method#2 using harf specification;
class FrameKptsDescriptorHistory: public TpFrameDataHistory<TpOneFrameKptDescriptor>
{
public:
    FrameKptsDescriptorHistory():TpFrameDataHistory(FuncIsIDAndFrameKptsDescriptorMatch){}
};


class FrameHistory: public TpFrameDataHistory<Type::Frame>
{
public:
    // another constructor method.
    //FrameHistory():TpFrameDataHistory([](Type::Frame& fFrame, const TpFrameID nFrmID)->bool{return fFrame.FrameID() == nFrmID;}){}
    FrameHistory():TpFrameDataHistory(FuncIsIDAndFrameMatch){}
};

class StereoFrameHistory: public TpFrameDataHistory<Type::StereoFrame>
{
public:
    // another constructor method.
    //FrameHistory():TpFrameDataHistory([](Type::Frame& fFrame, const TpFrameID nFrmID)->bool{return fFrame.FrameID() == nFrmID;}){}
    StereoFrameHistory():TpFrameDataHistory(FuncIsIDAndFrameMatch){}
};


}
}
#endif

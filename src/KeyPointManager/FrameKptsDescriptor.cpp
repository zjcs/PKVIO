#include "FrameKptsDescriptor.h"

namespace PKVIO{
namespace KeyPointManager{
    
TpFuncIsIDAndFrameMatch     FuncIsIDAndFrameMatch = [](Type::Frame&fFrame, const TpFrameID nFrmID)->bool{return fFrame.FrameID() == nFrmID;};
//std::function(bool<TpOneFrameKptDescriptor&fFrame,const TpFrameID nFrameID>)
TpFuncIsIDAndFrameKptsDescriptorMatch    
                            FuncIsIDAndFrameKptsDescriptorMatch = [](TpOneFrameKptDescriptor&fFrame,const TpFrameID nFrameID)->bool {return fFrame.FrameID()==nFrameID;};
TpFuncIsFrameStasify        FuncIsKeyFrame = [](TpOneFrameKptDescriptor& T)->bool{ throw; return false; };
TpFuncIsFrameStasify        FuncIsNotKeyFrame = [](TpOneFrameKptDescriptor& T)->bool{return !FuncIsKeyFrame(T);};
//
//DescriptorHistory::DescriptorHistory() 
//: mHistory(20+5)
//{
//    //
//}
//
//void DescriptorHistory::push(TpOneFrameKptDescriptor& One) {
//    mHistory.push(One);
//}
//
//
//bool DescriptorHistory::isExisting(const int nFrmID) {
//    for(auto Iter = mHistory.begin(), EndIter = mHistory.end(); Iter!=EndIter; ++Iter) {
//        const TpOneFrameKptDescriptor& h = *Iter;
//        if(h.mFrameID == nFrmID)
//            return true;
//    }
//    return false;
//    //for(const TpOneFrameKptDescriptor& h in mHistory){
//    //    if(h.mFrameID == nFrmID)
//    //        return true;
//    //}
//    //return false;
//}
//
//
//TpOneFrameKptDescriptor& DescriptorHistory::get(const PKVIO::Type::TpFrameID nFrmID) {
//    for(auto Iter = mHistory.begin(), EndIter = mHistory.end(); Iter!=EndIter; ++Iter) {
//        TpOneFrameKptDescriptor& h = *Iter;
//        if(h.mFrameID == nFrmID)
//            return h;
//    }
//
//    throw;
//}
//
//TpVecFrameKptDescriptors DescriptorHistory::getLastFrames(int nSz, bool bIncludeKeyFrame /*= true*/) {
//    TpVecFrameKptDescriptors vFrameKptDescriptors;
//    vFrameKptDescriptors.reserve(nSz);
//    for(auto iter = mHistory.rbegin(), EndIter = mHistory.rend(); iter!=EndIter && (nSz--); ++iter) {
//        bool bIsKeyFrame = false; 
//        // TODO
//        if(!bIncludeKeyFrame && bIsKeyFrame)
//            continue;
//        vFrameKptDescriptors.push_back(*iter);
//    }
//    return vFrameKptDescriptors;
//}
//

//
//KptsDescriptorHistory::KptsDescriptorHistory()
//: DataHistoryTemplate /* <TpFrameID, TpOneFrameKptDescriptor>*/ (FuncIsIDAndFrameKptsDescriptorMatch)
//{ }
//
//
//TpVecFrameKptDescriptors KptsDescriptorHistory::getLastFrames(int nSz, bool bIncludeKeyFrame /*=true*/)
//{
//    return bIncludeKeyFrame? getLastSeveral(nSz, FuncIsKeyFrame) : getLastSeveral(nSz);
//}
//
//TpVecFrameKptDescriptors KptsDescriptorHistory::getLastKeyFrames(int nSz) {
//    return getLastSeveral(nSz, FuncIsNotKeyFrame);
//}

}
}

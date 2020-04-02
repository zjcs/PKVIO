#include "FrameKptsDescriptor.h"

namespace PKVIO{
namespace KeyPointManager{

DescriptorHistory::DescriptorHistory() 
: mHistory(20+5)
{
    //
}

void DescriptorHistory::push(TpOneFrameKptDescriptor& One) {
    mHistory.push(One);
}


bool DescriptorHistory::isExisting(const int nFrmID) {
    for(auto Iter = mHistory.begin(), EndIter = mHistory.end(); Iter!=EndIter; ++Iter) {
        const TpOneFrameKptDescriptor& h = *Iter;
        if(h.mFrameID == nFrmID)
            return true;
    }
    return false;
    //for(const TpOneFrameKptDescriptor& h in mHistory){
    //    if(h.mFrameID == nFrmID)
    //        return true;
    //}
    //return false;
}


TpOneFrameKptDescriptor& DescriptorHistory::get(const PKVIO::Type::TpFrameID nFrmID) {
    for(auto Iter = mHistory.begin(), EndIter = mHistory.end(); Iter!=EndIter; ++Iter) {
        TpOneFrameKptDescriptor& h = *Iter;
        if(h.mFrameID == nFrmID)
            return h;
    }

    throw;
}


}
}

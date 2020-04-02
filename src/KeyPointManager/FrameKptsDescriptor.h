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
   Type::TpFrameID mFrameID;
} TpOneFrameKptDescriptor;
    
class DescriptorHistory{
public:
    DescriptorHistory();
    //operator= (DescriptorHistory& op){}

    void                        push(TpOneFrameKptDescriptor& One);

    bool                        isExisting(const TpFrameID nFrmID);
    TpOneFrameKptDescriptor&    get(const TpFrameID nFrmID);
private:
   Type::FixLengthQueue<TpOneFrameKptDescriptor> mHistory;
};
 
}
}
#endif

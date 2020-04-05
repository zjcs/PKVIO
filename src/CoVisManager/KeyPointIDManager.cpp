#include "KeyPointIDManager.h"

namespace PKVIO {
namespace KeyPointManager {
void KeyPointIDManager::addOneFrameIDManager(const Type::TpFrameIndex nFrameIndex, const Type::TpFrameID nFrameID, const int nCountKpts) 
{
    // allocate memory.
    mSetFrameIndex2FrameID.resize(nFrameIndex+1);
    mSetFrameIndex2OneFrameIDManager.resize(nFrameIndex+1);

    // initialize data.
    mSetFrameID2FrameIndex[nFrameID] = nFrameIndex;
    mSetFrameIndex2FrameID[nFrameIndex] = nFrameID;
    mSetFrameIndex2OneFrameIDManager[nFrameIndex] = TpOneFrameIDManager(nCountKpts);
}
}
}


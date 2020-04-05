#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <functional>
#include "../DatasetManager/DatasetInterface.h"
#include "../KeyPointManager/KeyPointManager.h"
#include "../CoVisManager/CoVisManager.h"
#include "../KeyFrameManager/KeyFrameManager.h"

using namespace std;

namespace PKVIO{
namespace System{
    
using namespace DatasetManager;

class System{
public:
    typedef std::function<void(void)> TpFuncDoExec;
    
    void showVideoOnly(void);
    void runVIO(void);
protected:
    void exec(void);
    
    void initialize(void);
    void doexec(void);
    void exit(void);
private:
    void debugCountTrackingKptIDWihtMapPointID(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
private:
    TpFuncDoExec mPtrFuncDoExec;
    DatasetManager::DatasetInterfacePtr mPtrDataset;
    KeyPointManager::KeyPointManager    mKeyPointMgr;
    CoVisManager::CoVisManager          mCoVisMgr;
    KeyFrameManager::PtrKeyFrameManager mPtrKeyFrameMgr;
};

}
}

#endif

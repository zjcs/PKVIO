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
    System(): mbRunAllFrame(true){
        // initialize();
    }
    ~System();
    
    void showVideoOnly(void);
    void runVIO(void);
    
    void setRunVIO(bool bRunAllFrame = true);
    
    void setRunVIOSimple(bool bRunAllFrame = true);
    void setRunVIOSimpleStereo(bool bRunAllFrame = true);
    
    void initialize(const DebugManager::TpDebugControl& nDbgCtrl);
    void doexec(void);
    void exit(void);
    
    cv::Mat     getDispalyImage(void){return mTrackingImageCurFrame;}
    cv::Vec3f   getCameraPoseCurFrame(void){return mPtrCameraPoseCurFrame->getPosition();}
protected:
    void exec(void);
    
private:
    cv::Matx44f solverCurrentFramePose(const TpFrameID nFrameIDCur);
    
    void        triangularMapPoint(Frame& mCurFrame);
    
    
    void debugCountTrackingKptIDWihtMapPointID(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr);
   
    map<TpFrameID, map<TpKeyPointIndex, TpPtrMapPoint3D>> nMapMap3D;
private:
    TpFuncDoExec                        mPtrFuncDoExec;
    DatasetManager::DatasetInterfacePtr mPtrDataset;
    KeyPointManager::KeyPointManager    mKeyPointMgr;
    CoVisManager::CoVisManager          mCoVisMgr;
    KeyFrameManager::PtrKeyFrameManager mPtrKeyFrameMgr;
    
    bool                                mbRunAllFrame;
    cv::Mat                             mTrackingImageCurFrame;
    TpPtrCameraPose                     mPtrCameraPoseCurFrame;
    
    DebugManager::TpDebugControl*       mPtrDbgCtrl;
};

typedef std::shared_ptr<System> TpPtrVIOSystem;

TpPtrVIOSystem generateVIOSystem(void);

}
}

#endif

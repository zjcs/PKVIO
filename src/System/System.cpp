#include <iostream>
#include <opencv2/highgui.hpp>
#include "System.h"
#include "../DatasetManager/DatasetEuRoc.h"
#include "../DatasetManager/DatasetSimulator.h"

#include "../Tools/Tools.h"
#include "../Solver/Solver.h"

namespace PKVIO {
namespace System {

TpPtrVIOSystem generateVIOSystem(void){
  return std::make_shared<System>();
};

    
System::~System(){
    exit();
}

void System::exec(void) {
    doexec();
}

void System::initialize(bool bUseSimulator /*= false*/) {
    //mPtrDataset = std::make_shared<DatasetEuRoc>("E:/DataSet/MH_01_easy/");
    string sDatasetPath = std::string("/home/ubuntu/work/dataset/DataSet/MH_01_easy/");
    
    if(!bUseSimulator) {
        mPtrDataset = std::make_shared<DatasetEuRoc>(sDatasetPath);   
    } else {
        mPtrDataset = std::make_shared<DatasetSimulator>(sDatasetPath);
    }
    
    mPtrDataset->initialize();
    
    if(dynamic_cast<DatasetSimulator*>(mPtrDataset.get())){
        mKeyPointMgr.setSimulator(true, mPtrDataset->getPtrCamera());
    }
    
    mPtrKeyFrameMgr = std::make_shared<KeyFrameManager::KeyFrameManager>();
    
    cout << "PkVio System intialization Finish." << endl;
}

void System::exit(void) {
    cout << "PkVio System Exit Now." << endl;
}

void System::doexec(void) {
    mPtrFuncDoExec();
}


void System::showVideoOnly() {
    auto FuncDoShowVideoOnly = [&](){
        for (int nIndex = 0; !mPtrDataset->isFinished(); ++nIndex) {
            Frame& mCurFrame = mPtrDataset->read();
            cv::imshow("Viewer", mCurFrame.getImage());
            cv::waitKey(40);
        }
    };
    
    mPtrFuncDoExec = FuncDoShowVideoOnly;
    exec();
}


void System::runVIO() {
    setRunVIO();
    exec();
}


void System::debugCountTrackingKptIDWihtMapPointID(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    int nCurCountKptIDsWithMapPoint = mPtrKeyFrameMgr->countTrackKptIDsWithMapPointID(mFrameKptIDMgr);
    static int nPrevCountKptIDsWithMapPoint = 0;
    
    if(mPtrKeyFrameMgr->isKeyFrame(fFrame.FrameID())) {
        // do nothing.
    } else {
        if(nCurCountKptIDsWithMapPoint>nPrevCountKptIDsWithMapPoint) {
            
            cout << "**** Error: Match Point number should decreas... Prev|Cur - " << nPrevCountKptIDsWithMapPoint 
                 << " | " << nCurCountKptIDsWithMapPoint << endl;
            
            TpFrameID nFrameIDPre2 = fFrame.FrameID()-2, nFrameIDPrev = fFrame.FrameID()-1, nFrameIDCur = fFrame.FrameID();
            
            KeyPointManager::TpOneFrameKptDescriptor nFrameKptDescriptorPre2Prev;
            KeyPointManager::TpDescriptorMatchResult nDescriptorsMatchResultPre2Prev;
            mKeyPointMgr.getTrackingKptDescriptorMatchResult(nFrameIDPre2, nFrameIDPrev, nFrameKptDescriptorPre2Prev, nDescriptorsMatchResultPre2Prev);
            
            KeyPointManager::TpOneFrameKptDescriptor nFrameKptDescriptorPrevCur;
            KeyPointManager::TpDescriptorMatchResult nDescriptorsMatchResultPrevCur;
            mKeyPointMgr.getTrackingKptDescriptorMatchResult(nFrameIDPrev, nFrameIDCur, nFrameKptDescriptorPrevCur, nDescriptorsMatchResultPrevCur);
            
            cv::Mat mPr2PrevMatch = mKeyPointMgr.showMatchResult(nFrameIDPre2, nFrameIDPrev , true);
            cv::Mat mPrevCurMatch = mKeyPointMgr.showMatchResult(nFrameIDPrev, nFrameIDCur, true);
            cv::waitKey();
            
        }
    }
    nPrevCountKptIDsWithMapPoint = nCurCountKptIDsWithMapPoint;
}


void System::setRunVIO(bool bRunAllFrame /*= true*/) {
    auto FuncDorunVIO = [&](){
        for (int nIndex = 0; !mPtrDataset->isFinished(); ++nIndex) {
            Frame& mCurFrame = mPtrDataset->read();
            
            
            if(mCurFrame.getImage().empty() && DatasetManager::isOfflineDatasetType(mPtrDataset->type())){
                FrameInfo mFrmInfo = dynamic_cast<DatasetOfflineImp*>(mPtrDataset.get())->getFrameInfor(mCurFrame.FrameID());
                cout<< "Empyt:" << mFrmInfo.mFrameIndex << " - " << mFrmInfo.mStrFileName <<endl;
            }
            
            auto mPtrDatasetEuRoc = dynamic_cast<DatasetManager::DatasetEuRoc*>(mPtrDataset.get());
            cv::Matx44f nPrTPl = mPtrDataset->getPtrCamera()->getTranslateCvtPtLViewToRView().get();
            const auto& nCameraLeft  = mPtrDataset->getPtrCamera()->CameraLeft();
            const auto& nCameraRight = mPtrDataset->getPtrCamera()->CameraRight();
            const TpCameraInnerParam& nCameraInnerLeft  = mPtrDataset->getPtrCamera()->CameraLeft().getInnerParam();
            const TpCameraInnerParam& nCameraInnerRight = mPtrDataset->getPtrCamera()->CameraRight().getInnerParam();
            
            //cout << "Feature Matching ..." <<endl;
            const KeyPointManager::FrameMatchResult& mFrameMatchResult = mKeyPointMgr.solve(mCurFrame);
            //cout << "Feature Matching Finish." <<endl;
            
            auto& mMatchResult = mFrameMatchResult.getInnerFrameDescriptorMatchResult();
            auto& mKptsDescriptors = mKeyPointMgr.getDescriptor(mCurFrame.FrameID());
            TpVecMatchResult nVecMatchResultWithParall;
            TpVecMatchResult nVecMatchResultWithoutParall;
            
#if 1
            cout << "Parallax on Stereo Frame :" <<endl;
            for(int nIdxMatch=0,nSzMatch=mMatchResult.get().size();nIdxMatch<nSzMatch;++nIdxMatch){
                TpKeyPointIndex nKptIdxLeft, nKptIdxRight;
                mMatchResult.getMatchKptIndex(nIdxMatch, nKptIdxLeft, nKptIdxRight);
                TpKeyPoint nKptLeft  = mKptsDescriptors.mKeyPointsLeft[nKptIdxLeft];
                TpKeyPoint nKptRight = mKptsDescriptors.mKeyPointsRight[nKptIdxRight];
                TpKeyPoint nKptLeftUndistor = nCameraInnerLeft.undistor(nKptLeft);
                TpKeyPoint nKptRightUndistor = nCameraInnerRight.undistor(nKptRight);
                auto nPtNormLeft  = nCameraInnerLeft.cvtPixelToNorm(nKptLeftUndistor.pt);
                auto nPtNormRight = nCameraInnerRight.cvtPixelToNorm(nKptRightUndistor.pt);
                
                cv::Vec3f nMapPointLeft;
                bool bTriangular; 
                bTriangular = (nKptRightUndistor.pt.x - nKptLeftUndistor.pt.x) < -1e-3;
                //bool bTriangular2 = Tools::triangulation(nPtNormLeft, nPtNormRight, nPrTPl, nMapPointLeft);
                //cout << nKptIdxLeft<<"-" << nKptRightUndistor.pt-nKptLeftUndistor.pt;
                if(bTriangular){
                    nVecMatchResultWithParall.push_back(mMatchResult.get()[nIdxMatch]);
                }else{
                    nVecMatchResultWithoutParall.push_back(mMatchResult.get()[nIdxMatch]);
                }
            }
            //cout <<endl;
            //Tools::drawMatch(mCurFrame.getImage(), mKptsDescriptors.mKeyPointsLeft, dynamic_cast<StereoFrame&>(mCurFrame).getImageRight(), mKptsDescriptors.mKeyPointsRight, nVecMatchResultWithParall, true, "Parall" );
            //Tools::drawMatch(mCurFrame.getImage(), mKptsDescriptors.mKeyPointsLeft, dynamic_cast<StereoFrame&>(mCurFrame).getImageRight(), mKptsDescriptors.mKeyPointsRight, nVecMatchResultWithoutParall, true, "NoParall" );
            //cv::waitKey();
#endif
            
            
            //cout << "CoVis Graph ..." <<endl;
            mCoVisMgr.solve(mCurFrame, mFrameMatchResult);
            //cout << "CoVis Graph Finish." <<endl;
            
            KeyPointManager::TpOneFrameIDManager& mOneFrameIDMgr = mCoVisMgr.OneFrameKptIDMgrByFrameID(mCurFrame.FrameID());
            mPtrKeyFrameMgr->solve(mCurFrame, mFrameMatchResult, mOneFrameIDMgr);
            
            debugCountTrackingKptIDWihtMapPointID(mCurFrame, mFrameMatchResult, mOneFrameIDMgr);
            
            if(mPtrKeyFrameMgr->isKeyFrame(mCurFrame.FrameID())){
                auto& nMapPointIDMgr = mPtrKeyFrameMgr->getMapPointIDManager();
                TpVecMapPointID nVecMapPointID = nMapPointIDMgr.getMapPointIDsGeneratedByFrame(mCurFrame.FrameID());
                map<TpKeyPointIndex, TpMapPointID> nMapKptIdx2MpID;
                for(int nIdxMapPointID=0;nIdxMapPointID<nVecMapPointID.size();++nIdxMapPointID){
                    TpMapPointID nMapPointIDNew = nVecMapPointID[nIdxMapPointID];
                    // triangulation;
                    auto nFrmIDAndKptIdx = nMapPointIDMgr.MapPoint(nMapPointIDNew).getVecMeasurments()[0];
                    TpKeyPointIndex nKptIdxLeft = nFrmIDAndKptIdx.second;
                    nMapKptIdx2MpID[nKptIdxLeft] = nMapPointIDNew;
                }
                
                const TpVecKeyPoints& nVecKptLeft = mKeyPointMgr.getDescriptor(mCurFrame.FrameID()).mKeyPointsLeft;
                const TpVecKeyPoints& nVecKptRight = mKeyPointMgr.getDescriptor(mCurFrame.FrameID()).mKeyPointsRight;
                const TpVecMatchPairs& nInnerMatch = mKeyPointMgr.getFrameMatchResult().getInnerFrameDescriptorMatchResult().getMatchPairs();
                cout << "For Key Point triangular on KF" <<endl;
                for(int nIdxMatch=0,nSzMatch = nInnerMatch.size();nIdxMatch<nSzMatch;++nIdxMatch){
                    const TpMatchPair& nMatchPair = nInnerMatch[nIdxMatch];
                    const TpKeyPointIndex& nMatchKptIdxLeft  = nMatchPair.first;
                    const TpKeyPointIndex& nMatchKptIdxRight = nMatchPair.second;
                    auto Iter = nMapKptIdx2MpID.find(nMatchKptIdxLeft);
                    if(Iter==nMapKptIdx2MpID.end())
                        continue;
                    const TpMapPointID& nMpID = Iter->second;
                    KeyFrameManager::TpMapPoint& nMp =  nMapPointIDMgr.MapPoint(nMpID);
                    TpKeyPoint nKptLeftUndistor  = nCameraLeft.getInnerParam().undistor(nVecKptLeft[nMatchKptIdxLeft]);
                    TpKeyPoint nKptRightUndistor = nCameraRight.getInnerParam().undistor(nVecKptRight[nMatchKptIdxRight]);
                    
                    float nDepthInLeftView = 0;
                    if(Tools::triangulation(nKptLeftUndistor.pt, nKptRightUndistor.pt, 435.262, 47.912663, nDepthInLeftView)){
                        TpKeyPoint nKptLeftUndistorNorm  = nCameraLeft.getInnerParam().cvtPixelToNorm(nKptLeftUndistor);
                        cv::Vec3f  nMp3DLeftView(nKptLeftUndistorNorm.pt.x*nDepthInLeftView,
                                                 nKptLeftUndistorNorm.pt.y*nDepthInLeftView, nDepthInLeftView);
                        auto nMp3DWorld = mPtrKeyFrameMgr->getFrameCameraPose(mCurFrame.FrameID())->cvtToWorld(nMp3DLeftView);
                        nMp.initMapPoint3D(cv::Point3f(nMp3DWorld));
                        cout << "Pixel Left|Right - PtInWorld(LeftView): " << nKptLeftUndistor.pt << "|" << nKptRightUndistor.pt << "  - "<< nMp3DWorld <<endl;
                    }
                }
                //cout <<endl;
            }else{
                // non-kf observe should insert to the mappoint?
            }
            
            cv::Matx44f nFramePoseCur = solverCurrentFramePose(mCurFrame.FrameID());
            mPtrCameraPoseCurFrame = mPtrKeyFrameMgr->getFrameCameraPose(mCurFrame.FrameID());
            
            cv::Mat mImgToShow = mCurFrame.Image().clone();
#if 0
            if(mKeyPointMgr.queryDescriptorExisting(mCurFrame.FrameID())){
                KeyPointManager::TpOneFrameKptDescriptor& CurFrmKptsDescriptor = mKeyPointMgr.getDescriptor(mCurFrame.FrameID());
                mImgToShow = Tools::drawKeyPoints(mImgToShow, CurFrmKptsDescriptor.mKeyPointsLeft);
                const Camera& nCameraLeft = mPtrDataset->getPtrCamera()->getCameraLeft();
                cv::Mat nImgUndistorLeft = nCameraLeft.getInnerParam().undistor(mCurFrame.Image());
                TpVecKeyPoints nVecKptUndistort = nCameraLeft.getInnerParam().undistor(CurFrmKptsDescriptor.mKeyPointsLeft);
                
                //mImgToShow = Tools::drawMatch(mImgToShow, CurFrmKptsDescriptor.mKeyPointsLeft, nImgUndistorLeft, nVecKptUndistort, false, "Distor|Undistor");
                mImgToShow = Tools::drawMatch(mImgToShow, CurFrmKptsDescriptor.mKeyPointsLeft, nImgUndistorLeft, nVecKptUndistort, false);
            }
#else
            auto& nFrmKptDes = mKeyPointMgr.getFrameKptsDescriptorHistory().get(mCurFrame.FrameID());
            auto& nFrmMatch  = mKeyPointMgr.getFrameMatchResult().getInnerFrameDescriptorMatchResult().get();
            mImgToShow       = Tools::drawMatch(mImgToShow, nFrmKptDes.mKeyPointsLeft, dynamic_cast<StereoFrame&>(mCurFrame).getImageRight(),
                                                nFrmKptDes.mKeyPointsRight, nFrmMatch, false);
#endif
            
            
            if(!mImgToShow.empty()){
                mTrackingImageCurFrame = mImgToShow;
                //cv::imshow("Viewer", mImgToShow);
                //cv::waitKey(mbRunAllFrame?40:1);
            }
            
            if(!mbRunAllFrame){
                break;
            }
        }
    };
    
    mPtrFuncDoExec = FuncDorunVIO;
    mbRunAllFrame = bRunAllFrame;
}

cv::Matx44f System::solverCurrentFramePose(const TpFrameID nFrameIDCur) {
    cv::Matx44f nFramePoseCur = cv::Matx44f::eye();
    if(nFrameIDCur == 0)    // TODO Fist Frame, need try other way.
        return nFramePoseCur;
    
    TpVecKeyPointID nVecKptIDs; TpVecKeyPointIndex nVecKptIndexs;
    KeyPointManager::TpOneFrameIDManager& nFrameIDManagerCur = mCoVisMgr.OneFrameKptIDMgrByFrameID(nFrameIDCur);
    nFrameIDManagerCur.getAllKptIDsAndIdexs(nVecKptIDs, nVecKptIndexs);
    
    KeyFrameManager::TpFrameKptIDMapPointPair nFrameKptIDMapPointPair(nFrameIDCur);
    mPtrKeyFrameMgr->getKptIDsWithMapPointID(nFrameIDManagerCur, nFrameKptIDMapPointPair);
    
    map<TpKeyPointID, TpMapPointID> nMapKeyPointID2MapPointID = nFrameKptIDMapPointPair.getMapKptID2MapPointID();
    vector<KeyFrameManager::TpKptIDMapPointPairWithFrameID> nVecKptIDMapPointPairWithFrameID;
    auto FuncGetCoVisWithCurrentFrame = [&](const TpFrameID nCosVisFrmID) {
            if(nFrameIDCur - nCosVisFrmID>DebugManager::getMaxCoVisLength()) return;
        
            TpVecKeyPointID nVecKptIDs; TpVecKeyPointIndex nVecKptIndexs;
            mCoVisMgr.OneFrameKptIDMgrByFrameID(nCosVisFrmID).getAllKptIDsAndIdexs(nVecKptIDs, nVecKptIndexs);
            for(int nIdxKptID=0,nSzKptIDs=nVecKptIDs.size();nIdxKptID<nSzKptIDs;++nIdxKptID){
                const TpKeyPointID nKptID       = nVecKptIDs[nIdxKptID];
                auto IterFind = nMapKeyPointID2MapPointID.find(nKptID);
                if(IterFind == nMapKeyPointID2MapPointID.end())
                    continue;
                
                // get one co-vis
                const TpKeyPointIndex nKptIdex  = nVecKptIndexs[nIdxKptID];
                KeyFrameManager::TpKptIDMapPointPairWithFrameID nOneMesurement(nCosVisFrmID, nKptID, nKptIdex, IterFind->second);
                nVecKptIDMapPointPairWithFrameID.push_back(nOneMesurement);
            }
        };
    mCoVisMgr.collectCoVisInfo(nFrameIDCur, FuncGetCoVisWithCurrentFrame);
    
    // Another way is (1)get all MapPointIDs in Current Frame, (2) get all measurments stored in TpMapPoint
    //      how to solve the measurment from non-keyframe, directly added in TpMapPoint, which still is not done in runVIO?
    
    // Through nVecKptIDMapPointPairWithFrameID get measurment Info: camera pose 6D, keypoint pixel 2D, mapoint 3D.
    // TODO
    map<TpFrameID, cv::Mat>             nMapFrameID2FrameImage;
    map<TpFrameID, TpPtrCameraPose>     nMapFrameID2CameraPose;
    map<TpMapPointID, TpPtrMapPoint3D>  nMapMapPointID2MapPoint3D;
    Solver::TpVecVisualMeasurement      nVecVisualMeasurement;
    
    for(int nIdxMeasurement=0,nSzMeasurements = nVecKptIDMapPointPairWithFrameID.size();nIdxMeasurement<nSzMeasurements;++nIdxMeasurement){
        auto& nMeasurement = nVecKptIDMapPointPairWithFrameID[nIdxMeasurement];
        if(nMapFrameID2CameraPose.find(nMeasurement.mFrameID) == nMapFrameID2CameraPose.end()){
            nMapFrameID2CameraPose[nMeasurement.mFrameID] = mPtrKeyFrameMgr->getFrameCameraPose(nMeasurement.mFrameID);
            nMapFrameID2FrameImage[nMeasurement.mFrameID] = mKeyPointMgr.getStereoFrameHistory().get(nMeasurement.mFrameID).getImageLeft().clone();
        }
        if(nMapMapPointID2MapPoint3D.find(nMeasurement.mMapPointID) == nMapMapPointID2MapPoint3D.end()){
            //nMapMapPointID2MapPoint3D[nMeasurement.mMapPointID] = TpMapPoint3D();
            const KeyFrameManager::TpMapPoint& nMp = mPtrKeyFrameMgr->getMapPointIDManager().MapPoint(nMeasurement.mMapPointID);
            if(!nMp.getMapPoint3DValid())
                continue;
            nMapMapPointID2MapPoint3D[nMeasurement.mMapPointID] = nMp.getMapPoint3D();
        }
    }
    
    Type::TpPtrCameraStereo nPtrCameraStereo = mPtrDataset->getPtrCamera();
    
    
    for(int nIdxMeasurement=0,nSzMeasurements = nVecKptIDMapPointPairWithFrameID.size();nIdxMeasurement<nSzMeasurements;++nIdxMeasurement){
        auto& nMeasurement = nVecKptIDMapPointPairWithFrameID[nIdxMeasurement];
        TpFrameID nFrameID = nMeasurement.mFrameID;
        if(mKeyPointMgr.getFrameKptsDescriptorHistory().isExisting(nFrameID)){
            
            cv::KeyPoint nKeyPoint = mKeyPointMgr.getFrameKptsDescriptorHistory().get(nFrameID).mKeyPointsLeft[nMeasurement.mKptIndex];
            //TpMapPoint3D nMapPoint = mPtrKeyFrameMgr->getMapPointIDManager().MapPoint(nMeasurement.mMapPointID).MapPoint3D();
            cv::KeyPoint nKeyPointUnDistor = mPtrDataset->getPtrCamera()->CameraLeft().getInnerParam().undistor(nKeyPoint);
            if(nMapMapPointID2MapPoint3D.find(nMeasurement.mMapPointID)==nMapMapPointID2MapPoint3D.end())
                continue;
            
            Solver::TpVisualMeasurement nVisualMeasurement;
            nVisualMeasurement.mFrameID     = nFrameID;
            nVisualMeasurement.mMapPointID  = nMeasurement.mMapPointID;
            //nVisualMeasurement.mKeyPoint  = nKeyPoint;
            nVisualMeasurement.mKeyPoint    =  nKeyPointUnDistor;
            
            //nSolverCurFramePose.addMeasurement(nVisualMeasurement);
            nVecVisualMeasurement.push_back(nVisualMeasurement);
        }else{
            cout << "Error: FrameKptsDescriptorHistory has been freed, FrameID:" << nFrameID<<endl;
            throw;
        }
    }
#if 0
    
    {
        
    bool bShowAll = false;
    for(auto Iter = nMapFrameID2CameraPose.begin();Iter!=nMapFrameID2CameraPose.end() && bShowAll;++Iter){
        auto nFrameIDCoVis = Iter->first;
        Tools::drawMatch(
            mKeyPointMgr.getStereoFrameHistory().get(nFrameIDCur).getImageLeft(),
            mKeyPointMgr.getDescriptor(nFrameIDCur).mKeyPointsLeft,
            mKeyPointMgr.getStereoFrameHistory().get(nFrameIDCoVis).getImageLeft(),
            mKeyPointMgr.getDescriptor(nFrameIDCoVis).mKeyPointsLeft,
            mCoVisMgr.getCoVis(nFrameIDCur, nFrameIDCoVis), 
            true, "CoVis"+cvtToString(nFrameIDCur)+"|"+cvtToString(nFrameIDCoVis));
        
        bShowAll = cv::waitKey() != 27;
    }
    cv::destroyAllWindows();
    }
    
    for(int nIdxVm=0,nSzVm=nVecVisualMeasurement.size();nIdxVm<nSzVm;++nIdxVm){
        const Solver::TpVisualMeasurement& nVm = nVecVisualMeasurement[nIdxVm];
        if(nMapFrameID2FrameImage.find(nVm.mFrameID) == nMapFrameID2FrameImage.end()){
            cout << "Error: " << nVm.mFrameID<<endl;
        }
        Tools::drawKeyPoints(nMapFrameID2FrameImage[nVm.mFrameID], TpVecKeyPoints(1, nVm.mKeyPoint));
        cv::putText(nMapFrameID2FrameImage[nVm.mFrameID], cvtToString(nVm.mMapPointID), nVm.mKeyPoint.pt, 1, 1, cv::Scalar(255));
    }
    {
    bool bShowAll = false;
    for(auto Iter = nMapFrameID2CameraPose.begin();Iter!=nMapFrameID2CameraPose.end() && bShowAll;++Iter){
        auto nFrameIDCoVis = Iter->first;
        cout << "Disp: " << nFrameIDCoVis << " - " << nFrameIDCur <<endl;
        Tools::showImageIfTitleNotEmpty(nMapFrameID2FrameImage[nFrameIDCoVis], "Vm"+cvtToString(nFrameIDCur)+"|"+cvtToString(nFrameIDCoVis));
        bShowAll = cv::waitKey() != 27;
    }
    cv::destroyAllWindows();
    }
#endif
    
    
    Solver::Solver nSolverCurFramePose;
    //nSolverCurFramePose.initCamerPoses(nMapFrameID2CameraPose);
    //nSolverCurFramePose.initMapPoints(nMapMapPointID2MapPoint3D);
    nSolverCurFramePose.solve(nMapFrameID2CameraPose, nMapMapPointID2MapPoint3D, nVecVisualMeasurement, nPtrCameraStereo);
    auto nPtrCameraPoseCur = nMapFrameID2CameraPose[nFrameIDCur];
    nFramePoseCur = nPtrCameraPoseCur->getMatx44f();
    //cout << "nFramePoseCur:" << nFrameIDCur <<endl << nFramePoseCur <<endl;
    cout    << "nFramePoseCur | nCosVisFrm | nMeasurement - "
            << nFrameIDCur << " | " << nMapFrameID2CameraPose.size() << " | " << nVecVisualMeasurement.size() <<endl 
            << nPtrCameraPoseCur->getPosition() <<endl;
    
    return nFramePoseCur;
}
}

}


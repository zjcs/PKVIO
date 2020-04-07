#include <iostream>
#include <opencv2/highgui.hpp>
#include "System.h"
#include "../DatasetManager/DatasetEuRoc.h"
#include "../Tools/Tools.h"


namespace PKVIO {
namespace System {

void System::exec(void) {
    initialize();
    doexec();
    exit();
    
}

void System::initialize(void) {
    //mPtrDataset = std::make_shared<DatasetEuRoc>("E:/DataSet/MH_01_easy/");
    string sDatasetPath = std::string("/home/ubuntu/work/dataset/DataSet/MH_01_easy/");
    
    mPtrDataset = std::make_shared<DatasetEuRoc>(sDatasetPath);
    mPtrDataset->initialize();
    
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
    
    auto FuncDorunVIO = [&](){
        for (int nIndex = 0; !mPtrDataset->isFinished(); ++nIndex) {
            Frame& mCurFrame = mPtrDataset->read();
            
            
            if(mCurFrame.getImage().empty() && DatasetManager::isOfflineDatasetType(mPtrDataset->type())){
                FrameInfo mFrmInfo = dynamic_cast<DatasetOfflineImp*>(mPtrDataset.get())->getFrameInfor(mCurFrame.FrameID());
                cout<< "Empyt:" << mFrmInfo.mFrameIndex << " - " << mFrmInfo.mStrFileName <<endl;
            }
            
            //cout << "Feature Matching ..." <<endl;
            const KeyPointManager::FrameMatchResult& mFrameMatchResult = mKeyPointMgr.solve(mCurFrame);
            //cout << "Feature Matching Finish." <<endl;
            
            //cout << "CoVis Graph ..." <<endl;
            mCoVisMgr.solve(mCurFrame, mFrameMatchResult);
            //cout << "CoVis Graph Finish." <<endl;
            
            KeyPointManager::TpOneFrameIDManager& mOneFrameIDMgr = mCoVisMgr.OneFrameKptIDMgrByFrameID(mCurFrame.FrameID());
            mPtrKeyFrameMgr->solve(mCurFrame, mFrameMatchResult, mOneFrameIDMgr);
            
            debugCountTrackingKptIDWihtMapPointID(mCurFrame, mFrameMatchResult, mOneFrameIDMgr);
            
            if(mPtrKeyFrameMgr->isKeyFrame(mCurFrame.FrameID())){
                // non-kf observe should insert to the mappoint.
            }else{
                
            }
            
            cv::Matx44f nFramePoseCur = solverCurrentFramePose(mCurFrame.FrameID());
            
            cv::Mat& mImgToShow = mCurFrame.Image();
            if(mKeyPointMgr.queryDescriptorExisting(mCurFrame.FrameID())){
                KeyPointManager::TpOneFrameKptDescriptor& CurFrmKptsDescriptor = mKeyPointMgr.getDescriptor(mCurFrame.FrameID());
                mImgToShow = Tools::drawKeyPoints(mImgToShow, CurFrmKptsDescriptor.mKeyPointsLeft);
            }
            if(!mImgToShow.empty()){
                cv::imshow("Viewer", mImgToShow);
                cv::waitKey(40);
            }
        }
    };
    
    mPtrFuncDoExec = FuncDorunVIO;
    exec();
}


void System::debugCountTrackingKptIDWihtMapPointID(Type::Frame& fFrame, const KeyPointManager::FrameMatchResult& mFrameMatchResult, KeyPointManager::TpOneFrameIDManager& mFrameKptIDMgr) {
    int nCurCountKptIDsWithMapPoint = mPtrKeyFrameMgr->countTrackKptIDsWithMapPointID(mFrameKptIDMgr);
    static int nPrevCountKptIDsWithMapPoint = 0;
    
    if(mPtrKeyFrameMgr->isKeyFrame(fFrame.FrameID())) {
        // do nothing.
    } else {
        if(nCurCountKptIDsWithMapPoint>nPrevCountKptIDsWithMapPoint) {
            
            cout << "**** Error: Match Point number should decreas... Prev|Cur - " << nPrevCountKptIDsWithMapPoint <<" | " << nCurCountKptIDsWithMapPoint << endl;
            
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

cv::Matx44f System::solverCurrentFramePose(const TpFrameID nFrameIDCur) {
    cv::Matx44f nFramePoseCur = cv::Matx44f::eye();
    if(nFrameIDCur == 0)    // TODO Fist Frame, need try other way.
        return nFramePoseCur;
    
    TpVecKeyPointID nVecKptIDs; TpVecKeyPointIndex nVecKptIndexs;
    KeyPointManager::TpOneFrameIDManager& nFrameIDManagerCur = mCoVisMgr.OneFrameKptIDMgrByFrameID(nFrameIDCur);
    nFrameIDManagerCur.getAllKptIDsAndIdexs(nVecKptIDs, nVecKptIndexs);
    
    KeyFrameManager::TpFrameKptIDMapPointPair nFrameKptIDMapPointPair(nFrameIDCur);
    mPtrKeyFrameMgr->getKptIDsWithMapPointID(nFrameIDManagerCur, nFrameKptIDMapPointPair);
    
    map<TpKeyPointID, TpMapPointID> nMapKeyPointID2MapPointID;
    for(int nIdxPair=0,nSzPairs=nFrameKptIDMapPointPair.size();nIdxPair<nSzPairs;++nIdxPair){
        const KeyFrameManager::TpKptIDMapPointPair& nPair = nFrameKptIDMapPointPair.getKptIDMapPointPair(nIdxPair);
        nMapKeyPointID2MapPointID[nPair.mKptID] = nPair.mMapPointID;
    }
    vector<KeyFrameManager::TpKptIDMapPointPairWithFrameID> nVecKptIDMapPointPairWithFrameID;
    
    auto FuncGetCoVisWithCurrentFrame = [&](const TpFrameID nCosVisFrmID, 
                                            const TpVecKeyPointID& nVecKptIDs, const TpVecKeyPointIndex& nVecKptIndexs){
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
    
    // Through nVecKptIDMapPointPairWithFrameID get measurment Info: camera pose 6D, keypoint pixel 2D, mapoint 3D.
    // TODO
    
    return nFrameIDCur;
}
}

}


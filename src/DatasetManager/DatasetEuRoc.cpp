#include "DatasetEuRoc.h"
//#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/ml.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <functional>
#include <opencv2/calib3d.hpp>
#include <../Tools/Tools.h>

namespace PKVIO{
namespace DatasetManager{

DatasetEuRoc::DatasetEuRoc(const string& sDatasetPath)  
: DatasetOfflineImp(sDatasetPath)
, mBoolUseUndistorCamera(true)
{
}

void DatasetEuRoc::initialize(void) {
    parseDatasetTimeStamp();

    initializeSize((int)mVecImageTimeStamp.size());

    parseCalibration();
}

Frame* DatasetEuRoc::load(const int nIndexToRead) {
    //Image
    StereoFrame* pStereoIMUFrame = new StereoFrame;
    getImage(nIndexToRead, pStereoIMUFrame->ImageLeft(), pStereoIMUFrame->ImageRight());
    //IMU
    getIMU();
    // Generate FrameID;
    pStereoIMUFrame->initFrameID(GeneratorFrameID());
    pStereoIMUFrame->initFrameIndex(nIndexToRead);
    
    remapInputStereoFrame(pStereoIMUFrame);
    
    return pStereoIMUFrame;
}

string DatasetEuRoc::getLeftViewFolder(void){
    return getDatasetPath() + "/mav0/cam0/";
}

string DatasetEuRoc::getRightViewFolder(void){
    return getDatasetPath() + "/mav0/cam1/";
}

string DatasetEuRoc::getIMUFolder() {
    return getDatasetPath() + "/mav0/imu0/";
}

void DatasetEuRoc::getImage(int nIndexToRead, cv::Mat& mLeft, cv::Mat& mRight) {
    string sLeftFile = getFrameAbsFileNmae(nIndexToRead, true);
    mLeft = cv::imread(sLeftFile, cv::IMREAD_GRAYSCALE);
    string sRightFile = getFrameAbsFileNmae(nIndexToRead, false);
    mRight = cv::imread(sRightFile, cv::IMREAD_GRAYSCALE);
    //cout << sLeftFile <<endl;
}


void DatasetEuRoc::getIMU() {
}

void DatasetEuRoc::parseDatasetTimeStamp() {
    //TODO; Camera TimeStamp;
    /*
    {
        string sTimeStampFile = getDatasetPath() + "/mav0/cam0/data.csv";
        ifstream fCameraTimeStamp (sTimeStampFile.c_str());
        if(!fCameraTimeStamp.is_open()){
            cerr << "Fail to open EuRoc TimeStamp File, exit." << sTimeStampFile <<endl;
            throw;
        }
        string sLine;
        while(getline(fCameraTimeStamp,sLine )){
            if(sLine.empty() || sLine[0]=='#')continue;
            int nIdx = std::distance(sLine.begin(), std::find(sLine.begin(),sLine.end(),','));
            string sFileName = sLine.substr(nIdx+1, sLine.size());
            istringstream sTimeStampStream(sLine.substr(0, nIdx));
            double tTimeStamp;
            sTimeStampStream >> tTimeStamp;
            
            mVecImageTimeStamp.push_back(tTimeStamp);
            // cout << fixed << setw(19) << tTimeStamp << " - " << sFileName <<endl;
        }
        fCameraTimeStamp.close(); 
        
    } 
    */
   
    auto FuncLoadCSV = [](const string& sFile, std::function<void(const string& sLine)> funcParse){
        ifstream fCameraTimeStamp (sFile.c_str());
        if(!fCameraTimeStamp.is_open()){
            cerr << "Fail to open TimeStamp File, exit." << sFile <<endl;
            throw;
        }
        string sLine;
        while(getline(fCameraTimeStamp,sLine )){
            if(sLine.empty() || sLine[0]=='#')continue;
            funcParse(sLine);
        }
        fCameraTimeStamp.close(); 
            
    };
   
    string sTimeStampFile = getDatasetPath() + "/mav0/cam0/data.csv";
    FuncLoadCSV(sTimeStampFile, [&](const string& sLine){
        int nIdx = std::distance(sLine.begin(), std::find(sLine.begin(),sLine.end(),','));
        string sFileName = sLine.substr(nIdx+1, sLine.size());
        istringstream sTimeStampStream(sLine.substr(0, nIdx));
        double tTimeStamp;
        sTimeStampStream >> tTimeStamp;
        
        this->mVecImageTimeStamp.push_back(tTimeStamp);
        //cout << fixed << setw(19) << tTimeStamp << " - " << sFileName <<endl;
    });
    
    
    string sIMUTimeStampFile = getDatasetPath() + "/mav0/imu0/data.csv";
    FuncLoadCSV(sIMUTimeStampFile, [&](const string& ssLine){
        string sLine = ssLine;
        for(int nIdx=0,nSz=sLine.size();nIdx<nSz;++nIdx)
            if(sLine[nIdx]==',')sLine[nIdx]=' ';
        
        double tTimeStamp; cv::Vec6d fIMU;
        
        istringstream sTimeStampStream(sLine);
        sTimeStampStream >> tTimeStamp;
        for(int nIdx=0;nIdx<6;++nIdx) {
            sTimeStampStream>>fIMU[nIdx];   
            //cout << setw(19) << fIMU[nIdx]<<endl;
        }
        
        this->mVecIMUTimeStamp.push_back(std::make_pair(tTimeStamp, fIMU));
        //cout << "src:" << sLine<<endl;
        //cout << fixed << setw(19) << tTimeStamp << " - " << fIMU <<endl;
    });
    cout << "Warning: for IMU data, only 0.6f precision, a little different to original value."<<endl;
}


void DatasetEuRoc::parseCalibration() {
    /*
    OpenCv read Yaml, you need some change for *.yaml
    %YAML:1.0
    T_BS: !!opencv-matrix
      dt: d
    */
    
    //left
    string sLeftCalibFile = getLeftViewFolder() + "/sensor.yaml";
    TpCameraInnerParam nCameraInnerParamLeft; TpCameraOuterParam nCameraOuterParamLeft;
    parseCalibrationCamera(sLeftCalibFile, nCameraInnerParamLeft, nCameraOuterParamLeft);
    Camera nCameraLeftDistor  = Camera(nCameraInnerParamLeft);
    
    //right
    string sRightCalibFile = getRightViewFolder() + "/sensor.yaml";
    TpCameraInnerParam nCameraInnerParamRight; TpCameraOuterParam nCameraOuterParamRight;
    parseCalibrationCamera(sRightCalibFile, nCameraInnerParamRight, nCameraOuterParamRight);
    Camera nCameraRightDistor = Camera(nCameraInnerParamRight);
    cv::Matx44f nCameraPrTPl = nCameraOuterParamRight.get().inv()*nCameraOuterParamLeft.get();
    //nCameraPrTPl = nCameraPrTPl.inv();
    
    cv::Mat R,T, R1,R2,P1,P2,Q; cv::Mat Rvec,Tvec;
    cv::Size nSz = nCameraInnerParamLeft.getImageSize();
    cv::Mat nCameraPrTPl64f = Type::cvtMat32fToMat64f(cv::Mat(nCameraPrTPl));
    Type::cvtMatx44fToR33T31(cv::Mat(nCameraPrTPl64f), R, T);
    Type::cvtMatx44fToRTvec(cv::Mat(nCameraPrTPl64f), Rvec, Tvec);
    //cout << CV_32F << "," << CV_64F <<endl;
    //R1 = cv::Mat(3,3, CV_64FC1); R2 = cv::Mat(3,3, CV_64FC1); P1 = cv::Mat(3,4, CV_64FC1); P2 = cv::Mat(3,4, CV_64FC1); Q  = cv::Mat(4,4, CV_64FC1);
    //cout << nCameraInnerParamLeft.getInnerMat64F().type() <<"," <<endl << nCameraInnerParamRight.getInnerMat64F().type() <<"," <<endl << nCameraInnerParamLeft.getDistorParam64F().type() <<"," <<endl <<  nCameraInnerParamRight.getDistorParam64F().type()<<"," <<endl << R.type() <<"," <<endl <<  T.type() <<endl;
    //cout << nCameraInnerParamLeft.getInnerMat() <<"," <<endl << nCameraInnerParamRight.getInnerMat() <<"," <<endl << nCameraInnerParamLeft.getDistorParam() <<"," <<endl <<  nCameraInnerParamRight.getDistorParam() <<endl << R <<"," <<endl <<  T <<endl; 
    cv::stereoRectify(nCameraInnerParamLeft.getInnerMat64F(), nCameraInnerParamLeft.getDistorParam64F(),
                     nCameraInnerParamRight.getInnerMat64F(), nCameraInnerParamRight.getDistorParam64F(),
                      cv::Size2i(nSz), R, T, R1,R2,P1,P2,Q, CV_CALIB_ZERO_DISPARITY, 0
    );
    mStereoRectResult = new TpStereoRectResult(R,T,R1,R2,P1,P2,Q,nSz);
    //cout << "------------" <<endl;
    //cout << R <<endl << T <<endl << R1 <<endl << R2 <<endl << P1 <<endl << P2 <<endl <<endl;
    //cout << "stereo" <<endl;
    
    cv::Vec2f fxyUndistor, cxyUndistor;
    cvtProjectionMatrixToFxyCxy(P1, fxyUndistor, cxyUndistor);
    TpCameraInnerParam nCameraInnerParamLeftUndistor(fxyUndistor(0), fxyUndistor(1), cxyUndistor(0),cxyUndistor(1), nSz.width, nSz.height);
    //TpCameraOuterParam nCameraOuterParamLeftUndistor(Type::cvtR33T31ToMatx44f(R1, cv::Mat(cv::Vec3d(0,0,0))));
    TpCameraOuterParam nCameraOuterParamLeftUndistor(cv::Matx44f::eye());
    Camera nCameraLeftDistorUndistor  = Camera(nCameraInnerParamLeftUndistor);
    
    cv::Vec2f fxyRightUndistor, cxyRightUndistor;
    cvtProjectionMatrixToFxyCxy(P2, fxyRightUndistor, cxyRightUndistor);
    assert((fxyRightUndistor-fxyUndistor).dot((fxyRightUndistor-fxyUndistor))<1);
    assert((cxyRightUndistor-cxyUndistor).dot((cxyRightUndistor-cxyUndistor))<1);
    TpCameraInnerParam nCameraInnerParamRightUndistor(fxyUndistor(0), fxyUndistor(1), cxyUndistor(0),cxyUndistor(1), nSz.width, nSz.height);
    cv::Mat R2Norm = cv::Mat::eye(3,3,CV_64FC1); // R2Norm = R2;
    TpCameraOuterParam nCameraOuterParamRightUndistor(Type::cvtR33T31ToMatx44f(R2Norm, cv::Mat(cv::Vec3d(P2.at<double>(0,3),0,0))));
    Camera nCameraRightDistorUndistor = Camera(nCameraInnerParamRightUndistor);
    
    Camera nCameraLeftUndistor (nCameraInnerParamLeftUndistor);
    Camera nCameraRightUndistor(nCameraInnerParamRightUndistor);
    cv::Matx44f nCameraPrTPlUndistor = nCameraOuterParamRightUndistor.get()*nCameraOuterParamLeftUndistor.get().inv();
    
    //cout << nCameraOuterParamLeftUndistor.str()<<endl << nCameraOuterParamRightUndistor.str()<<endl;
    //cout<< nCameraPrTPlUndistor <<endl;
    
    //imu
    string sIMUCalibFile = getIMUFolder() + "/sensor.yaml";
    parseCalibrationIMU(sIMUCalibFile);
    
    mPtrCameraStereo = std::make_shared<CameraStereo>();
    
    mPtrCameraStereo->CameraLeft()  = nCameraLeftDistor;
    mPtrCameraStereo->CameraRight() = nCameraRightDistor;
    mPtrCameraStereo->setCameraOuterParamCvtPtLViewToRView(nCameraPrTPl);
    
    if(getUseUndistorInput()){
        mPtrCameraStereo->CameraLeft() = nCameraLeftUndistor;
        mPtrCameraStereo->CameraRight() = nCameraRightUndistor;
        mPtrCameraStereo->setCameraOuterParamCvtPtLViewToRView(nCameraPrTPlUndistor);
    }else{
        // TODO: camera param(RT) still have some problem
        throw;
    }
    cout << "EuRoc Sensor Config:" << endl << mPtrCameraStereo->str() <<endl;
    //cout << "PrTPl :" <<endl << mPtrCameraStereo->CameraLeft().getCameraOuterParam().get() <<endl << mPtrCameraStereo->CameraRight().getCameraOuterParam().get() <<endl;
}


void DatasetEuRoc::parseCalibrationIMU(const string& sIMUCalibFile) {

}


const string DatasetEuRoc::getFrameAbsFileNmae ( Type::TpFrameIndex nFrmIndex, bool bTrueLeftFalseRight )
{
    return ( bTrueLeftFalseRight?getLeftViewFolder() :getRightViewFolder() )+"/data/"+getFrameFileName ( nFrmIndex );
}


void DatasetEuRoc::parseCalibrationCamera(const string& sCameraCalibFile, PKVIO::Type::TpCameraInnerParam& nCameraInnerParam,
                                          PKVIO::Type::TpCameraOuterParam& nCameraOuterParam)
{
    cv::FileStorage fs(sCameraCalibFile, cv::FileStorage::READ);
    if(!fs.isOpened()){
        cout << "Error: fail to open CameraCalibFile, exit:" <<endl << sCameraCalibFile <<endl;
    }
    
    // T_BS, the camera refereced to IMU(Body) Coordination, Xb = T_BS* Xc which transpose the Xc to Xb.
    cv::Mat T_BS ; 
    fs["T_BS"] >> T_BS;
    cv::Vec2i wxhy;
    fs["resolution"] >> wxhy;
    cv::Vec4f fxycxy;
    fs["intrinsics"] >> fxycxy;
    cv::Vec4f fdistor;
    fs["distortion_coefficients"] >> fdistor;
    fs.release();
    
    nCameraInnerParam  = Type::TpCameraInnerParam(wxhy, fxycxy, fdistor);
    nCameraOuterParam  = Type::TpCameraOuterParam(T_BS);
}


void DatasetEuRoc::remapInputStereoFrame(Type::StereoFrame* pStereoFrame) {
    auto mPtrDataset = this;
    
    cv::Matx44f nPrTPl = mPtrDataset->getPtrCamera()->getTranslateCvtPtLViewToRView().get();
    const auto& nCameraLeft  = mPtrDataset->getPtrCamera()->CameraLeft();
    const auto& nCameraRight = mPtrDataset->getPtrCamera()->CameraRight();
    const TpCameraInnerParam& nCameraInnerLeft  = mPtrDataset->getPtrCamera()->CameraLeft().getInnerParam();
    const TpCameraInnerParam& nCameraInnerRight = mPtrDataset->getPtrCamera()->CameraRight().getInnerParam();

    StereoFrame& mStereoCurFrame = *pStereoFrame;
    TpStereoRectResult& nStereoRect = *this->mStereoRectResult;
    cv::Mat nRemapXLeft,nRemapYLeft, nRemapXRight,nRemapYRight;
    cv::initUndistortRectifyMap(nCameraInnerLeft.getInnerMat(), nCameraInnerLeft.getDistorParam(),
                                nStereoRect.R1, nStereoRect.P1,nStereoRect.Sz,CV_32FC1,nRemapXLeft, nRemapYLeft );
    cv::initUndistortRectifyMap(nCameraInnerRight.getInnerMat(), nCameraInnerRight.getDistorParam(),
                                nStereoRect.R2, nStereoRect.P2,nStereoRect.Sz,CV_32FC1,nRemapXRight, nRemapYRight );
    cv::Mat nImageStereoRectLeft,nImageStereoRectRight;
    cv::remap(mStereoCurFrame.getImageLeft(), nImageStereoRectLeft, nRemapXLeft, nRemapYLeft, CV_INTER_LINEAR);
    cv::remap(mStereoCurFrame.getImageRight(), nImageStereoRectRight, nRemapXRight, nRemapYRight, CV_INTER_LINEAR);
    cv::Mat nImageStereoRectStereo = Tools::merge(nImageStereoRectLeft, nImageStereoRectRight);
    //Tools::showImageIfTitleNotEmpty(nImageStereoRectStereo, "StereoRect");

    if(getUseUndistorInput()){
        pStereoFrame->ImageLeft()  = nImageStereoRectLeft;
        pStereoFrame->ImageRight() = nImageStereoRectRight;
    }
}

}
}

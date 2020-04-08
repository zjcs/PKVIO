#include "DatasetEuRoc.h"
//#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/ml.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <functional>

namespace PKVIO{
namespace DatasetManager{

DatasetEuRoc::DatasetEuRoc(const string& sDatasetPath)  
:DatasetOfflineImp(sDatasetPath){
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
    
    
    mPtrCameraStereo = std::make_shared<CameraStereo>();
    //left
    string sLeftCalibFile = getLeftViewFolder() + "/sensor.yaml";
    TpCameraInnerParam nCameraInnerParamLeft; TpCameraDistorParam nCameraDistorParamLeft; TpCameraOuterParam nCameraOuterParamLeft;
    parseCalibrationCamera(sLeftCalibFile, nCameraInnerParamLeft, nCameraDistorParamLeft, nCameraOuterParamLeft);
    mPtrCameraStereo->CameraLeft() = Camera(nCameraInnerParamLeft, nCameraDistorParamLeft);
    
    //right
    string sRightCalibFile = getRightViewFolder() + "/sensor.yaml";
    TpCameraInnerParam nCameraInnerParamRight; TpCameraDistorParam nCameraDistorParamRight; TpCameraOuterParam nCameraOuterParamRight;
    parseCalibrationCamera(sRightCalibFile, nCameraInnerParamRight, nCameraDistorParamRight, nCameraOuterParamRight);
    mPtrCameraStereo->CameraRight() = Camera(nCameraInnerParamRight, nCameraDistorParamRight);
    
    cout << "Info: here is need test. Not sure it's right." << endl;
    mPtrCameraStereo->setCameraOuterParamCvtPtLViewToRView(nCameraOuterParamLeft.get().t()*nCameraOuterParamRight.get());
    
    //imu
    string sIMUCalibFile = getIMUFolder() + "/sensor.yaml";
    parseCalibrationIMU(sIMUCalibFile);
}


void DatasetEuRoc::parseCalibrationIMU(const string& sIMUCalibFile) {

}


const string DatasetEuRoc::getFrameAbsFileNmae ( Type::TpFrameIndex nFrmIndex, bool bTrueLeftFalseRight )
{
    return ( bTrueLeftFalseRight?getLeftViewFolder() :getRightViewFolder() )+"/data/"+getFrameFileName ( nFrmIndex );
}


void DatasetEuRoc::parseCalibrationCamera(const string& sCameraCalibFile, PKVIO::Type::TpCameraInnerParam& nCameraInnerParam,
                                          PKVIO::Type::TpCameraDistorParam& nCameraDistorParam, PKVIO::Type::TpCameraOuterParam& nCameraOuterParam)
{
    cv::FileStorage fs(sCameraCalibFile, cv::FileStorage::READ);
    if(!fs.isOpened()){
        cout << "Error: fail to open CameraCalibFile, exit:" <<endl << sCameraCalibFile <<endl;
    }
    
    cv::Mat T_BS ; 
    fs["T_BS"] >> T_BS;
    cv::Vec2i wxhy;
    fs["resolution"] >> wxhy;
    cv::Vec4f fxycxy;
    fs["intrinsics"] >> fxycxy;
    cv::Vec4f fdistor;
    fs["distortion_coefficients"] >> fdistor;
    fs.release();
    
    nCameraInnerParam  = Type::TpCameraInnerParam(wxhy, fxycxy);
    nCameraDistorParam = Type::TpCameraDistorParam(fdistor);
    nCameraOuterParam  = Type::TpCameraOuterParam(T_BS);
}

}
}

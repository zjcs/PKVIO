#include "DatasetOfflineImp.h"

namespace PKVIO
{
namespace DatasetManager
{
    
DatasetOfflineImp::DatasetOfflineImp(const std::string& sDatasetPath)
: nIndexNextToRead(0)
, mStrDatasetPath(sDatasetPath)
{
    
}


Type::Frame& DatasetOfflineImp::read() {
    mPtrCurrentFrame = load(nIndexNextToRead);
    ++nIndexNextToRead;
    return CurrentFrame();
}


bool DatasetOfflineImp::isFinished() {
    return size() <= nIndexNextToRead;
}


int DatasetOfflineImp::size() {
    return mSzDataset;
}


Type::Frame& DatasetOfflineImp::CurrentFrame() {
    return *mPtrCurrentFrame;
}


const std::string& DatasetOfflineImp::getDatasetPath() {
    return mStrDatasetPath;
}


Type::FrameInfo DatasetOfflineImp::getFrameInfor ( const Type::TpFrameID nFrmID )
{
    FrameInfo mFrmInfo;
    if(mPtrCurrentFrame->FrameID() == nFrmID){
        mFrmInfo.mFrameID = nFrmID; 
        mFrmInfo.mFrameIndex = nIndexNextToRead-1;
        mFrmInfo.mStrFileName = getFrameFileName(mFrmInfo.mFrameIndex);
        mFrmInfo.mStrFileAbsName = getFrameAbsFileNmae(mFrmInfo.mFrameIndex, true);
        return mFrmInfo;
    }
    cout << "Error: TODO:DatasetOfflineImp::getFrameInfor " <<endl;
    throw;
    return FrameInfo();
}

}
}

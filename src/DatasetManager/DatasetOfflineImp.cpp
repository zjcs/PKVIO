#include "DatasetOfflineImp.h"

namespace PKVIO
{
namespace DatasetManager
{
    
DatasetOfflineImp::DatasetOfflineImp(const std::string& sDatasetPath)
: mStrDatasetPath(sDatasetPath)
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

}
}

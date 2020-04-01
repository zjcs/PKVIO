#ifndef __DATASETOFFLINEIMP_H__
#define __DATASETOFFLINEIMP_H__

#include "DatasetInterface.h"
#include <vector>
#include <string>
#include <queue>

namespace PKVIO
{
namespace DatasetManager
{
    class DatasetOfflineImp :public DatasetInterface{
    public:
        DatasetOfflineImp(const string& sDatasetPath);
        virtual TpDatasetType type(void) override {return TpOffline;};
        virtual Frame&      read(void) override;
        virtual bool        isFinished(void) override;
        virtual int         size(void);
        inline Frame&       CurrentFrame(void);
        const string&       getDatasetPath(void);
        virtual FrameInfo   getFrameInfor(const TpFrameID nFrmID);
    protected:
        virtual Frame*      load(const int nIndexToRead) = 0;
        inline void         initializeSize(const int size){ mSzDataset = size; }
        
        virtual inline const TpTimeStamp&   getFrameTimeStamp(TpFrameIndex nFrmIndex) = 0;
        virtual const string                getFrameFileName(TpFrameIndex nFrmIndex) = 0;
        virtual const string                getFrameAbsFileNmae(TpFrameIndex nFrmIndex, bool bTrueLeftFalseRight) = 0;
    private:
        Frame*              mPtrCurrentFrame;
        int                 nIndexNextToRead;
        int                 mSzDataset;
    private:
        const string        mStrDatasetPath;
    private:
        //void push(Frame* p){ mFrames.push(p); }
        //queue<Frame*> mFrames;
    };
}
}

#endif

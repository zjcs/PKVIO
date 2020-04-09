#ifndef __DATASETINTERFACE_H__
#define __DATASETINTERFACE_H__

#include "../Type/type.h"
#include <memory>

namespace PKVIO{
namespace DatasetManager{
    
    typedef enum {
        TpOnlie             = 0x00<<0,
        TpOffline           = 0x01<<0,
        TpOfflineEuRoc      = 0x01<<1,
        TpOfflineTUMRoom,
        TpOnelineT265,
        TpOnlineD435i
    } TpDatasetType;
    
    inline bool isOfflineDatasetType(TpDatasetType tp){return tp|TpOffline;}
    
    class DatasetInterface{
    public:
        typedef Type::IDGenerator<TpFrameID> FrameIDGenerator;
        virtual ~DatasetInterface() {};
        virtual TpDatasetType type(void) = 0;
        virtual void initialize(void) {};
        virtual Frame& read(void) = 0;
        virtual void exit(void) {};
        virtual bool isFinished(void){ return true; }
        
        virtual FrameInfo getFrameInfor(const TpFrameID nFrmID){ return FrameInfo(); }

        TpFrameID GeneratorFrameID(void){ return mIDGenerator.create(); }
        
        virtual const TpPtrCameraStereo     getPtrCamera(void)const = 0;
    private:
        FrameIDGenerator mIDGenerator;
    };

    typedef std::shared_ptr<DatasetInterface> DatasetInterfacePtr;

}
}
#endif

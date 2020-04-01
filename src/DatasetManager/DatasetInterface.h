#ifndef __DATASETINTERFACE_H__
#define __DATASETINTERFACE_H__

#include "../Type/type.h"
#include <memory>

namespace PKVIO{
namespace DatasetManager{
    
    template<typename TpID>
    class IDGenerator{
    public:
        IDGenerator(TpID nFirstIDToGenerate = 0): mNextIDToGenerate(nFirstIDToGenerate){}
        TpID create(void){ return mNextIDToGenerate++; }
    private:
        TpID mNextIDToGenerate;
    };
    
    class DatasetInterface{
    public:
        typedef IDGenerator<TpFrameID> FrameIDGenerator;
        virtual ~DatasetInterface() {};
        virtual void initialize(void) {};
        virtual Frame& read(void) = 0;
        virtual void exit(void) {};
        virtual bool isFinished(void){ return true; }

        TpFrameID GeneratorFrameID(void){ return mIDGenerator.create(); }
    private:
        FrameIDGenerator mIDGenerator;
    };

    typedef std::shared_ptr<DatasetInterface> DatasetInterfacePtr;

}
}
#endif

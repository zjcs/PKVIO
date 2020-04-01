#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "../DatasetManager/DatasetInterface.h"

using namespace std;

namespace PKVIO{
namespace System{
    
using namespace DatasetManager;

class System{
public:
    void exec(void);
protected:
    void initialize(void);
    void doexec(void);
    void exit(void);
private:
    DatasetManager::DatasetInterfacePtr mPtrDataset;
};

}
}

#endif

#ifndef __VERSION_H__
#define __VERSION_H__
#include <string>

using namespace std;

namespace PKVIO{
namespace Version{
    class Version{
    public:
        string str(void){
            return "0.0";
        }
    };
    
    string version(void);
}
}
#endif

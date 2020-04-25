#ifndef __VERSION_H__
#define __VERSION_H__
#include <string>

using namespace std;

namespace PKVIO{
namespace Version{
    class Version{
    public:
        string str(void){
            return "1.0";
        }
    };
    
    string version(void);
}
}
#endif

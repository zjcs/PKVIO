#include "type.h"
#include <stdio.h>

namespace PKVIO{
namespace Type{
    
string cvtTimeStampToString(const TpTimeStamp& t){
    char a[100];
    sprintf(a,"%19.0f",t);
    return string(a);
}

}
}

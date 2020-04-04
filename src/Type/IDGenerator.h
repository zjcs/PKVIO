#ifndef __IDGENERATOR_H__
#define __IDGENERATOR_H__

namespace PKVIO
{
namespace IDGenerator 
{
    
template<typename TpID>
class IDGenerator{
public:
    IDGenerator(TpID nFirstIDToGenerate = 0): mNextIDToGenerate(nFirstIDToGenerate){}
    TpID create(void){ return mNextIDToGenerate++; }
private:
    TpID mNextIDToGenerate;
};
       
}
}


#endif

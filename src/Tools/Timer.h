#ifndef __TIMER_H__
#define __TIMER_H__

#include <string>
#include <time.h>
#include <iostream>
#include <sstream>

using namespace std;

namespace PKVIO
{
namespace Timer 
{
    
class Timer
{
public:
    Timer(const string sStrName = "", bool bAutoLog = true) :mStrName(sStrName), mbAutoLog(bAutoLog) { mStartClock = clock(); }
    virtual                 ~Timer() { if(mbAutoLog) cout <<str()<< endl; }
    const string            str(void) { ostringstream sStrStream; sStrStream<<"Timer: " << mStrName << "  -- " << ms() << " ms." <<endl; return sStrStream.str(); }
    inline const string&    name(void)const{return mStrName;}
protected:
    float                   ms(void) { return (clock()-mStartClock)*1.0/CLOCKS_PER_SEC * 1000; }
private:
    const string            mStrName;
    const bool              mbAutoLog;
    clock_t                 mStartClock;
    
};
    
}
}

#endif

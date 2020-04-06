#ifndef __TIMER_H__
#define __TIMER_H__

#include <string>
#include <time.h>
#include <iostream>
#include <sstream>
#include <functional>
#include "../Type/type.h"

using namespace std;

namespace PKVIO
{
namespace Timer 
{
typedef std::function<void(void)> TpFuncTimerExec;
class Timer
{
public:
    Timer(const string sStrName = "", bool bAutoLog = true, TpTimeStamp* nPtrTimeCostSaveTo = nullptr) 
    :mStrName(sStrName), mbAutoLog(bAutoLog),mPtrTimeCostSaveTo(nPtrTimeCostSaveTo) { mStartClock = clock(); }
    Timer(TpFuncTimerExec mFunc,const string sStrName = "", bool bAutoLog = true, TpTimeStamp* nPtrTimeCostSaveTo = nullptr)
    : mStrName(sStrName), mbAutoLog(bAutoLog),mPtrTimeCostSaveTo(nPtrTimeCostSaveTo),mFuncTimerExecWhenExit(mFunc) { mStartClock = clock(); }
    
    virtual                 ~Timer() { 
                                if(mPtrTimeCostSaveTo)*mPtrTimeCostSaveTo=ms();
                                if(mFuncTimerExecWhenExit)mFuncTimerExecWhenExit();
                                if(mbAutoLog) cout <<str()<< endl;
                            }
    
    TpTimeStamp             ms(void) const { return (clock()-mStartClock)*1.0/CLOCKS_PER_SEC * 1000; }
    const string            str(void) const { 
                                ostringstream sStrStream; 
                                sStrStream<<"Timer: " << mStrName << "  -- " << ms() << " ms." <<endl;
                                return sStrStream.str();
                                
                            }
    inline const string&    name(void)const{return mStrName;}
    
protected:
private:
    const string            mStrName;
    const bool              mbAutoLog;
    clock_t                 mStartClock;
    TpTimeStamp*            mPtrTimeCostSaveTo;
    TpFuncTimerExec         mFuncTimerExecWhenExit;
    
};
    
}
}

#endif

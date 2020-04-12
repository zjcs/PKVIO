#ifndef __FIXLENGTHQUEUE_H__
#define __FIXLENGTHQUEUE_H__

namespace PKVIO
{

namespace Type
{

template<typename T>
class FixLengthQueue: public list<T>
{
public:
    FixLengthQueue(const int nMaxLength = 100):mMaxLength(nMaxLength){}
    void            push(const T& t) {
        auto nCurSz = (int)this->size();
        if(isFixedLength() && nCurSz>=mMaxLength){
            for(int nIdx=0,nPop=nCurSz+1-mMaxLength;nIdx<nPop;++nIdx){
                list<T>::pop_front();
            }
        }
        list<T>::push_back(t);
    }
    inline virtual bool     isFull(void){return isFixedLength() && mMaxLength>(int)this->size();}
protected:
    inline bool             isFixedLength(void) {return mMaxLength > 0;}
private:
    const int mMaxLength;       // if is lessequal than 0, means not fixed length and save all values.
};

}

}



#endif

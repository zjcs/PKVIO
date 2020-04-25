#ifndef __DATAHISTORYTEMPLATE_H__
#define __DATAHISTORYTEMPLATE_H__

#include "FixLengthQueue.h"
#include <iostream>

using namespace std;

namespace PKVIO
{
namespace Type
{
        
// // okay
//template<int n>
//class nTTest{
//    
//};
// // wrong, is not a valid type for a template non-type parameter.
//template<typename std::function<bool(int&,const int)> >
//class nTTest2{
//    
//};
//typedef template<typename TpID, typename T > std::function<bool(T&,const TpID)> TpFuncIDAndTMatch;
//template <typename TpID, typename T,  TpFuncIDAndTMatch f>
//template <typename TpID, typename T,  std::function<bool(T&,const TpID)> f>
template <typename T, typename TpID>
class DataHistoryTemplate{
public:
    typedef std::function<bool(T&,const TpID)>  TpFuncIsIDAndTMatch;
    typedef std::vector<T>                      TpVecTs;
    DataHistoryTemplate(TpFuncIsIDAndTMatch/*std::function<bool(T&,const TpID)>*/ fFuncIsIDAndTMatch)
    : mFuncIsIDAndTMatch(fFuncIsIDAndTMatch)
    {
        
    }
    
    //operator= (DescriptorHistory& op){}
    void            initDataSpecialFunction(std::function<bool(const T&)> nFunc){mFuncIsSpecial = nFunc;}
    
    void            push(const T& One){
                        mHistory.push(One);
                        //updateSpecial();  // the Special flag is setted after data, so the calling here always is not Special;
                    }
    void            updateSpecial(void){
                        if(mFuncIsSpecial&&mFuncIsSpecial(back())) {
                            mHistorySpecial.push(back());  
                            //cout << "Push Kf" <<endl;
                        } 
                    }
    bool            isExisting(const TpID nID){
                        for(auto Iter = mHistory.begin(), EndIter = mHistory.end(); Iter!=EndIter; ++Iter) {
                            T& h = *Iter;
                            if(mFuncIsIDAndTMatch(h, nID))
                                return true;
                        }
                        
                        for(auto Iter = mHistorySpecial.begin(), EndIter = mHistorySpecial.end(); Iter!=EndIter; ++Iter) {
                            T& h = *Iter;
                            if(mFuncIsIDAndTMatch(h, nID))
                                return true;
                        }
                        
                        return false;
                    }
    T&              get(const TpID nID){
                        for(auto Iter = mHistory.begin(), EndIter = mHistory.end(); Iter!=EndIter; ++Iter) {
                            T& h = *Iter;
                            if(mFuncIsIDAndTMatch(h, nID))
                                return h;
                        }
                        
                        for(auto Iter = mHistorySpecial.begin(), EndIter = mHistorySpecial.end(); Iter!=EndIter; ++Iter) {
                            T& h = *Iter;
                            if(mFuncIsIDAndTMatch(h, nID))
                                return h;
                        }
                        // not found, errro call; should call isExisting() before get();
                        throw;
                    }
    inline T&       back(void){ if(mHistory.empty())throw; return mHistory.back();}
    inline T&       getLastOne(void){return back();}
    
    // nSz: -1 means no limition.
    TpVecTs         getLastSeveral(int nSz, std::function<bool(T&)> bFuncIsThisToBeExcluded = [](T&)->bool{return false;}){
                        TpVecTs vTs;
                        
                        if(nSz>0) 
                            vTs.reserve(nSz);
                        
                        for(auto iter = mHistorySpecial.rbegin(), EndIter = mHistorySpecial.rend(); iter!=EndIter && (nSz <0 || vTs.size() < nSz); ++iter) {
                            vTs.push_back(*iter);
                        }
                        // free the memory of last NotUsed place.
                        vTs.resize(vTs.size());
                        return vTs;
                        
                        
                        for(auto iter = mHistory.rbegin(), EndIter = mHistory.rend(); iter!=EndIter && vTs.size() < nSz; ++iter) {
                            if(bFuncIsThisToBeExcluded(*iter))
                                continue;
                            vTs.push_back(*iter);
                        }
                        // free the memory of last NotUsed place.
                        vTs.resize(vTs.size());
                        return vTs;
                        
                        auto LastIter = vTs.begin();
                        std::advance(LastIter, vTs.size());
                        vTs.erase(LastIter, vTs.end());
                        // Error as below;
                        //vTs.erase(std::advance(vTs.begin(), vTs.size()), vTs.end());
                        return vTs;
                                            
                    }
    bool            empty(void){return !size();}
    int             size(void){return (int)mHistory.size();}
protected:
   Type::FixLengthQueue<T>  mHistory;
   Type::FixLengthQueue<T>  mHistorySpecial;
   
   std::function<bool(const T&)>  mFuncIsSpecial;
private:
   TpFuncIsIDAndTMatch      mFuncIsIDAndTMatch;
};

}
}

#endif

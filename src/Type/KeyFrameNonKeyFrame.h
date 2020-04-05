#ifndef __KEYFRAMENONKEYFRAME_H__
#define __KEYFRAMENONKEYFRAME_H__

#if 1
namespace PKVIO
{
namespace Type
{
    
typedef enum{
    EnKeyFrame,
    EnNonKeyFrame
}EnKeyNonKeyFrame;

    
class BasicKeyNonKeyFrame
{
public:
    virtual                     ~BasicKeyNonKeyFrame(){}
    virtual EnKeyNonKeyFrame    type(void) = 0;
};
   
class NonKeyFrame:public BasicKeyNonKeyFrame
{
public:
    virtual EnKeyNonKeyFrame    type(void) override {return EnNonKeyFrame; }
    
};

class KeyFrame:public BasicKeyNonKeyFrame
{
public:
    virtual EnKeyNonKeyFrame    type(void) override {return EnKeyFrame;}
};

}
}


#endif
#endif

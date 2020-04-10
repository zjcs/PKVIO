#ifndef __VIEWER_H__
#define __VIEWER_H__

#include <QApplication>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpacerItem>
#include <QTimer>
#include <QPaintEvent>

#include "../System/System.h"

namespace PKVIO
{
namespace Viewer
{
    
    
class ImageWidget: public QWidget{
public:
    void setImage(const cv::Mat& nImgBk);
protected:
    virtual void paintEvent(QPaintEvent* e) override;
    const cv::Mat& getImage(void){return mImgBk;}
    QImage         getQImage(void);
private:
    cv::Mat mImgBk;
};

class PKVIOMainWindow: public QMainWindow{
public:
    PKVIOMainWindow();
private:
    void initUi(void);
    
    
    void runVIO(void);
    
    void initVIO(void);
private:
    PKVIO::System::System   mVioSystem;
    ImageWidget*            mPtrFrameImageWgt;
};

}
}

#endif

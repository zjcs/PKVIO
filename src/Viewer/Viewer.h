#ifndef __VIEWER_H__
#define __VIEWER_H__

#include <QApplication>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QSpacerItem>
#include <QTimer>
#include <QPaintEvent>
#include <qglviewer.h>

#include "../System/System.h"


namespace PKVIO
{
namespace Viewer
{
    
/**
* \brief OpenGL based viewer for the graph
*/
class G2oQGLViewer : public QGLViewer
{
public:
    G2oQGLViewer(QWidget* parent=NULL, const QGLWidget* shareWidget=0, Qt::WindowFlags flags=0);
    ~G2oQGLViewer();
    void draw();
    void init();

    /**
    * the viewer uses a display list to cache the drawing, use setUpdateDisplay() to force
    * the creation of an updated display list.
    */
    bool updateDisplay() const { return _updateDisplay;}
    void setUpdateDisplay(bool updateDisplay);

    virtual void dodraw(void){}
public:

protected:
    GLuint _drawList;
    bool _updateDisplay;
};
    
class CameraPoseGLViewer: public G2oQGLViewer{
public:
    void addCameraPose(cv::Vec3f p){ mVecPose.push_back(p); setUpdateDisplay(true);}
    virtual void dodraw(void) override;
    void         clear(void){mVecPose.clear(); setUpdateDisplay(true);}
private:
    std::vector<cv::Vec3f> mVecPose;
};

class ImageWidget: public QWidget{
public:
    void setImage(const cv::Mat& nImgBk);
    void resetImage(void){setImage(cv::Mat());}
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
    
    void newVIO(void);
    
    void clear(void);
private:
    PKVIO::System::TpPtrVIOSystem   mPtrVioSystem;
    ImageWidget*                    mPtrFrameImageWgt;
    CameraPoseGLViewer*             mPtrGLViewer;
    QTimer*                         mPtrTimerVIO;
    QPushButton* pBtnStart;
    QPushButton* pBtnContinue;
    QPushButton* pBtnStop;
    QPushButton* pBtnClose; 
    QCheckBox*   pCBXSimulator;
};

}
}

#endif

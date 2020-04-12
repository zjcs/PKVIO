#include "Viewer.h"
#include <QPainter>
#include "../System/Version.h"

// some macro helpers for identifying the version number of QGLViewer
// QGLViewer changed some parts of its API in version 2.6.
// The following preprocessor hack accounts for this. THIS SUCKS!!!
#if (((QGLVIEWER_VERSION & 0xff0000) >> 16) >= 2 && ((QGLVIEWER_VERSION & 0x00ff00) >> 8) >= 6)
#define qglv_real qreal
#else
#define qglv_real float
#endif

// Again, some API changes in QGLViewer which produce annoying text in the console
// if the old API is used.
#if (((QGLVIEWER_VERSION & 0xff0000) >> 16) >= 2 && ((QGLVIEWER_VERSION & 0x00ff00) >> 8) >= 5)
#define QGLVIEWER_DEPRECATED_MOUSEBINDING
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif


namespace PKVIO
{
namespace Viewer
{
    
// cannot be a statistic variable, why?
//PKVIO::System::System mPtrVioSystem;
    
/**
* \brief helper for setting up a camera for qglviewer
*/
class StandardCamera : public qglviewer::Camera
{
public:
    StandardCamera() : _standard(true) {};

    qglv_real zNear() const {
    if (_standard) 
        return qglv_real(0.001);
    else 
        return Camera::zNear(); 
    }

    qglv_real zFar() const
    {  
    if (_standard) 
        return qglv_real(10000.0);
    else 
        return Camera::zFar();
    }

    bool standard() const {return _standard;}
    void setStandard(bool s) { _standard = s;}

private:
    bool _standard;
};


G2oQGLViewer::G2oQGLViewer(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags flags) :
  QGLViewer(parent, shareWidget, flags)
{
  setAxisIsDrawn(false);
}

G2oQGLViewer::~G2oQGLViewer()
{
  glDeleteLists(_drawList, 1);
}

void G2oQGLViewer::draw()
{
  if (_updateDisplay) {
    _updateDisplay = false;
    glNewList(_drawList, GL_COMPILE_AND_EXECUTE);
    dodraw();
    glEndList();
  } else {
    glCallList(_drawList); 
  }
}

void G2oQGLViewer::init()
{
  QGLViewer::init();
  //glDisable(GL_LIGHT0);
 //glDisable(GL_LIGHTING);

  setBackgroundColor(QColor::fromRgb(51, 51, 51));

  // some default settings i like
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND); 
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  //glEnable(GL_CULL_FACE);
  glShadeModel(GL_FLAT);
  //glShadeModel(GL_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setAxisIsDrawn();

  // don't save state
  setStateFileName(QString::null);

  // mouse bindings
#ifdef QGLVIEWER_DEPRECATED_MOUSEBINDING
  setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, TRANSLATE);
  setMouseBinding(Qt::NoModifier, Qt::MidButton, CAMERA, TRANSLATE);
#else
  setMouseBinding(Qt::RightButton, CAMERA, TRANSLATE);
  setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);
#endif

  // keyboard shortcuts
  setShortcut(CAMERA_MODE, 0);
  setShortcut(EXIT_VIEWER, 0);
  //setShortcut(SAVE_SCREENSHOT, 0);

  // replace camera
  qglviewer::Camera* oldcam = camera();
  qglviewer::Camera* cam = new StandardCamera();
  setCamera(cam);
  cam->setPosition(qglviewer::Vec(0., 0., 75.));
  cam->setUpVector(qglviewer::Vec(0., 1., 0.));
  cam->lookAt(qglviewer::Vec(0., 0., 0.));
  delete oldcam;

  // getting a display list
  _drawList = glGenLists(1);
}

void G2oQGLViewer::setUpdateDisplay(bool updateDisplay)
{
  _updateDisplay = updateDisplay;
  update();
}

    

void PKVIOMainWindow::initUi() {
    QWidget* pWgtMainWin =  new QWidget;
    this->setCentralWidget(pWgtMainWin);
    QHBoxLayout* pHBLMainWin = new QHBoxLayout;
    pWgtMainWin->setLayout(pHBLMainWin);

    mPtrFrameImageWgt   = new ImageWidget;
    mPtrGLViewer        = new CameraPoseGLViewer;
    QWidget* pWgtCtr    = new QWidget;

    pHBLMainWin->addWidget(mPtrFrameImageWgt);
    pHBLMainWin->addWidget(mPtrGLViewer);
    pHBLMainWin->addWidget(pWgtCtr);
    pHBLMainWin->setStretch(0, 1);
    pHBLMainWin->setStretch(1, 2);
    pHBLMainWin->setStretch(2, 1);

    QVBoxLayout* pVBLControl = new QVBoxLayout;
    pWgtCtr->setLayout(pVBLControl);
    pBtnStart     = new QPushButton("Start");
    pBtnContinue  = new QPushButton("Continue");
    pBtnStop      = new QPushButton("Stop");
    pBtnClose     = new QPushButton("Close");
    pCBXSimulator = new QCheckBox("Simulator");
    pVBLControl->addWidget(pBtnStart);
    pVBLControl->addWidget(pBtnContinue);
    pVBLControl->addWidget(pBtnStop);
    pVBLControl->addWidget(pBtnClose);
    pVBLControl->addWidget(pCBXSimulator);
    pVBLControl->addSpacerItem(new QSpacerItem(0,0,QSizePolicy::Minimum, QSizePolicy::Expanding));

    pWgtCtr->setFixedWidth(100);
    //this->setMinimumSize(1000, 600);
    
    pBtnContinue->setVisible(false);
    pBtnStop->setEnabled(false);
    pBtnClose->setEnabled(false);

    mPtrTimerVIO = new QTimer(this);
    connect(mPtrTimerVIO, &QTimer::timeout, this, [&]() {
        if(!mPtrVioSystem)
            return;
        mPtrVioSystem->doexec();
        mPtrFrameImageWgt->setImage(mPtrVioSystem->getDispalyImage());
        mPtrGLViewer->addCameraPose(mPtrVioSystem->getCameraPoseCurFrame());
        //cout << "1000ms" <<endl;
    });
    
    connect(pBtnStart, &QPushButton::clicked, this, [&](){
        newVIO();
    });
    connect(pBtnStop, &QPushButton::clicked, this, [&](){
        pBtnContinue->setVisible(true);
        pBtnStop->setVisible(false);
        mPtrTimerVIO->stop();
    });
    connect(pBtnContinue, &QPushButton::clicked, this, [&](){
        pBtnContinue->setVisible(false);
        pBtnStop->setVisible(true);
        mPtrTimerVIO->start();
    });
    connect(pBtnClose, &QPushButton::clicked, this, [&](){
        pBtnContinue->setVisible(false);
        pBtnStop->setVisible(true);
        pBtnStop->setEnabled(false);
        pBtnClose->setEnabled(false);
        mPtrTimerVIO->stop();
        mPtrVioSystem.reset();
        mPtrGLViewer->clear();
        mPtrFrameImageWgt->resetImage();
        mPtrGLViewer->update();
    });
    
    connect(pCBXSimulator, &QCheckBox::clicked, this, [&](){
        if(!mPtrVioSystem)return;
        newVIO();
    });
    
    /*
    */
    
    mPtrFrameImageWgt->setStyleSheet("background-color:#0000ff");
    //pWgtMap->setStyleSheet("background-color:#ff0000");
    //mPtrGLViewer->setFixedSize();
    
    //this->update();
    pBtnStart->click();
}


void PKVIOMainWindow::runVIO() {
    cout << "Hello, This is PKVIO~" << endl;
    cout << "Version Infor: " << PKVIO::Version::version() << endl;
    auto VioSystem = PKVIO::System::System();
    VioSystem.runVIO();
}


void PKVIOMainWindow::initVIO() {
    cout << "Hello, This is PKVIO~" << endl;
    cout << "Version Infor: " << PKVIO::Version::version() << endl;
    
    if(mPtrVioSystem){
        mPtrVioSystem.reset();
    }
    mPtrVioSystem = PKVIO::System::generateVIOSystem();
    
    mPtrVioSystem->initialize(pCBXSimulator->isChecked());
    mPtrVioSystem->setRunVIO(false);
}


PKVIOMainWindow::PKVIOMainWindow() 
: QMainWindow()
{
    initUi();
}


void ImageWidget::setImage(const cv::Mat& nImgBk) {
    mImgBk = nImgBk.clone();
    // request widget redraw.
    this->update();
}

cv::Mat qim2mat(QImage & qim)
{
    cv::Mat mat = cv::Mat(qim.height(), qim.width(),CV_8UC3,(void*)qim.constBits(),qim.bytesPerLine());
    return mat;
} 

QImage mat2qim(const cv::Mat & mat, int nWidth=0, int nHeight = 0) 
{
    auto nMatTp = mat.type();
    assert(nMatTp == CV_8UC1 || nMatTp == CV_8UC3);
    
    if(nWidth==0){ nWidth = mat.cols? mat.cols: 100; }
    if(nHeight==0){ nHeight = mat.rows? mat.rows: 100; }
    
    cv::Mat nImgRGB;
    if(mat.empty()){
        nImgRGB = cv::Mat(nWidth, nHeight, CV_8UC3, cv::Scalar(0));
    }else{
        cv::cvtColor(mat, nImgRGB, nMatTp == CV_8UC1? cv::COLOR_GRAY2BGR: cv::COLOR_RGB2BGR);
        if(nImgRGB.size() != cv::Size(nWidth, nHeight)){
            cv::resize(nImgRGB, nImgRGB, cv::Size(nWidth, nHeight));
        }
    }
    
    QImage qim((const unsigned char*)nImgRGB.data, nImgRGB.cols, nImgRGB.rows, nImgRGB.step, QImage::Format_RGB888);
    return qim;
}

void PKVIOMainWindow::newVIO() {
    pBtnContinue->setVisible(false);
    pBtnStop->setVisible(true);
    pBtnStop->setEnabled(true);
    pBtnClose->setEnabled(true);
    initVIO();
    mPtrTimerVIO->setInterval(10);
    mPtrTimerVIO->start();
}


QImage ImageWidget::getQImage() {
    return mat2qim(getImage(), this->width(), this->height());
}




void ImageWidget::paintEvent(QPaintEvent* e) {
    QPainter nPainter(this);
    QRect    nRect(0,0,this->width(),this->height());
    QImage   nImage = getQImage();
    //return QWidget::paintEvent(e);
    nPainter.drawImage(nRect, nImage);
}

void CameraPoseGLViewer::dodraw() {
    size_t nSzCameraPose = mVecPose.size();
    if(!nSzCameraPose)
        return;
    setAxisIsDrawn(nSzCameraPose%20<10);
    
    glPushMatrix();
    float mPointSize = 20;
    glPointSize(mPointSize);
    //glBegin(GL_POINTS);
    glBegin(GL_LINE_STRIP);
    // R,G,B
    glColor3f(1.0,1.0,0.0); // yellow

    for(size_t i=0, iend=nSzCameraPose; i<iend;i++)
    {
        auto p = mVecPose[i];
        if(i>=iend-2){
            glColor3f(1.0,0.0,0.0);
        }
        glVertex3f(p(0),p(1),p(2));
    }
    
    glEnd();
    glPopMatrix();
    
    /*
    glBegin(GL_LINE_LOOP);
    glVertex3f(0,0,0);
    glVertex3f(100,0,0);
    glVertex3f(0,100,0);
    glVertex3f(100,100,100);
    glVertex3f(0,0,100);
    */
}


}
}



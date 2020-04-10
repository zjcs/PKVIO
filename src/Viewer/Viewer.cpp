#include "Viewer.h"
#include <QPainter>
#include "../System/Version.h"


namespace PKVIO
{
namespace Viewer
{
    
// cannot be a statistic variable, why?
//PKVIO::System::System mVioSystem;
    
    

void PKVIOMainWindow::initUi() {
    QWidget* pWgtMainWin =  new QWidget;
    QHBoxLayout* pHBLMainWin = new QHBoxLayout;
    pWgtMainWin->setLayout(pHBLMainWin);

    mPtrFrameImageWgt = new ImageWidget;
    QWidget* pWgtMap = new QWidget;
    QWidget* pWgtCtr = new QWidget;

    pHBLMainWin->addWidget(mPtrFrameImageWgt);
    pHBLMainWin->addWidget(pWgtMap);
    pHBLMainWin->addWidget(pWgtCtr);

    QVBoxLayout* pVBLControl = new QVBoxLayout;
    pWgtCtr->setLayout(pVBLControl);
    QPushButton* pStart = new QPushButton("Start");
    QPushButton* pEnd = new QPushButton("End");
    pVBLControl->addWidget(pStart);
    pVBLControl->addWidget(pEnd);
    pVBLControl->addSpacerItem(new QSpacerItem(0,0,QSizePolicy::Minimum, QSizePolicy::Expanding));

    this->setCentralWidget(pWgtMainWin);


    pWgtCtr->setFixedWidth(100);
    this->setFixedSize(1000, 600);

    QTimer* pTimerVIO = new QTimer(this);
    connect(pTimerVIO, &QTimer::timeout, this, [&]() {
        mVioSystem.doexec();
        mPtrFrameImageWgt->setImage(mVioSystem.getDispalyImage());
        //cout << "1000ms" <<endl;
    } );
    pTimerVIO->start(10);
    
    mPtrFrameImageWgt->setStyleSheet("background-color:#0000ff");
    pWgtMap->setStyleSheet("background-color:#ff0000");
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
    mVioSystem.initialize();
    mVioSystem.setRunVIO(false);
}


PKVIOMainWindow::PKVIOMainWindow() 
: QMainWindow()
{
    initUi();
    initVIO();
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

}
}


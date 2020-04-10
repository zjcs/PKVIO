#include <iostream>
#include <string>
#include <QApplication>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpacerItem>
#include <QTimer>
#include "System/Version.h"
#include "System/System.h"
#include "Viewer/Viewer.h"

using namespace std;
/*
class ImageWidget: public QWidget{
public:
private:
    
};

class PKVIOMainWindow: public QMainWindow{
public:
    PKVIOMainWindow(): QMainWindow(){
        initUi();
        initVIO();
    }
private:
    void initUi(void){
        QWidget* pWgtMainWin =  new QWidget;
        QHBoxLayout* pHBLMainWin = new QHBoxLayout;
        pWgtMainWin->setLayout(pHBLMainWin);
        
        QWidget* pWgtImg = new QWidget;
        QWidget* pWgtMap = new QWidget;
        QWidget* pWgtCtr = new QWidget;
        
        pHBLMainWin->addWidget(pWgtImg);
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
        connect(pTimerVIO, &QTimer::timeout, this, [&](){
            mVioSystem.doexec();
            //cout << "1000ms" <<endl;
        } );
        pTimerVIO->start(10);
    }
    
    
    void runVIO(void){
        cout << "Hello, This is PKVIO~" << endl;
        cout << "Version Infor: " << PKVIO::Version::version() << endl;
        auto VioSystem = PKVIO::System::System();
        VioSystem.runVIO();
    }
    
    void initVIO(void){
        cout << "Hello, This is PKVIO~" << endl;
        cout << "Version Infor: " << PKVIO::Version::version() << endl;
        mVioSystem.initialize();
        mVioSystem.setRunVIO(true);
    }
private:
    PKVIO::System::System mVioSystem;
};
*/
int main(int argc, char** argv)
{
    QApplication qapp(argc, argv);
    
    //PKVIOMainWindow mWin;
    PKVIO::Viewer::PKVIOMainWindow mWin;
    mWin.show();
    qapp.exec();
    return 0;
}



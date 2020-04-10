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
void runVIO(void){
    cout << "Hello, This is PKVIO~" << endl;
    cout << "Version Infor: " << PKVIO::Version::version() << endl;
    auto VioSystem = PKVIO::System::System();
    VioSystem.runVIO();
}
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



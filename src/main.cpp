#include <iostream>
#include <string>
using namespace std;

#include "System/Version.h"
#include "System/System.h"

int main(int argc, char** argv)
{
    cout << "Hello, This is PKVIO~" << endl;
    cout << "Version Infor: " << PKVIO::Version::version() << endl;
    
    auto VioSystem = PKVIO::System::System();
    VioSystem.runVIO();

#ifdef _WINDOWS_
	system("pause");
#endif
    return 0;
}


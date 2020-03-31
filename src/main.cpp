#include <iostream>
#include <string>
using namespace std;

#include "System/Version.h"

int main(char argc, char** argv)
{
    cout << "Hello, This is PKVIO~" << endl;
    cout << "Version Infor: " << PKVIO::Version::version() << endl;
	system("pause");
    return 0;
}


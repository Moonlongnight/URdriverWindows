#include <cstdio>
#include "controlReal.h"
#include "getLineNumber.h"
#include "automatic_pieceWise.h"
#include "linearWithParabola.h"
#include"LSPB.h"
#include"automatic_getIKsolution.h"
#include"automatic_interpolate_p.h"
#include"movel_Test.h"
int main()
{
	WSADATA wsaData;
	if (WSAStartup(0x0202, &wsaData) != 0) {// Initialize Windows socket library
		std::cout << "fail to load winsock" << endl;
		exit(0);
	}
	controlReal cr;

	cr.openworkWindows();

    return 0;
}
#pragma once
/*
Function:  检查文件是否正常打开
i
o
m
*/
#define checkfp(fp)\
if (fp.fail()) {\
	cout << "ERROR IN checkfp _" << __LINE__ << ": fail to open file" << endl;\
	system("pause");\
	exit(0);\
}
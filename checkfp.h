#pragma once
/*
Function:  ����ļ��Ƿ�������
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
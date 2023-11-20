/*
记录了：
·常量
·矩阵的基本函数：打印和创建
·判断是否近似为0的函数
·计算向量夹角的函数
*/
#pragma once
#include<iostream>
#include<math.h>
#include<Eigen/Dense>
#include "URPose.h"
//using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;

#define PI 3.14159265358979
#define mm2m 0.001
#define m2mm 1000
#define m2um 1000000
#define d2r(x) (x/360.0*2*PI)
#define r2d(x) (x/(2*PI)*360.0)
const double ZERO_THRESH = 0.00001;
/*
Function:  init a square matrixXd
i
o
m
*/
#define initM(m,num)\
for (int i = 0; i < m.rows(); i++) {\
	for (int j = 0; j < m.cols(); j++) {\
	m(i, j) = num;\
	}\
}
/*
Function:  print a square matrixXd
i
o
m
*/
#define printM(m)\
for (int i = 0; i < m.rows(); i++) {\
	for (int j = 0; j < m.cols(); j++) {\
		printf("%12.6f",m(i,j));\
	}\
	printf("\n");\
}

#define CheckZeroThreshAndSet(m)\
for (int i = 0; i < m.rows(); i++) {\
	for (int j = 0; j < m.cols(); j++) {\
		if(abs(m(i,j))<ZERO_THRESH)\
			m(i,j)=0;\
	}\
}

/*
Function:  if a is zeroThresh return 1
i
o
m
*/
#define CheckZeroThresh(a)\
	(abs(a)<ZERO_THRESH)

#define print44M_Array(m)\
	for(int i=0;i<4;i++){\
		for (int j = 0; j < 4; j++) {\
			printf("%12.6f",m[i*4+j]);\
		}\
		printf("\n");\
	}


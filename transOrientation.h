#pragma once
#include<iostream>
#include<math.h>
#include<Eigen/Dense>
#include"RotateVector.h"
#include"Quaternion.h"
#include"trans.h"
using Eigen::MatrixXd;
using namespace std;
/*
Function:  此.h用于对姿态描述的转换
*/

/*
Function:  将四元数转旋转矩阵
i
o
m
*/
MatrixXd transQuaternionToRotateMatrix(const Quaternion& q) {
	MatrixXd m(3, 3);

	m(0, 0) = 1 - 2 * q.c * q.c - 2 * q.d * q.d;
	m(0, 1) = 2 * q.b * q.c - 2 * q.a * q.d;
	m(0, 2) = 2 * q.a * q.c + 2 * q.b * q.d;

	m(1, 0) = 2 * q.b * q.c + 2 * q.a * q.d;
	m(1, 1) = 1 - 2 * q.b * q.b - 2 * q.d * q.d;
	m(1, 2) = 2 * q.c * q.d - 2 * q.a * q.b;

	m(2, 0) = 2 * q.b * q.d - 2 * q.a * q.c;
	m(2, 1) = 2 * q.a * q.b + 2 * q.c * q.d;
	m(2, 2) = 1 - 2 * q.b * q.b - 2 * q.c * q.c;

	CheckZeroThreshAndSet(m);

	return m;
}

/*
Function:  将旋转矩阵转四元数
i
o
m
*/
Quaternion transRotateMatrxToQuaternion(const MatrixXd& m) {
	Quaternion q;
	double zeroHold = 0.1;//若a<zeroHold则表示a趋向0。
	q.a = sqrt(1 + m(0, 0) + m(1, 1) + m(2, 2)) / 2;
	if (q.a < zeroHold) {//a趋向0
		double t;//temp
		cout << m << endl;
		if (m(0, 0) >= m(1, 1) && m(0, 0) >= m(2, 2)) {//b必不趋向于0
			t = sqrt(1 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;//4b
			q.a = (m(2, 1) - m(1, 2)) / t;
			q.b = t / 4;
			q.c = (m(0, 2) + m(2, 0)) / t;
			q.d = (m(0, 1) + m(1, 0)) / t;
		}
		else if (m(1, 1) >= m(0, 0) && m(1, 1) >= m(2, 2)) {//c必不趋向于0
			t = sqrt(1 - m(0, 0) + m(1, 1) - m(2, 2)) * 2;//4c
			q.a = (m(0, 2) - m(2, 0)) / t;
			q.b = (m(0, 1) + m(1, 0)) / t;
			q.c = t / 4;
			q.d = (m(2, 1) + m(1, 2)) / t;
		}
		else {//d必不趋向于0
			t = sqrt(1 - m(0, 0) - m(1, 1) + m(2, 2)) * 2;//4d
			q.a = (m(1, 0) - m(0, 1)) / t;
			q.b = (m(0, 2) + m(2, 0)) / t;
			q.c = (m(1, 2) - m(2, 1)) / t;
			q.d = t / 4;
		}
	}
	else {//q不趋向于0
		q.a = q.a;
		q.b = (m(2, 1) - m(1, 2)) / (4 * q.a);
		q.c = (m(0, 2) - m(2, 0)) / (4 * q.a);
		q.d = (m(1, 0) - m(0, 1)) / (4 * q.a);
	}
	return q;
}
/*
Function:  将四元数转旋转矢量
i
o
m:
根据双倍覆盖，一个四元数有两个旋转矢量对应。由于旋转的等效性再结合旋转矢量的角度是模长（正数），我们取sita>0的对应结果
结合cos(sita)的图像是对称的，可以直接取acos(value)的绝对值即可--若acos值为负数，转正后，对应sin会变符号，这样u会变反方向，因此旋转还是等效的。
*/
RotateVector transQuaternionToRotateVector(const Quaternion& q) {
	RotateVector rv;

	double magnitude = 2 * abs(acos(q.a));
	rv.Rx = q.b / sin(magnitude / 2) * magnitude;
	rv.Ry = q.c / sin(magnitude / 2) * magnitude;
	rv.Rz = q.d / sin(magnitude / 2) * magnitude;

	return rv;
}
/*
Function:  将旋转矢量转四元数
i
o
m
*/
Quaternion transRotateVectorToQuaternion(const RotateVector& rv) {
	Quaternion q;

	double magnitude = sqrt(rv.Rx * rv.Rx + rv.Ry * rv.Ry + rv.Rz * rv.Rz);
	q.a = cos(magnitude / 2);
	q.b = sin(magnitude / 2) * (rv.Rx / magnitude);
	q.c = sin(magnitude / 2) * (rv.Ry / magnitude);
	q.d = sin(magnitude / 2) * (rv.Rz / magnitude);

	return q;
}


/*
Function:  trans X-Y-Z rotate into 3X3 Matrix
i
o
m
*/
MatrixXd transXYZRotate(double gamma, double beta, double alpha) {
	MatrixXd m(3, 3);
	double sAlpha = sin(alpha);
	double cAlpha = cos(alpha);
	double sBeta = sin(beta);
	double cBeta = cos(beta);
	double sGamma = sin(gamma);
	double cGamma = cos(gamma);

	m(0, 0) = cAlpha * cBeta;
	m(0, 1) = cAlpha * sBeta * sGamma - sAlpha * cGamma;
	m(0, 2) = cAlpha * sBeta * cGamma + sAlpha * sGamma;

	m(1, 0) = sAlpha * cBeta;
	m(1, 1) = sAlpha * sBeta * sGamma + cAlpha * cGamma;
	m(1, 2) = sAlpha * sBeta * cGamma - cAlpha * sGamma;

	m(2, 0) = -sBeta;
	m(2, 1) = cBeta * sGamma;
	m(2, 2) = cBeta * cGamma;

	CheckZeroThreshAndSet(m);

	return m;
}
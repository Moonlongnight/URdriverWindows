#pragma once
#include<iostream>
#include<Eigen/Dense>
#include<vector>
#include<math.h>
#include"trans.h"
#include "transOrientation.h"
#include "URPose.h"
using Eigen::MatrixXd;//Eigen是一个namespace，MatrixXd是一个类
using namespace std;

//仿真：理论
const double d1 = 0.1519;
const double a2 = 0.24365;
const double a3 = 0.21325;
const double d4 = 0.11235;
const double d5 = 0.08535;
const double d6 = 0.0819;

//实际机械臂
//const double d1 = 0.153612;
//const double a2 = 0.244732;
//const double a3 = 0.2125635;
//const double d4 = 0.11308875;
//const double d5 = 0.08526945;
//const double d6 = 0.082368675;

//const double d1 = 0.153618;
//const double a2 = 0.244754;
//const double a3 = 0.212545;
//const double d4 = 0.113072;
//const double d5 = 0.0852653;
//const double d6 = 0.0823637;


MatrixXd FK(vector<double> angles) {
	MatrixXd result(4, 4);
	double C1 = cos(angles[0]);
	double C2 = cos(angles[1]);
	double C3 = cos(angles[2]);
	double C4 = cos(angles[3]);
	double C5 = cos(angles[4]);
	double C6 = cos(angles[5]);

	double S1 = sin(angles[0]);
	double S2 = sin(angles[1]);
	double S3 = sin(angles[2]);
	double S4 = sin(angles[3]);
	double S5 = sin(angles[4]);
	double S6 = sin(angles[5]);

	double C234 = cos(angles[1] + angles[2] + angles[3]);
	double S234 = sin(angles[1] + angles[2] + angles[3]);
	double C23 = cos(angles[1] + angles[2]);
	double S23 = sin(angles[1] + angles[2]);

	result(0, 0) = C6 * (S1 * S5 + C234 * C1 * C5) - S234 * C1 * S6;
	result(0, 1) = -S6 * (S1 * S5 + C234 * C1 * C5) - S234 * C1 * C6;
	result(0, 2) = C5 * S1 - C234 * C1 * S5;
	result(0, 3) = d6 * (C5 * S1 - C234 * C1 * S5) + d4 * S1 - a3 * C23 * C1 - a2 * C1 * C2 + d5 * S234 * C1;

	result(1, 0) = -C6 * (C1 * S5 - C234 * C5 * S1) - S234 * S1 * S6;
	result(1, 1) = S6 * (C1 * S5 - C234 * C5 * S1) - S234 * C6 * S1;
	result(1, 2) = -C1 * C5 - C234 * S1 * S5;
	result(1, 3) = d5 * S234 * S1 - d4 * C1 - a3 * C23 * S1 - a2 * C2 * S1 - d6 * (C1 * C5 + C234 * S1 * S5);

	result(2, 0) = C234 * S6 + S234 * C5 * C6;
	result(2, 1) = C234 * C6 - S234 * C5 * S6;
	result(2, 2) = -S234 * S5;
	result(2, 3) = d1 - d5 * (C23 * C4 - S23 * S4) - a3 * S23 - a2 * S2 - d6 * S5 * (C23 * S4 + S23 * C4);

	result(3, 0) = 0;
	result(3, 1) = 0;
	result(3, 2) = 0;
	result(3, 3) = 1;

	CheckZeroThreshAndSet(result);

	double temp = result(0, 3)*1000;
	temp = result(1, 3)*1000;
	temp = result(2, 3)*1000;
	return result;
}

/*
Function:  IK:以米和弧度为单位
i
o
m
*/
vector<vector<double>> IK(URPose &target,double theta6) {
	vector<vector<double>> sol;
	sol.resize(8);
	for (int i = 0; i < 8; i++) {
		sol[i].resize(6);
		for (int j = 0; j < 6; j++) {
			sol[i][j] = 0;
		}
	}
	/*sita*/
	double sita1[2];
	double sita5[4];
	double sita6[4];
	double sita234[4];
	double sita3[8];
	double sita2[8];
	double sita4[8];
	/*temp*/
	double a, b, c;
	double t1, t2;
	/*matrix*/
	RotateVector rv_trans(target.rx, target.ry, target.rz);
	MatrixXd m_trans(3, 3);
	m_trans = transQuaternionToRotateMatrix(transRotateVectorToQuaternion(rv_trans));
	
	MatrixXd m(4, 4);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			m(i, j) = m_trans(i, j);
		}
	}

	m(0, 3) = target.x;
	m(1, 3) = target.y;
	m(2, 3) = target.z;
	m(3, 0) = 0.0;
	m(3, 1) = 0.0;
	m(3, 2) = 0.0;
	m(3, 3) = 1.0;

	/*Tknown*/
	double r11 = m(0, 0);
	double r12 = m(0, 1);
	double r13 = m(0, 2);
	double px = m(0, 3);

	double r21 = m(1, 0);
	double r22 = m(1, 1);
	double r23 = m(1, 2);
	double py = m(1, 3);

	double r31 = m(2, 0);
	double r32 = m(2, 1);
	double r33 = m(2, 2);
	double pz = m(2, 3);

	/*sita1*/
	a = px - d6 * r13;
	b = py - d6 * r23;
	if (CheckZeroThresh(a) && CheckZeroThresh(b)) {
		cout << "ERROR IN IK of sita1 : CheckZeroThresh(a) && CheckZeroThresh(b) 无法通过当前方法解得sita1。" << endl;
		system("pause");
		exit(0);
	}
	c = -d4;

	sita1[0] = atan2(b, a) - atan2(c, sqrt(a * a + b * b - c * c));
	sita1[1] = atan2(b, a) - atan2(c, -sqrt(a * a + b * b - c * c));
	for (int i = 0; i < 4; i++) {
		sol[i][0] = sita1[0];
	}
	for (int i = 4; i < 8; i++) {
		sol[i][0] = sita1[1];
	}
	/*sita5*/
		/*sita1[0]*/
	double c5;
	double s5;
	c5 = sin(sita1[0]) * r13 - cos(sita1[0]) * r23;
	if (CheckZeroThresh(fabs(c5 - 1))) {
		sita5[0] = 0;
		sita5[1] = 0;
	}
	else {
		s5 = sqrt(1 - c5 * c5);//正
		sita5[0] = atan2(s5, c5);
		s5 = -s5;//负
		sita5[1] = atan2(s5, c5);
	}
	/*sita1[1]*/
	c5 = sin(sita1[1]) * r13 - cos(sita1[1]) * r23;
	if (CheckZeroThresh(fabs(c5 - 1))) {
		sita5[2] = 0;
		sita5[3] = 0;
	}
	else {
		s5 = sqrt(1 - c5 * c5);//正
		sita5[2] = atan2(s5, c5);
		s5 = -s5;//负
		sita5[3] = atan2(s5, c5);
	}

	sol[0][4] = sita5[0];
	sol[1][4] = sita5[0];
	sol[2][4] = sita5[1];
	sol[3][4] = sita5[1];
	sol[4][4] = sita5[2];
	sol[5][4] = sita5[2];
	sol[6][4] = sita5[3];
	sol[7][4] = sita5[3];

	/*sita6*/
	double C6, S6;
	//sita1[0]->sita5[0]
	if (CheckZeroThresh(sita5[0])) {//检查奇异性
		sita6[0] = theta6;
	}
	else {
		C6 = -(cos(sita1[0]) * r21 - sin(sita1[0]) * r11) / sin(sita5[0]);
		S6 = (cos(sita1[0]) * r22 - sin(sita1[0]) * r12) / sin(sita5[0]);
		if (CheckZeroThresh(C6) && CheckZeroThresh(S6)) {
			cout << "ERROR IN IK of sita6 : CheckZeroThresh(C6) && CheckZeroThresh(S6)" << endl;
			system("pause");
			exit(0);
		}
		sita6[0] = atan2(S6, C6);
	}
	//sita1[0]->sita5[1]
	if (CheckZeroThresh(sita5[1])) {//检查奇异性
		sita6[1] = theta6;
	}
	else {
		C6 = -(cos(sita1[0]) * r21 - sin(sita1[0]) * r11) / sin(sita5[1]);
		S6 = (cos(sita1[0]) * r22 - sin(sita1[0]) * r12) / sin(sita5[1]);
		if (CheckZeroThresh(C6) && CheckZeroThresh(S6)) {
			cout << "ERROR IN IK of sita6 : CheckZeroThresh(C6) && CheckZeroThresh(S6)" << endl;
			system("pause");
			exit(0);
		}
		sita6[1] = atan2(S6, C6);
	}
	//sita1[1]->sita5[2]
	if (CheckZeroThresh(sita5[2])) {//检查奇异性
		sita6[2] = theta6;
	}
	else {
		C6 = -(cos(sita1[1]) * r21 - sin(sita1[1]) * r11) / sin(sita5[2]);
		S6 = (cos(sita1[1]) * r22 - sin(sita1[1]) * r12) / sin(sita5[2]);
		if (CheckZeroThresh(C6) && CheckZeroThresh(S6)) {
			cout << "ERROR IN IK of sita6 : CheckZeroThresh(C6) && CheckZeroThresh(S6)" << endl;
			system("pause");
			exit(0);
		}
		sita6[2] = atan2(S6, C6);
	}
	//sita1[1]->sita5[3]
	if (CheckZeroThresh(sita5[3])) {//检查奇异性
		sita6[3] = theta6;
	}
	else {
		C6 = -(cos(sita1[1]) * r21 - sin(sita1[1]) * r11) / sin(sita5[3]);
		S6 = (cos(sita1[1]) * r22 - sin(sita1[1]) * r12) / sin(sita5[3]);
		if (CheckZeroThresh(C6) && CheckZeroThresh(S6)) {
			cout << "ERROR IN IK of sita6 " << ": CheckZeroThresh(C6) && CheckZeroThresh(S6)" << endl;
			system("pause");
			exit(0);
		}
		sita6[3] = atan2(S6, C6);
	}

	sol[0][5] = sita6[0];
	sol[1][5] = sita6[0];
	sol[2][5] = sita6[1];
	sol[3][5] = sita6[1];
	sol[4][5] = sita6[2];
	sol[5][5] = sita6[2];
	sol[6][5] = sita6[3];
	sol[7][5] = sita6[3];

	/*sita(2+3+4)*/
	double S234;
	double C234;

	/*sita1[0]->sita5[0] sita6[0]*/
	S234 = cos(sita5[0]) * (cos(sita6[0]) * r31 - sin(sita6[0]) * r32) - sin(sita5[0]) * r33;
	C234 = cos(sita6[0]) * r32 + sin(sita6[0]) * r31;
	sita234[0] = atan2(S234, C234);

	/*sita1[0]->sita5[1] sita6[1]*/
	S234 = cos(sita5[1]) * (cos(sita6[1]) * r31 - sin(sita6[1]) * r32) - sin(sita5[1]) * r33;
	C234 = cos(sita6[1]) * r32 + sin(sita6[1]) * r31;
	sita234[1] = atan2(S234, C234);

	/*sita1[1]->sita5[2] sita6[2]*/
	S234 = cos(sita5[2]) * (cos(sita6[2]) * r31 - sin(sita6[2]) * r32) - sin(sita5[2]) * r33;
	C234 = cos(sita6[2]) * r32 + sin(sita6[2]) * r31;
	sita234[2] = atan2(S234, C234);

	/*sita1[1]->sita5[3] sita6[3]*/
	S234 = cos(sita5[3]) * (cos(sita6[3]) * r31 - sin(sita6[3]) * r32) - sin(sita5[3]) * r33;
	C234 = cos(sita6[3]) * r32 + sin(sita6[3]) * r31;
	sita234[3] = atan2(S234, C234);

	/*sita3*/
	double C3, S3;
	/*sita1[0]->sita5[0]->sita6[0]->sita234[0]*/
	t1 = cos(sita1[0]) * px + sin(sita1[0]) * py - d5 * sin(sita234[0]) + d6 * sin(sita5[0]) * cos(sita234[0]);
	t2 = pz - d1 + d5 * cos(sita234[0]) + d6 * sin(sita5[0]) * sin(sita234[0]);
	C3 = (t1 * t1 + t2 * t2 - a3 * a3 - a2 * a2) / (2 * a2 * a3);
	if (CheckZeroThresh(fabs(C3 - 1))) {
		sita3[0] = 0;
		sita3[1] = 0;
	}
	else {
		S3 = sqrt(1 - C3 * C3);
		sita3[0] = atan2(S3, C3);
		sita3[1] = atan2(-S3, C3);
	}
	/*sita1[0]->sita5[1]->sita6[1]->sita234[1]*/
	t1 = cos(sita1[0]) * px + sin(sita1[0]) * py - d5 * sin(sita234[1]) + d6 * sin(sita5[1]) * cos(sita234[1]);
	t2 = pz - d1 + d5 * cos(sita234[1]) + d6 * sin(sita5[1]) * sin(sita234[1]);
	C3 = (t1 * t1 + t2 * t2 - a3 * a3 - a2 * a2) / (2 * a2 * a3);
	if (CheckZeroThresh(fabs(C3 - 1))) {
		sita3[2] = 0;
		sita3[3] = 0;
	}
	else {
		S3 = sqrt(1 - C3 * C3);
		sita3[2] = atan2(S3, C3);
		sita3[3] = atan2(-S3, C3);
	}
	/*sita1[1]->sita5[2]->sita6[2]->sita234[2]*/
	t1 = cos(sita1[1]) * px + sin(sita1[1]) * py - d5 * sin(sita234[2]) + d6 * sin(sita5[2]) * cos(sita234[2]);
	t2 = pz - d1 + d5 * cos(sita234[2]) + d6 * sin(sita5[2]) * sin(sita234[2]);
	C3 = (t1 * t1 + t2 * t2 - a3 * a3 - a2 * a2) / (2 * a2 * a3);
	if (CheckZeroThresh(fabs(C3 - 1))) {
		sita3[4] = 0;
		sita3[5] = 0;
	}
	else {
		S3 = sqrt(1 - C3 * C3);
		sita3[4] = atan2(S3, C3);
		sita3[5] = atan2(-S3, C3);
	}
	/*sita1[1]->sita5[3]->sita6[3]->sita234[3]*/
	t1 = cos(sita1[1]) * px + sin(sita1[1]) * py - d5 * sin(sita234[3]) + d6 * sin(sita5[3]) * cos(sita234[3]);
	t2 = pz - d1 + d5 * cos(sita234[3]) + d6 * sin(sita5[3]) * sin(sita234[3]);
	C3 = (t1 * t1 + t2 * t2 - a3 * a3 - a2 * a2) / (2 * a2 * a3);
	if (CheckZeroThresh(fabs(C3 - 1))) {
		sita3[6] = 0;
		sita3[7] = 0;
	}
	else {
		S3 = sqrt(1 - C3 * C3);
		sita3[6] = atan2(S3, C3);
		sita3[7] = atan2(-S3, C3);
	}

	sol[0][2] = sita3[0];
	sol[1][2] = sita3[1];
	sol[2][2] = sita3[2];
	sol[3][2] = sita3[3];
	sol[4][2] = sita3[4];
	sol[5][2] = sita3[5];
	sol[6][2] = sita3[6];
	sol[7][2] = sita3[7];

	/*sita2*/
	double C2, S2;
	/*sita1[0]->sita5[0]->sita6[0]->sita234[0]->sita3[0]*/
	t1 = cos(sita1[0]) * px + sin(sita1[0]) * py - d5 * sin(sita234[0]) + d6 * sin(sita5[0]) * cos(sita234[0]);
	t2 = pz - d1 + d5 * cos(sita234[0]) + d6 * sin(sita5[0]) * sin(sita234[0]);
	C2 = ((a3 * cos(sita3[0]) + a2) * t1 + t2 * a3 * sin(sita3[0])) / -(a2 * a2 + a3 * a3 + 2 * a2 * a3 * cos(sita3[0]));
	S2 = (t2 + a3 * sin(sita3[0]) * C2) / -(a3 * cos(sita3[0]) + a2);
	sita2[0] = atan2(S2, C2);
	/*sita1[0]->sita5[0]->sita6[0]->sita234[0]->sita3[1]*/
	t1 = cos(sita1[0]) * px + sin(sita1[0]) * py - d5 * sin(sita234[0]) + d6 * sin(sita5[0]) * cos(sita234[0]);
	t2 = pz - d1 + d5 * cos(sita234[0]) + d6 * sin(sita5[0]) * sin(sita234[0]);
	C2 = ((a3 * cos(sita3[1]) + a2) * t1 + t2 * a3 * sin(sita3[1])) / -(a2 * a2 + a3 * a3 + 2 * a2 * a3 * cos(sita3[1]));
	S2 = (t2 + a3 * sin(sita3[1]) * C2) / -(a3 * cos(sita3[1]) + a2);
	sita2[1] = atan2(S2, C2);
	/*sita1[0]->sita5[1]->sita6[1]->sita234[1]->sita3[2]*/
	t1 = cos(sita1[0]) * px + sin(sita1[0]) * py - d5 * sin(sita234[1]) + d6 * sin(sita5[1]) * cos(sita234[1]);
	t2 = pz - d1 + d5 * cos(sita234[1]) + d6 * sin(sita5[1]) * sin(sita234[1]);
	C2 = ((a3 * cos(sita3[2]) + a2) * t1 + t2 * a3 * sin(sita3[2])) / -(a2 * a2 + a3 * a3 + 2 * a2 * a3 * cos(sita3[2]));
	S2 = (t2 + a3 * sin(sita3[2]) * C2) / -(a3 * cos(sita3[2]) + a2);
	sita2[2] = atan2(S2, C2);
	/*sita1[0]->sita5[1]->sita6[1]->sita234[1]->sita3[3]*/
	t1 = cos(sita1[0]) * px + sin(sita1[0]) * py - d5 * sin(sita234[1]) + d6 * sin(sita5[1]) * cos(sita234[1]);
	t2 = pz - d1 + d5 * cos(sita234[1]) + d6 * sin(sita5[1]) * sin(sita234[1]);
	C2 = ((a3 * cos(sita3[3]) + a2) * t1 + t2 * a3 * sin(sita3[3])) / -(a2 * a2 + a3 * a3 + 2 * a2 * a3 * cos(sita3[3]));
	S2 = (t2 + a3 * sin(sita3[3]) * C2) / -(a3 * cos(sita3[3]) + a2);
	sita2[3] = atan2(S2, C2);

	/*sita1[1]->sita5[2]->sita6[2]->sita234[2]->sita3[4]*/
	t1 = cos(sita1[1]) * px + sin(sita1[1]) * py - d5 * sin(sita234[2]) + d6 * sin(sita5[2]) * cos(sita234[2]);
	t2 = pz - d1 + d5 * cos(sita234[2]) + d6 * sin(sita5[2]) * sin(sita234[2]);
	C2 = ((a3 * cos(sita3[4]) + a2) * t1 + t2 * a3 * sin(sita3[4])) / -(a2 * a2 + a3 * a3 + 2 * a2 * a3 * cos(sita3[4]));
	S2 = (t2 + a3 * sin(sita3[4]) * C2) / -(a3 * cos(sita3[4]) + a2);
	sita2[4] = atan2(S2, C2);
	/*sita1[1]->sita5[2]->sita6[2]->sita234[2]->sita3[5]*/
	t1 = cos(sita1[1]) * px + sin(sita1[1]) * py - d5 * sin(sita234[2]) + d6 * sin(sita5[2]) * cos(sita234[2]);
	t2 = pz - d1 + d5 * cos(sita234[2]) + d6 * sin(sita5[2]) * sin(sita234[2]);
	C2 = ((a3 * cos(sita3[5]) + a2) * t1 + t2 * a3 * sin(sita3[5])) / -(a2 * a2 + a3 * a3 + 2 * a2 * a3 * cos(sita3[5]));
	S2 = (t2 + a3 * sin(sita3[5]) * C2) / -(a3 * cos(sita3[5]) + a2);
	sita2[5] = atan2(S2, C2);
	/*sita1[1]->sita5[3]->sita6[3]->sita234[3]->sita3[6]*/
	t1 = cos(sita1[1]) * px + sin(sita1[1]) * py - d5 * sin(sita234[3]) + d6 * sin(sita5[3]) * cos(sita234[3]);
	t2 = pz - d1 + d5 * cos(sita234[3]) + d6 * sin(sita5[3]) * sin(sita234[3]);
	C2 = ((a3 * cos(sita3[6]) + a2) * t1 + t2 * a3 * sin(sita3[6])) / -(a2 * a2 + a3 * a3 + 2 * a2 * a3 * cos(sita3[6]));
	S2 = (t2 + a3 * sin(sita3[6]) * C2) / -(a3 * cos(sita3[6]) + a2);
	sita2[6] = atan2(S2, C2);
	/*sita1[1]->sita5[3]->sita6[3]->sita234[3]->sita3[7]*/
	t1 = cos(sita1[1]) * px + sin(sita1[1]) * py - d5 * sin(sita234[3]) + d6 * sin(sita5[3]) * cos(sita234[3]);
	t2 = pz - d1 + d5 * cos(sita234[3]) + d6 * sin(sita5[3]) * sin(sita234[3]);
	C2 = ((a3 * cos(sita3[7]) + a2) * t1 + t2 * a3 * sin(sita3[7])) / -(a2 * a2 + a3 * a3 + 2 * a2 * a3 * cos(sita3[7]));
	S2 = (t2 + a3 * sin(sita3[7]) * C2) / -(a3 * cos(sita3[7]) + a2);
	sita2[7] = atan2(S2, C2);

	sol[0][1] = sita2[0];
	sol[1][1] = sita2[1];
	sol[2][1] = sita2[2];
	sol[3][1] = sita2[3];
	sol[4][1] = sita2[4];
	sol[5][1] = sita2[5];
	sol[6][1] = sita2[6];
	sol[7][1] = sita2[7];

	/*sita4*/
		/*sita234[0]->sita3[0]->sita2[0]*/
	sita4[0] = sita234[0] - sita3[0] - sita2[0];
	/*sita234[0]->sita3[1]->sita2[1]*/
	sita4[1] = sita234[0] - sita3[1] - sita2[1];
	/*sita234[1]->sita3[2]->sita2[2]*/
	sita4[2] = sita234[1] - sita3[2] - sita2[2];
	/*sita234[1]->sita3[3]->sita2[3]*/
	sita4[3] = sita234[1] - sita3[3] - sita2[3];
	/*sita234[2]->sita3[4]->sita2[4]*/
	sita4[4] = sita234[2] - sita3[4] - sita2[4];
	/*sita234[2]->sita3[5]->sita2[5]*/
	sita4[5] = sita234[2] - sita3[5] - sita2[5];
	/*sita234[3]->sita3[6]->sita2[6]*/
	sita4[6] = sita234[3] - sita3[6] - sita2[6];
	/*sita234[3]->sita3[7]->sita2[7]*/
	sita4[7] = sita234[3] - sita3[7] - sita2[7];

	sol[0][3] = sita4[0];
	sol[1][3] = sita4[1];
	sol[2][3] = sita4[2];
	sol[3][3] = sita4[3];
	sol[4][3] = sita4[4];
	sol[5][3] = sita4[5];
	sol[6][3] = sita4[6];
	sol[7][3] = sita4[7];

	return sol;
}


/*
Function:  已知q和p，求解出d1，a2，a3，d4，d5，d6。单位为弧度和米 （此方法弃之）
i
o
m：已知两组q和p，得到含6个方程的方程组，求解即可得到IKparam
*/
MatrixXd getIKparam(vector<double> q1,vector<double> p1,vector<double> q2,vector<double> p2) {
	assert(q1.size() == 6 && q2.size() == 6);
	assert(p1.size() == 3 && p2.size() == 3);//此函数中p只需要x，y，z。不用rx ry rz

	MatrixXd IKparam(6, 1);
	
	MatrixXd matrixOF2p(6, 1);//matrixOF2p=(p1,p2)^T
	matrixOF2p(0, 0) = p1[0];
	matrixOF2p(1, 0) = p1[1];
	matrixOF2p(2, 0) = p1[2];
	matrixOF2p(3, 0) = p2[0];
	matrixOF2p(4, 0) = p2[1];
	matrixOF2p(5, 0) = p2[2];

	MatrixXd m(6, 6);
	//由第一组q给m的前三行赋值
	vector<double> angles = q1;
	double C1 = cos(angles[0]);
	double C2 = cos(angles[1]);
	double C3 = cos(angles[2]);
	double C4 = cos(angles[3]);
	double C5 = cos(angles[4]);
	double C6 = cos(angles[5]);

	double S1 = sin(angles[0]);
	double S2 = sin(angles[1]);
	double S3 = sin(angles[2]);
	double S4 = sin(angles[3]);
	double S5 = sin(angles[4]);
	double S6 = sin(angles[5]);

	double C234 = cos(angles[1] + angles[2] + angles[3]);
	double S234 = sin(angles[1] + angles[2] + angles[3]);
	double C23 = cos(angles[1] + angles[2]);
	double S23 = sin(angles[1] + angles[2]);
	
	m(0, 0) = 0;
	m(0, 1) = -C1 * C2;
	m(0, 2) = -C23 * C1;
	m(0, 3) = S1;
	m(0, 4) = S234 * C1;
	m(0, 5) = (C5 * S1 - C234 * C1 * S5);

	m(1, 0) = 0;
	m(1, 1) = -C2 * S1;
	m(1, 2) = -C23 * S1;
	m(1, 3) = -C1;
	m(1, 4) = S234 * S1;
	m(1, 5) = -(C1 * C5 + C234 * S1 * S5);

	m(2, 0) = 1;
	m(2, 1) = -S2;
	m(2, 2) = -S23;
	m(2, 3) = 0;
	m(2, 4) = -(C23 * C4 - S23 * S4);
	m(2, 5) = -S5 * (C23 * S4 + S23 * C4);

	//由第二组q给m的前三行赋值
	angles = q2;
	C1 = cos(angles[0]);
	C2 = cos(angles[1]);
	C3 = cos(angles[2]);
	C4 = cos(angles[3]);
	C5 = cos(angles[4]);
	C6 = cos(angles[5]);

	S1 = sin(angles[0]);
	S2 = sin(angles[1]);
	S3 = sin(angles[2]);
	S4 = sin(angles[3]);
	S5 = sin(angles[4]);
	S6 = sin(angles[5]);

	C234 = cos(angles[1] + angles[2] + angles[3]);
	S234 = sin(angles[1] + angles[2] + angles[3]);
	C23 = cos(angles[1] + angles[2]);
	S23 = sin(angles[1] + angles[2]);

	m(3, 0) = 0;
	m(3, 1) = -C1 * C2;
	m(3, 2) = -C23 * C1;
	m(3, 3) = S1;
	m(3, 4) = S234 * C1;
	m(3, 5) = (C5 * S1 - C234 * C1 * S5);

	m(4, 0) = 0;
	m(4, 1) = -C2 * S1;
	m(4, 2) = -C23 * S1;
	m(4, 3) = -C1;
	m(4, 4) = S234 * S1;
	m(4, 5) = -(C1 * C5 + C234 * S1 * S5);

	m(5, 0) = 1;
	m(5, 1) = -S2;
	m(5, 2) = -S23;
	m(5, 3) = 0;
	m(5, 4) = -(C23 * C4 - S23 * S4);
	m(5, 5) = -S5 * (C23 * S4 + S23 * C4);

	MatrixXd m_inverse = m.inverse();
	IKparam = m_inverse * matrixOF2p;

	return IKparam;
}


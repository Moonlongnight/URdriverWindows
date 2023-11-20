#pragma once
#include <iostream>
#include <Eigen/Dense>
#include<math.h>
#include<fstream>
#include<vector>
#include "trans.h"
#include"Point.h"
using namespace Eigen;
using namespace std;

//��������ģ��
double calNormOFvector(Vector3d v1) {
	return sqrt(v1(0) * v1(0) + v1(1) * v1(1) + v1(2) * v1(2));
}

//�����������нǣ�����ֵΪ���ȡ�
//ʹ�ù�ʽΪ |sinx|/cosx������sinxȡ����ֵ����˷��ؽǶȷ�ΧΪ[0,PI]�������Ǹ�����
double calAngleOFvector(Vector3d v1, Vector3d v2) {
	return atan2(calNormOFvector(v1.cross(v2)), v1.dot(v2));
}

//�����������
double calDisOF_2points(Vector3d p1, Vector3d p2) {
	return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) + pow(p2[2] - p1[2], 2));
}

double calDisOF_2points(vector<double> p1, vector<double> p2) {
	return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) + pow(p2[2] - p1[2], 2));
}
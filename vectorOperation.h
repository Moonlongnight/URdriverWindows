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

//计算向量模长
double calNormOFvector(Vector3d v1) {
	return sqrt(v1(0) * v1(0) + v1(1) * v1(1) + v1(2) * v1(2));
}

//计算两向量夹角，返回值为弧度。
//使用公式为 |sinx|/cosx。由于sinx取绝对值，因此返回角度范围为[0,PI]，不会是负数。
double calAngleOFvector(Vector3d v1, Vector3d v2) {
	return atan2(calNormOFvector(v1.cross(v2)), v1.dot(v2));
}

//计算两点距离
double calDisOF_2points(Vector3d p1, Vector3d p2) {
	return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) + pow(p2[2] - p1[2], 2));
}

double calDisOF_2points(vector<double> p1, vector<double> p2) {
	return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) + pow(p2[2] - p1[2], 2));
}
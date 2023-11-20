#pragma once
#include<iostream>
#include<Eigen/Dense>
#include<math.h>
#include"trans.h"
using Eigen::MatrixXd;
using namespace std;
/*
Function: transformOrientation
i
o
m:为便于运算，统一认为四元数的sita>=0。要表示所有旋转，sita范围为[0,2PI],因此sita/2的范围为[0,PI/2]
*/
class Quaternion {
public:
	double a = 0, b = 0, c = 0, d = 0;//Quaternion
public:
	/*
	Function:  四元数构造函数
	i
	o
	m
	*/
	Quaternion(double a_ = 0, double b_ = 0, double c_ = 0, double d_ = 0) {
		a = a_;
		b = b_;
		c = c_;
		d = d_;
	}
};
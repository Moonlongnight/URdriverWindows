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
m:Ϊ�������㣬ͳһ��Ϊ��Ԫ����sita>=0��Ҫ��ʾ������ת��sita��ΧΪ[0,2PI],���sita/2�ķ�ΧΪ[0,PI/2]
*/
class Quaternion {
public:
	double a = 0, b = 0, c = 0, d = 0;//Quaternion
public:
	/*
	Function:  ��Ԫ�����캯��
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
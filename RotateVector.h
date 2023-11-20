#pragma once
#include<iostream>
#include<math.h>
using namespace std;

/*
Function:  Ðý×ªÊ¸Á¿Àà
i
o
m
*/
class RotateVector {
public:
	double Rx = 0, Ry = 0, Rz = 0;
	RotateVector(double Rx_ = 0, double Ry_ = 0, double Rz_ = 0) {
		Rx = Rx_;
		Ry = Ry_;
		Rz = Rz_;
	}
	double getMagnitude() {
		return sqrt(Rx * Rx + Ry * Ry + Rz * Rz);
	}
};
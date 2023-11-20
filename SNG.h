#pragma once
#include<iostream>
using namespace std;


template <typename T>
int SNG(T num) {
	if (num == 0)
		return 0;
	else if (num > 0)
		return 1;
	else
		return -1;
}

#pragma once
#include<string>
#include<vector>

using namespace std;

//stopj(a)
//Decelerate joint speeds with a to zero
string get_stopj_str(double a) {
	string str = "stopj(" + to_string(a) + ")";
	return str;
}
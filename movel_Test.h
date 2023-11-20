#pragma once
#include<iostream>
#include<fstream>
#include<vector>
#include<assert.h>
#include<string>

#include"getLineNumber.h"
#include"checkfp.h"
#include"absd.h"
#include"trans.h"

using namespace std;

//movel([0, 0, 0, d2r(-90), 0, 0], a = 2, v = 2)
//a£ºm/s^2  v£ºm/s  r£ºm
string get_movel_str(vector<double> q, double a, double v, double t, double r) {
	string str;

	str = "movel([" + \
		to_string(q[0]) + "," + to_string(q[1]) + "," + to_string(q[2]) + "," + to_string(q[3]) + "," + to_string(q[4]) + "," + to_string(q[5]) + "]," + \
		"a=" + to_string(a) + "," + "v=" + to_string(v) + "," + "t=" + to_string(t) + "," + "r=" + to_string(r) + ")";

	return str;
}

string generate_movel_code() {
	int lineNum = getLineNumber("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt", "");
	ifstream ifp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
	checkfp(ifp);

	vector<double> q(6);
	double a, v, t, r;
	a = 1;// m/s^2
	v = 0.2;// 0.2m/s
	t = 0;
	r = 0.15 * mm2m;// r(i) < 0.5*min(dis(i,j),dis(j,k))

	string cmd_str;
	cmd_str += "def driverProg():\n";

	//²âÊÔÇ°41¸öµã
	ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
	//cmd_str += "\t" + get_movel_str(q, a, v, t, r) + "\n";

	r = 0.15 * mm2m;
	for (int i = 0; i < 40; i++) {
		ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
		cmd_str += "\t" + get_movel_str(q, a, v, t, r) + "\n";
	}

	cmd_str += "end\n";
	return cmd_str;
}
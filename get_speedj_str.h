#pragma once
#include<string>
#include<vector>

using namespace std;

//speedj([0.2,0.3,0.1,0.05,0,0], 0.5, 0.5)
string get_speedj_str(vector<double> qd, double a, double t) {
	string str;
	if (t != 0) {
		str = "speedj([" + \
			to_string(qd[0]) + "," + to_string(qd[1]) + "," + to_string(qd[2]) + "," + to_string(qd[3]) + "," + to_string(qd[4]) + "," + to_string(qd[5]) + "]," + \
			"a=" + to_string(a) + "," + "t=" + to_string(t) + ")";
	}
	else {//≤ª π”√t
		str = "speedj([" + \
			to_string(qd[0]) + "," + to_string(qd[1]) + "," + to_string(qd[2]) + "," + to_string(qd[3]) + "," + to_string(qd[4]) + "," + to_string(qd[5]) + "]," + \
			"a=" + to_string(a) + ")";
	}
	return str;
}
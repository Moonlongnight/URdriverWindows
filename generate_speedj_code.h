//X

#pragma once
#include<iostream>
#include<fstream>
#include<vector>
#include<string>

#include"checkfp.h"
#include"getLineNumber.h"
#include"get_speedj_str.h"
#include"get_stopj_str.h"

using namespace std;

/*����speedj�ű�*/
string generate_speedj_code() {
	/*read imp.txt*/
	int groupNum = getLineNumber("C:/Users/XY/Desktop/path/trajectoryPlanning/imp.txt", "")/5;
	
	ifstream ifp("C:/Users/XY/Desktop/path/trajectoryPlanning/imp.txt");//q v main_axis_a t tl
	checkfp(ifp);
	vector<vector<double>> q(groupNum);
	vector<vector<double>> v(groupNum);
	vector<double> a(groupNum);
	vector<double> t(groupNum);
	vector<double> tl(groupNum);

	for (int i = 0; i < groupNum; i++) {
		for (int j = 0; j < 6; j++) {
			q[i].resize(6);
			ifp >> q[i][j];
		}
		for (int j = 0; j < 6; j++) {
			v[i].resize(6);
			ifp >> v[i][j];
		}
		ifp >> a[i];
		ifp >> t[i];
		ifp >> tl[i];
	}

	//now we have
	//q[0][...]--0�����Ƕ�
	//v[0][...]--01ֱ�߶θ����ٶ�
	//a[0]--0��Ĺ��ɶ�������ٶ�
	//t[0]--0��Ĺ��ɶ�ʱ�䡣ֵ��ע����ǣ�ʼĩ��Ĺ��ɶξ�����һ·���Ρ��м��Ĺ��ɶ����Ƿֲ������ĵ����ߵ�·���Ρ�
	//tl[0]--01ֱ�߶�ʱ��

	//ÿ�����Ӧһ��speedj������0��speedj: �Լ��ٶ�a[0]������v[0][...]��Ȼ�������˶����������̵ĳ���ʱ��Ϊt = t[0] + tl[0]
	//�ر�ģ�����ĩ��n = q.size()-1�����Ӧ������Ϊstopj: stopj�ļ��ٶ�Ϊa = a[q.size()-1]

	//speedj(qd, a, t)
	//Accelerate linearly in joint space and continue with constant joint\
	speed.The time t is optional; if provided the function will return after\
	time t, regardless of the target speed has been reached.If the time t is\
	not provided, the function will return when the target speed is reached
	string cmd_str;
	cmd_str += "def driverProg():\n";
	vector<double> qd(6);
	for (int i = 0; i < 6; i++) {
		qd[i] = 0;
	}
	double a_cur = 0;
	double t_cur = 0;
	for (int i = 0; i < q.size() - 1; i++) {
		for (int j = 0; j < 6; j++) {
			qd[j] = v[i][j];
		}
		a_cur = a[i];
		t_cur = t[i] + tl[i];
		cmd_str += "\t" + get_speedj_str(qd, a_cur, t_cur) + "\n";
	}
	a_cur = a[q.size() - 1];
	cmd_str += "\t" + get_stopj_str(a_cur) + "\n";
	cmd_str += "end\n";

	return cmd_str;
}




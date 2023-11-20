#pragma once
#include<iostream>
#include<fstream>
#include<vector>
#include<assert.h>

#include"getLineNumber.h"
#include"checkfp.h"
#include"absd.h"
#include"generate_speedj_code.h"

using namespace std;
/*此文件针对论文《Creating Through Points in Linear Function with Parabolic Blends Path by Optimization Method》中所提及的方法进行了依次复现*/

//LSPB STRUCT
typedef struct {
	vector<double> q, v, a, t, tl, td;
}LSPB_STRUCT;

/*Normal LSPB*/

//solve 1 dimension LSPB problem
//Known:q,td,t		Get:v,tl,a
LSPB_STRUCT Normal_LSPB_1D(vector<double> q, vector<double> td, vector<double> t) {
	int i_temp;
	double d_temp;
	vector<double> vd_temp;

	assert(q.size() == t.size() && q.size() - 1 == td.size());
	int qsize = q.size();
	int tdsize = qsize - 1;

	//cal tl
	vector<double> tl(tdsize);
	tl[0] = td[0] - t[0] - 0.5 * t[1];
	assert(tl[0] >= 0);
	for (int i = 1; i < tl.size() - 1; i++) {
		tl[i] = td[i] - 0.5 * t[i] - 0.5 * t[i + 1];
		assert(tl[i] >= 0);
	}
	tl[tl.size() - 1] = td[td.size() - 1] - 0.5 * t[t.size() - 2] - t[t.size() - 1];
	assert(tl[tl.size() - 1] >= 0);

	//cal v
	vector<double> v(tdsize);
	v[0] = (q[1] - q[0]) / (td[0] - 0.5 * t[0]);
	for (int i = 1; i < v.size() - 1; i++) {
		v[i] = (q[i + 1] - q[i]) / td[i];
	}
	v[v.size() - 1] = (q[qsize - 1] - q[qsize - 2]) / (td[tdsize - 1] - 0.5 * t[t.size() - 1]);
	
	//cal a
	vector<double> a(qsize);
	a[0] = (v[0] - 0) / t[0];
	for (int i = 1; i < a.size() - 1; i++) {
		a[i] = (v[i] - v[i - 1]) / t[i];
	}
	a[a.size() - 1] = (0 - v[v.size() - 1]) / t[t.size() - 1];

	//return
	LSPB_STRUCT result;
	result.q = q;
	result.v = v;
	result.a = a;
	result.t = t;
	result.tl = tl;
	result.td = td;
	return result;
}


//LSPB_6D
string Normal_LSPB_6D() {
	int i_temp;
	double d_temp;
	vector<double> vd_temp;

	/*read IKresult into q*/
	string qfile_addr = "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt";
	int qsize = getLineNumber(qfile_addr, "");
	ifstream qfile(qfile_addr);
	checkfp(qfile);

	vector<double> q1(qsize);
	vector<double> q2(qsize);
	vector<double> q3(qsize);
	vector<double> q4(qsize);
	vector<double> q5(qsize);
	vector<double> q6(qsize);
	for (int i = 0; i < qsize; i++) {
		qfile >> q1[i];
		qfile >> q2[i];
		qfile >> q3[i];
		qfile >> q4[i];
		qfile >> q5[i];
		qfile >> q6[i];
	}
	qfile.close();

	/*将t手动赋值*/
	//针对hand85.txt的关节2运动进行实验,采用normal_LSPB
	vector<double> t(qsize);
	t[0] = 0.125;
	t[1] = 0.133;
	for (int i = 2; i < t.size()-1; i++) {
		t[i] = 0.016;
	}
	t[t.size() - 1] = 0.016;

	//实验speedj的特性,采用normal_LSPB，将td减小至0.016，观察是否能正常运行（自己写q，保证a<5rad/s）
	t[0] = 0.125;
	t[1] = 0.133;
	for (int i = 2; i < t.size() - 1; i++) {
		t[i] = 0.016;
	}
	t[t.size() - 1] = 0.008;

	//实验speedj的特性,采用normal_LSPB，测试当td不是0.008整数倍时的运行情况
	t[0] = 0.125;
	t[1] = 0.133;
	for (int i = 2; i < t.size() - 1; i++) {
		t[i] = 0.016;
	}
	t[t.size() - 1] = 0.016;

	/*将td手动赋值*/
	//针对hand85.txt的关节2运动进行实验,采用normal_LSPB,使用恒定的td=0.032
	vector<double> td(qsize-1);
	td[0] = 0.4415;
	td[1] = 0.0905;
	for (int i = 2; i < td.size() - 1; i++) {
		td[i] = 0.032;
	}
	td[td.size() - 1] = 0.032;

	//实验speedj的特性,采用normal_LSPB，将td减小至0.016，观察是否能正常运行（自己写q，保证a<5rad/s）
	td[0] = 0.4415;
	td[1] = 0.0905;
	for (int i = 2; i < td.size() - 1; i++) {
		td[i] = 0.016;
	}
	td[td.size() - 1] = 0.016;
	
	//实验speedj的特性,采用normal_LSPB，将td=0.016和td=0.032混合，观察是否能正常运行（自己写q，保证a<5rad/s^2）
	td[0] = 0.4415;
	td[1] = 0.0905;
	for (int i = 2; i < td.size() - 1; i++) {
		td[i] = 0.016;
	}
	td[4] = 0.032;
	td[5] = 0.032;
	td[7] = 0.032;
	td[td.size() - 1] = 0.016;

	//使用hand85.txt,实验speedj的特性,采用normal_LSPB，测试当td不是0.008整数倍时的运行情况
	td[0] = 0.4415;
	td[1] = 0.0905;
	for (int i = 2; i <= 34; i++) {
		td[i] = 0.032 + 0.001 * (i - 2);
	}
	for (int i = 35; i <= 67; i++) {
		td[i] = 0.032 + 0.001 * (i - 35);
	}
	for (int i = 68; i <= 83; i++) {
		td[i] = 0.032 + 0.001 * (i - 68);
	}

	/*分别计算各个关节的LSPB结果*/
	vector<LSPB_STRUCT> joints_result(6);

	joints_result[0] = Normal_LSPB_1D(q1, td, t);
	joints_result[1] = Normal_LSPB_1D(q2, td, t);
	joints_result[2] = Normal_LSPB_1D(q3, td, t);
	joints_result[3] = Normal_LSPB_1D(q4, td, t);
	joints_result[4] = Normal_LSPB_1D(q5, td, t);
	joints_result[5] = Normal_LSPB_1D(q6, td, t);

	/*计算主轴加速度*/
	vector<double> main_axis_a(qsize);
	d_temp = 0;//cur max of a
	i_temp = 0;//cur max's index of a
	for (int i = 0; i < main_axis_a.size(); i++) {
		d_temp = 0;
		for (int j = 0; j < 6; j++) {//关节j
			if (absd(joints_result[j].a[i]) > d_temp) {
				i_temp = j;
				d_temp = absd(joints_result[j].a[i]);//注意为了对速度的大小进行比较，d_temp为速度的绝对值
			}
		}

		main_axis_a[i] = joints_result[i_temp].a[i];
	}

	/*将结果写入*/
	//q  v  main_axis_a  t  tl
	ofstream ofp("C:/Users/XY/Desktop/path/trajectoryPlanning/imp.txt");
	checkfp(ofp);
	for (int i = 0; i < qsize-1; i++) {
		ofp << joints_result[0].q[i] << " " << joints_result[1].q[i] << " " << joints_result[2].q[i] << " " << joints_result[3].q[i] << " " << joints_result[4].q[i] << " " << joints_result[5].q[i] << endl;
		ofp << joints_result[0].v[i] << " " << joints_result[1].v[i] << " " << joints_result[2].v[i] << " " << joints_result[3].v[i] << " " << joints_result[4].v[i] << " " << joints_result[5].v[i] << endl;
		ofp << main_axis_a[i] << endl;
		ofp << t[i] << endl;
		ofp << joints_result[0].tl[i] << endl;//各轴的t和tl序列是一样的
	}
	ofp << joints_result[0].q[qsize-1] << " " << joints_result[1].q[qsize - 1] << " " << joints_result[2].q[qsize - 1] << " " << joints_result[3].q[qsize - 1] << " " << joints_result[4].q[qsize - 1] << " " << joints_result[5].q[qsize - 1] << endl;
	ofp << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << endl;
	ofp << main_axis_a[qsize - 1] << endl;
	ofp << t[qsize - 1] << endl;
	ofp << 0 << endl;

	//td
	ofstream td_modified("C:/Users/XY/Desktop/path/trajectoryPlanning/td_modifyed.txt");
	checkfp(td_modified);
	for (int i = 0; i < td.size(); i++) {
		td_modified << td[i] << endl;
	}
	td_modified.close();

	//存放各时刻点对应的“6个轴对应的加速度”
	ofstream ofp_a_6axis("C:/Users/XY/Desktop/path/trajectoryPlanning/a_6axis.txt");
	checkfp(ofp_a_6axis);
	for (int i = 0; i < qsize; i++) {
		ofp_a_6axis << joints_result[0].a[i] << " " << joints_result[1].a[i] << " " << joints_result[2].a[i] << " " << joints_result[3].a[i] << " " << joints_result[4].a[i] << " " << joints_result[5].a[i] << endl;
	}
	
	ofp.close();
	td_modified.close();
	ofp_a_6axis.close();

	return generate_speedj_code();
}


//获得合理的td序列
vector<double> Normal_LSPB_get_td(vector<double> td_seq) {
	const int Joint_Num = 6;
	int i_temp;
	double d_temp;
	vector<double> vd_temp;
	vector<vector<double>> vvd_temp;

	/*read IKresult into q*/
	string qfile_addr = "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt";
	int qsize = getLineNumber(qfile_addr, "");
	ifstream qfile(qfile_addr);
	checkfp(qfile);

	vector<vector<double>> q(qsize);
	for (int i = 0; i < qsize; i++) {
		q[i].resize(Joint_Num);
		for (int j = 0; j < Joint_Num; j++) {
			qfile >> q[i][j];
		}
	}

	qfile.close();

	/*将t手动赋值*/
	//针对hand85.txt的关节2运动进行实验,采用normal_LSPB
	vector<double> t(qsize);
	t[0] = 0.125;
	t[1] = 0.133;
	for (int i = 2; i < t.size() - 1; i++) {
		t[i] = 0.016;
	}
	t[t.size() - 1] = 0.016;

	/*td*/
	vector<double> td(qsize - 1);
	td[0] = 0.4415;
	td[1] = 0.0905;

	/*开始计算td_sol*/
	vector<vector<double>> td_sol(td_seq.size());
	td_sol[0].push_back(td[0]);
	td_sol[0].push_back(td[1]);
	for (int i = 1; i < td_sol.size(); i++) {
		td_sol[i].push_back(-1);
	}
	
	vector<double> sumOFtd(td_sol.size());//记录各td_sol对应的Σtd
	sumOFtd[0] = td_sol[0][0] + td_sol[0][1];

	//每次迭代，我们只需要为td_seq各项选择一条Σtd最小的线路即可。
	//我们让各td_sol随着下标从小到大，其Σtd是递增的
	for (int i = 2; i < qsize-2; i++) {//求td[i]
		for (int j = 0; j<td_seq.size(); j++) {//为0.016和0.032依次找Σtd最小序列
			for (int k = 0; k < td_sol.size(); k++) {//按照Σtd从小到大，依次分析“各线路td_sol[k]”在“td[i]=td_seq[j]时”，各轴加速度是否小于a_max
				//if td_sol[k][0]!=-1 && 各轴a[i]<a_max，then td_sol[k]+td_seq[j] as td_seq[j]'s minΣtd sol
				




			}

		}
		//将vdd_temp内容拷回给td_sol，期间维系各td_sol随着下标从小到大，其Σtd是递增的
	
	
	}
}
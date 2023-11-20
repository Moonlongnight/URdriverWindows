#pragma once
#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<assert.h>
#include<math.h>

#include"checkfp.h"
#include"getLineNumber.h"
#include"SNG.h"
#include"absd.h"
#include"vectorOperation.h"
#include"get_speedj_str.h"
#include"get_stopj_str.h"

using namespace std;	

const double INSPIRE_FACTOR=0.25;//生成过渡段时间的启发算法因子，用于“CAL_A函数中的过渡段时间计算”和“linearWithParabola_multiAxis函数中的直线段时间计算”。

//计算第i个点对应的过渡段加速度,其中过渡段时间采用启发算法:ti=1/4*t(i-1)(i)+1/4*t(i)(i+1)（始末点单独考虑）。
//theim1:theta(i-1)
double CAL_A(double theim1, double thei, double theip1, double tdim1, double tdi) {
	assert((tdi == 0 && tdim1 == 0) == 0);
	if (tdim1 == 0) {//始点
		return ((theip1 - thei) / tdi - 0) / (INSPIRE_FACTOR * tdi);
	}
	else if (tdi == 0) {//末点
		return (0 - (thei - theim1) / tdim1) / (INSPIRE_FACTOR * tdim1);
	}
	else {//中间点
		return ((theip1 - thei) / tdi - (thei - theim1) / tdim1) / (INSPIRE_FACTOR * tdi + INSPIRE_FACTOR * tdim1);
	}
}

vector<double> get_timeSeqWith_constantVcartesian(double v);
vector<double> check_timeSeqWith_amax(vector<double> td, double amax);
void write_IntoFile_imp(vector<double> q, vector<double> v, vector<double> a, vector<double> t, vector<double> tl);
void write_IntoFile_imp(vector<vector<double>> q, vector<vector<double>> v, vector<double> a, vector<double> t, vector<double> tl);


/*
Function:  关节空间下的单个关节的“带有抛物线过渡的线性函数”轨迹规划方法
i: tdij即i和j点的时间,对于过渡段加速度大小的设定，一般情况下使用anormal。
o: 轨迹规划对应的speedj脚本
m:
1.对于过渡段加速度大小的设定，一般情况下使用anormal，若amormal不够大，可使用amax
2.针对单个关节的旋转进行了轨迹规划，此中针对IK解中的关节1进行了轨迹规划。
3.使用方法:首先调用get_timeSeqWith_constantVcartesian(v)得到td，再调用此函数
*/
string linearWithParabola(vector<double> td,double anormal,double amax) {
	int i_temp;
	/*read IKresult into q*/
	string qfile_addr = "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt";
	i_temp = getLineNumber(qfile_addr, "");
	assert(i_temp - 1 == td.size());
	ifstream qfile(qfile_addr);
	checkfp(qfile);

	vector<vector<double>> q(i_temp);
	for (int i = 0; i < i_temp; i++) {
		q[i].resize(6);
		for (int j = 0; j < 6; j++) {
			qfile >> q[i][j];
		}
	}
	qfile.close();
	/*handle theta1*/
	vector<double> q_theta1(q.size());
	for (int i = 0; i < q.size(); i++) {
		q_theta1[i] = q[i][0];
	}
	//计算直线段速度，对于q.size个点的路径，有q.size-1个直线段
	//v_theta1[1]: v12
	vector<double> v_theta1(q.size() - 1);
	for (int i = 0; i < q.size()-1; i++) {
		v_theta1[i] = (q_theta1[i + 1] - q_theta1[i]) / td[i];
	}
	//计算加速度
	vector<double> a_theta1(q.size());
	a_theta1[0] = SNG(v_theta1[0] - 0) * anormal;//始
	a_theta1[q.size() - 1] = SNG(0 - v_theta1[q.size() - 2]) * anormal;//末
	for (int i = 1; i < q.size() - 1; i++) {
		a_theta1[i] = SNG(v_theta1[i] - v_theta1[i - 1]) * anormal;
	}
	//计算过渡段时间
	//每个点都有一对应过渡段。特别的，对于始点和末点，其过渡段负责从0加速至v01 以及 从v(n-1)(n)减速至0
	vector<double> t_theta1(q.size());
	t_theta1[0] = absd((v_theta1[0] - 0) / a_theta1[0]);//始
	t_theta1[q.size() - 1] = absd((0 - v_theta1[q.size() - 2]) / a_theta1[q.size() - 1]);//末
	for (int i = 1; i < q.size()-1; i++) {
		t_theta1[i] = absd((v_theta1[i] - v_theta1[i - 1]) / a_theta1[i]);
	}
	//计算直线段时间
	//tl_theta1[1]: time of line segment 12
	vector<double> tl_theta1(q.size() - 1);//time of line segment
	tl_theta1[0] = td[0] - t_theta1[0] - 0.5 * t_theta1[1];//始 tline01
	tl_theta1[q.size() - 2] = td[q.size() - 2] - t_theta1[q.size() - 1] - 0.5 * t_theta1[q.size() - 2];//末 tline(n-1)(n)
	for (int i = 1; i < q.size()-2; i++) {
		tl_theta1[i] = td[i] - 0.5 * t_theta1[i] - 0.5 * t_theta1[i + 1];
	}
	for (int i = 0; i < q.size()-1; i++) {
		assert(tl_theta1[i] >= 0);//若不满足，则表示加速度不够大
	}
	/*生成speedj脚本*/
	//now we have
	//q_theta1[0]--0点角度
	//v_theta1[0]--01直线段速度
	//a_theta1[0]--0点的过渡段加速度
	//t_theta1[0]--0点的过渡段时间。值得注意的是，始末点的过渡段局限于一路径段。中间点的过渡段则是分布于中心点两边的路径段。
	//tl_theta1[0]--01直线段时间
	write_IntoFile_imp(q_theta1, v_theta1, a_theta1, t_theta1, tl_theta1);

	//每个点对应一个speedj命令，如点0的speedj: 以加速度a_theta[0]加速至v_theta[0]，然后匀速运动，整个过程的持续时间为t = t_theta[0] + tl_theta[0]
	//特别的，对于末点n = q.size()-1，其对应的命令为stopj: stopj的加速度为a = a_theta[q.size()-1]

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
	double a = 0;
	double t = 0;
	for (int i = 0; i < q.size()-1; i++) {
		qd[0] = v_theta1[i];
		a = a_theta1[i];
		t = t_theta1[i] + tl_theta1[i];
		cmd_str += "\t" + get_speedj_str(qd, a, t) + "\n";
	}
	a = a_theta1[q.size() - 1];
	cmd_str += "\t" + get_stopj_str(a) + "\n";
	cmd_str += "end\n";
	
	return cmd_str;
}


//linearWithParabola与此函数的不同之处在于:\
1.前者是先确定过渡段加速度，由加速度确定过渡段时间；此函数则是采用启发算法确定过渡段时间，然后由过渡段时间确定过渡段加速度:ai=△(v)/ti\
2.前者目前只完成了针对单关节的轨迹规划，此函数完成了6轴轨迹规划\
3.前者针对过渡段加速度设置过小，导致直线段时间<=0的情况只做报错处理，而此函数经过check_timeSeqWith_amax函数对td的修正，使得main_axis的加速度<=amax\
4.前者的设计目标是测试speedj函数是否能完成预期的“带抛物线的直线插补”的轨迹规划，因此前者可以更多的用来测试speedj的特性，并调整算法使得单轴运动更加符合我们的预期。\
功能上来说：此函数完成了针对6轴机械臂的“带抛物线的直线插补”的轨迹规划，并返回speedj控制脚本
string linearWithParabola_multiAxis(double v_cartesian,double amax) {
	vector<double> td = get_timeSeqWith_constantVcartesian(v_cartesian);
	td = check_timeSeqWith_amax(td, amax);

	/*临时：将td手动赋值*/
	//td.resize(2);
	td[0] = 0.5;
	for (int i = 1; i < td.size()-1; i++) {
		td[i] = 0.032;
	}
	td[td.size() - 1] = 0.064;
	ofstream td_modified("C:/Users/XY/Desktop/path/trajectoryPlanning/td_modifyed.txt");
	checkfp(td_modified);
	for (int i = 0; i < td.size(); i++) {
		td_modified << td[i] << endl;
	}
	td_modified.close();
	/*临时结束*/

	ofstream ofp_a_6axis("C:/Users/XY/Desktop/path/trajectoryPlanning/a_6axis.txt");//用以存放各时刻点对应的“6个轴对应的加速度”
	checkfp(ofp_a_6axis);

	int i_temp;
	double d_temp;
	vector<double> vd_temp;//vector<double>的临时变量

	/*read IKresult into q*/
	string qfile_addr = "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt";
	i_temp = getLineNumber(qfile_addr, "");
	assert(i_temp - 1 == td.size());
	ifstream qfile(qfile_addr);
	checkfp(qfile);

	vector<vector<double>> q(i_temp);
	for (int i = 0; i < i_temp; i++) {
		q[i].resize(6);
		for (int j = 0; j < 6; j++) {
			qfile >> q[i][j];
		}
	}
	qfile.close();

	//计算直线段速度，对于q.size个点的路径，有q.size-1个直线段
	vector<vector<double>> v(q.size() - 1);
	for (int j = 0; j < q.size()-1; j++) {//j点
		v[j].resize(6);
		for (int i = 0; i < 6; i++) {
			v[j][i] = (q[j + 1][i] - q[j][i]) / td[j];
		}
	}
	
	//计算主轴加速度,每个点都有一对应加速度，因此有q.size个主轴加速度
	vector<double> a(q.size());
	vector<double> main_axis_index(q.size());//记录每次求a时，对应的主轴下标
	vd_temp.resize(6);//当前点下各轴加速度
	d_temp = 0;//当前点下各轴加速度中的最大值，作为主轴加速度
	i_temp = 0;//当前点下各轴加速度中的最大值的下标
	//始点
	for (int i = 0; i < 6; i++) {//cal a
		vd_temp[i] = CAL_A(0, q[0][i], q[1][i], 0, td[0]);
	}
	d_temp = 0;
	i_temp = 0;
	for (int i = 0; i < 6; i++) {//find max
		if (d_temp < absd(vd_temp[i])) {
			i_temp = i;
			d_temp = absd(vd_temp[i]);
		}
	}
	assert(d_temp <= amax+0.1);//+0.1以防止d_temp略大于amax而报错
	a[0] = vd_temp[i_temp];
	main_axis_index[0] = i_temp;
	ofp_a_6axis << vd_temp[0] << " " << vd_temp[1] << " " << vd_temp[2] << " " << vd_temp[3] << " " << vd_temp[4] << " " << vd_temp[5] << endl;
	//
	//中间点
	for (int j = 1; j < q.size() - 1; j++) {
		for (int i = 0; i < 6; i++) {
			vd_temp[i] = CAL_A(q[j - 1][i], q[j][i], q[j + 1][i], td[j - 1], td[j]);
		}
		d_temp = 0;
		i_temp = 0;
		for (int i = 0; i < 6; i++) {
			if (d_temp < absd(vd_temp[i])) {
				i_temp = i;
				d_temp = absd(vd_temp[i]);
			}
		}
		assert(d_temp <= amax + 0.1);
		a[j] = vd_temp[i_temp];
		main_axis_index[j] = i_temp;
		ofp_a_6axis << vd_temp[0] << " " << vd_temp[1] << " " << vd_temp[2] << " " << vd_temp[3] << " " << vd_temp[4] << " " << vd_temp[5] << endl;
	}
	//末点
	for (int i = 0; i < 6; i++) {//cal a
		vd_temp[i] = CAL_A(q[q.size() - 2][i], q[q.size() - 1][i], 0, td[q.size() - 2], 0);
	}
	d_temp = 0;
	i_temp = 0;
	for (int i = 0; i < 6; i++) {//find max
		if (d_temp < absd(vd_temp[i])) {
			i_temp = i;
			d_temp = absd(vd_temp[i]);
		}
	}
	assert(d_temp <= amax + 0.1);
	a[q.size() - 1] = vd_temp[i_temp];
	main_axis_index[q.size() - 1] = i_temp;
	ofp_a_6axis << vd_temp[0] << " " << vd_temp[1] << " " << vd_temp[2] << " " << vd_temp[3] << " " << vd_temp[4] << " " << vd_temp[5] << endl;
	ofp_a_6axis.close();
	
	//计算过渡段时间:ti=|△v/ai|
	//每个点都有一对应过渡段。特别的，对于始点和末点，其过渡段负责从0加速至v01 以及 从v(n-1)(n)减速至0
	//对于多轴运动来讲，各轴在同一时刻点的过渡段时间是相等的，因此我们可以计算出任意关节对应的过渡段时间，来作为此时刻点对应的过渡段时间。此中，我们计算主轴对应的过渡段时间。\
	  进一步的，由于过渡段时间相等，因此直线段时间也相等，也只需求关节1的直线段时间。
	vector<double> t(q.size());
	t[0] = absd((v[0][main_axis_index[0]] - 0) / a[0]);//始
	t[q.size() - 1] = absd((0 - v[q.size() - 2][main_axis_index[q.size() - 1]]) / a[q.size() - 1]);//末
	for (int i = 1; i < q.size() - 1; i++) {
		if (a[i] == 0) {
			t[i] = 0.25 * (td[i] + td[i - 1]);
		}
		else {
			t[i] = absd((v[i][main_axis_index[i]] - v[i - 1][main_axis_index[i]]) / a[i]);
		}
	}

	//计算直线段时间
	vector<double> tl(q.size() - 1);
	for (int i = 0; i < q.size() - 1; i++) {
		tl[i] = 2 * INSPIRE_FACTOR * td[i];
	}

	/*生成speedj脚本*/
	//now we have
	//q[0][...]--0点各轴角度
	//v[0][...]--01直线段各轴速度
	//a[0]--0点的过渡段加速度
	//t[0]--0点的过渡段时间。值得注意的是，始末点的过渡段局限于一路径段。中间点的过渡段则是分布于中心点两边的路径段。
	//tl[0]--01直线段时间
	write_IntoFile_imp(q, v, a, t, tl);

	//每个点对应一个speedj命令，如点0的speedj: 以加速度a[0]加速至v[0][...]，然后匀速运动，整个过程的持续时间为t = t[0] + tl[0]
	//特别的，对于末点n = q.size()-1，其对应的命令为stopj: stopj的加速度为a = a[q.size()-1]

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




/*
Function: 对由get_timeSeqWith_constantVcartesian得到的td(相邻点的时间间隔序列:tdij=DisOF2p/v)进行检查与修正
i:待检查与修正的vector<double> td；触发修正的amax
o:使得a<=amax的td序列
m :首先结合td序列和IK解(各关节轴的角度序列),计算出6个关节在一相同时刻td[i]对应的6个加速度a(a.size=6),取最大的a。若a<=amax，则继续计算下一时刻；若a>amax,则以一定比例系数调整td(i-1)(i)和td(i)(i+1),使得a=amax,然后继续下一个
*/
vector<double> check_timeSeqWith_amax(vector<double> td,double amax) {
	double d_temp;
	double i_temp;

	/*read IKresult into q*/
	string qfile_addr = "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt";
	i_temp = getLineNumber(qfile_addr, "");
	assert(i_temp - 1 == td.size());
	ifstream qfile(qfile_addr);
	checkfp(qfile);

	vector<vector<double>> q(i_temp);
	for (int i = 0; i < i_temp; i++) {
		q[i].resize(6);
		for (int j = 0; j < 6; j++) {
			qfile >> q[i][j];
		}
	}
	qfile.close();
	
	/*检查与修正:计算同一时刻下，各关节轴对应的加速度a,选取最大的a并与amax进行比较*/
	vector<double> a(6);
	double k, k2;//k2:td(i-1)(i)和td(i)(i+1)的放缩系数
	//始点
	for (int i = 0; i < 6; i++) {//cal a
		a[i] = CAL_A(0, q[0][i], q[1][i], 0, td[0]);
	}
	d_temp = 0;
	for (int i = 0; i < 6; i++) {//find max
		if (d_temp < absd(a[i])) {
			d_temp = absd(a[i]);
		}
	}
	if (d_temp > amax) {//check and modify
		k = amax / d_temp;
		k2 = sqrt(1 / k);
		td[0] = k2 * td[0];
	}
	//中间点
	for (int j = 1; j < q.size()-1; j++) {
		for (int i = 0; i < 6; i++) {
			a[i] = CAL_A(q[j - 1][i], q[j][i], q[j + 1][i], td[j - 1], td[j]);
		}
		d_temp = 0;
		for (int i = 0; i < 6; i++) {
			if (d_temp < absd(a[i])) {
				d_temp = absd(a[i]);
			}
		}
		if (d_temp > amax) {
			k = amax / d_temp;
			k2 = sqrt(1 / k);
			td[j - 1] = k2 * td[j - 1];
			td[j] = k2 * td[j];
		}
	}
	//末点
	for (int i = 0; i < 6; i++) {//cal a
		a[i] = CAL_A(q[q.size() - 2][i], q[q.size() - 1][i], 0, td[q.size() - 2], 0);
	}
	d_temp = 0;
	for (int i = 0; i < 6; i++) {//find max
		if (d_temp < absd(a[i])) {
			d_temp = absd(a[i]);
		}
	}
	if (d_temp > amax) {//check and modify
		k = amax / d_temp;
		k2 = sqrt(1 / k);
		td[q.size() - 2] = k2 * td[q.size() - 2];
	}
	
	ofstream td_modified("C:/Users/XY/Desktop/path/trajectoryPlanning/td_modifyed.txt");
	checkfp(td_modified);
	for (int i = 0; i < td.size(); i++) {
		td_modified << td[i] << endl;
	}

	td_modified.close();
	return td;
}



//在速度v下，根据末端相邻点距，求得linearWithParabola的形参td: t=d/v
vector<double> get_timeSeqWith_constantVcartesian(double v) {
	int i_temp=0;
	/*read pfile into p*/
	string pfile_addr = "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated.txt";
	i_temp = getLineNumber(pfile_addr, "");
	ifstream pfile(pfile_addr);
	checkfp(pfile);

	vector<vector<double>> p(i_temp);
	for (int i = 0; i < p.size(); i++) {
		p[i].resize(3);
		for (int j = 0; j < 3; j++) {
			pfile >> p[i][j];
		}
	}
	pfile.close();
	/*get td*/
	vector<double> td(p.size() - 1);//对于一有p.size个点的路径，其对应的，有p.size-1个路径段
	for (int i = 0; i < p.size()-1; i++) {
		td[i] = calDisOF_2points(p[i], p[i + 1]) / v;
	}

	return td;
}

/*
格式:
q[0]
v[0]:q.size-1个
a[0]
t[0]
tl[0]:q.size-1个 

*/
void write_IntoFile_imp(vector<double> q, vector<double> v, vector<double> a, vector<double> t, vector<double> tl) {
	ofstream ofp("C:/Users/XY/Desktop/path/trajectoryPlanning/imp.txt");
	checkfp(ofp);
	for (int i = 0; i < q.size()-1; i++) {
		ofp << q[i] << endl;
		ofp << v[i] << endl;
		ofp << a[i] << endl;
		ofp << t[i] << endl;
		ofp<< tl[i] << endl;
	}
	ofp << q[q.size()-1] << endl;
	ofp << 0 << endl;
	ofp << a[q.size() - 1] << endl;
	ofp << t[q.size() - 1] << endl;
	ofp << 0 << endl;
	ofp.close();
	return;
}
void write_IntoFile_imp(vector<vector<double>> q, vector<vector<double>> v, vector<double> a, vector<double> t,vector<double> tl) {
	ofstream ofp("C:/Users/XY/Desktop/path/trajectoryPlanning/imp.txt");
	checkfp(ofp);

	int axis_num = 6;

	for (int i = 0; i < q.size()-1; i++) {
		ofp << q[i][0] << " " << q[i][1] << " " << q[i][2] << " " << q[i][3] << " " << q[i][4] << " " << q[i][5] << endl;

		ofp << v[i][0] << " " << v[i][1] << " " << v[i][2] << " " << v[i][3] << " " << v[i][4] << " " << v[i][5] << endl;

		ofp << a[i] << endl;

		ofp << t[i] << endl;

		ofp << tl[i] << endl;
	}

	ofp << q[q.size() - 1][0] << " " << q[q.size() - 1][1] << " " << q[q.size() - 1][2] << " " << q[q.size() - 1][3] << " " << q[q.size() - 1][4] << " " << q[q.size() - 1][5] << endl;
	ofp << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << endl;
	ofp << a[q.size() - 1] << endl;
	ofp << t[q.size() - 1] << endl;
	ofp << 0 << endl;
	ofp.close();
	return;
}
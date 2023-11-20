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

const double INSPIRE_FACTOR=0.25;//���ɹ��ɶ�ʱ��������㷨���ӣ����ڡ�CAL_A�����еĹ��ɶ�ʱ����㡱�͡�linearWithParabola_multiAxis�����е�ֱ�߶�ʱ����㡱��

//�����i�����Ӧ�Ĺ��ɶμ��ٶ�,���й��ɶ�ʱ����������㷨:ti=1/4*t(i-1)(i)+1/4*t(i)(i+1)��ʼĩ�㵥�����ǣ���
//theim1:theta(i-1)
double CAL_A(double theim1, double thei, double theip1, double tdim1, double tdi) {
	assert((tdi == 0 && tdim1 == 0) == 0);
	if (tdim1 == 0) {//ʼ��
		return ((theip1 - thei) / tdi - 0) / (INSPIRE_FACTOR * tdi);
	}
	else if (tdi == 0) {//ĩ��
		return (0 - (thei - theim1) / tdim1) / (INSPIRE_FACTOR * tdim1);
	}
	else {//�м��
		return ((theip1 - thei) / tdi - (thei - theim1) / tdim1) / (INSPIRE_FACTOR * tdi + INSPIRE_FACTOR * tdim1);
	}
}

vector<double> get_timeSeqWith_constantVcartesian(double v);
vector<double> check_timeSeqWith_amax(vector<double> td, double amax);
void write_IntoFile_imp(vector<double> q, vector<double> v, vector<double> a, vector<double> t, vector<double> tl);
void write_IntoFile_imp(vector<vector<double>> q, vector<vector<double>> v, vector<double> a, vector<double> t, vector<double> tl);


/*
Function:  �ؽڿռ��µĵ����ؽڵġ����������߹��ɵ����Ժ������켣�滮����
i: tdij��i��j���ʱ��,���ڹ��ɶμ��ٶȴ�С���趨��һ�������ʹ��anormal��
o: �켣�滮��Ӧ��speedj�ű�
m:
1.���ڹ��ɶμ��ٶȴ�С���趨��һ�������ʹ��anormal����amormal�����󣬿�ʹ��amax
2.��Ե����ؽڵ���ת�����˹켣�滮���������IK���еĹؽ�1�����˹켣�滮��
3.ʹ�÷���:���ȵ���get_timeSeqWith_constantVcartesian(v)�õ�td���ٵ��ô˺���
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
	//����ֱ�߶��ٶȣ�����q.size�����·������q.size-1��ֱ�߶�
	//v_theta1[1]: v12
	vector<double> v_theta1(q.size() - 1);
	for (int i = 0; i < q.size()-1; i++) {
		v_theta1[i] = (q_theta1[i + 1] - q_theta1[i]) / td[i];
	}
	//������ٶ�
	vector<double> a_theta1(q.size());
	a_theta1[0] = SNG(v_theta1[0] - 0) * anormal;//ʼ
	a_theta1[q.size() - 1] = SNG(0 - v_theta1[q.size() - 2]) * anormal;//ĩ
	for (int i = 1; i < q.size() - 1; i++) {
		a_theta1[i] = SNG(v_theta1[i] - v_theta1[i - 1]) * anormal;
	}
	//������ɶ�ʱ��
	//ÿ���㶼��һ��Ӧ���ɶΡ��ر�ģ�����ʼ���ĩ�㣬����ɶθ����0������v01 �Լ� ��v(n-1)(n)������0
	vector<double> t_theta1(q.size());
	t_theta1[0] = absd((v_theta1[0] - 0) / a_theta1[0]);//ʼ
	t_theta1[q.size() - 1] = absd((0 - v_theta1[q.size() - 2]) / a_theta1[q.size() - 1]);//ĩ
	for (int i = 1; i < q.size()-1; i++) {
		t_theta1[i] = absd((v_theta1[i] - v_theta1[i - 1]) / a_theta1[i]);
	}
	//����ֱ�߶�ʱ��
	//tl_theta1[1]: time of line segment 12
	vector<double> tl_theta1(q.size() - 1);//time of line segment
	tl_theta1[0] = td[0] - t_theta1[0] - 0.5 * t_theta1[1];//ʼ tline01
	tl_theta1[q.size() - 2] = td[q.size() - 2] - t_theta1[q.size() - 1] - 0.5 * t_theta1[q.size() - 2];//ĩ tline(n-1)(n)
	for (int i = 1; i < q.size()-2; i++) {
		tl_theta1[i] = td[i] - 0.5 * t_theta1[i] - 0.5 * t_theta1[i + 1];
	}
	for (int i = 0; i < q.size()-1; i++) {
		assert(tl_theta1[i] >= 0);//�������㣬���ʾ���ٶȲ�����
	}
	/*����speedj�ű�*/
	//now we have
	//q_theta1[0]--0��Ƕ�
	//v_theta1[0]--01ֱ�߶��ٶ�
	//a_theta1[0]--0��Ĺ��ɶμ��ٶ�
	//t_theta1[0]--0��Ĺ��ɶ�ʱ�䡣ֵ��ע����ǣ�ʼĩ��Ĺ��ɶξ�����һ·���Ρ��м��Ĺ��ɶ����Ƿֲ������ĵ����ߵ�·���Ρ�
	//tl_theta1[0]--01ֱ�߶�ʱ��
	write_IntoFile_imp(q_theta1, v_theta1, a_theta1, t_theta1, tl_theta1);

	//ÿ�����Ӧһ��speedj������0��speedj: �Լ��ٶ�a_theta[0]������v_theta[0]��Ȼ�������˶����������̵ĳ���ʱ��Ϊt = t_theta[0] + tl_theta[0]
	//�ر�ģ�����ĩ��n = q.size()-1�����Ӧ������Ϊstopj: stopj�ļ��ٶ�Ϊa = a_theta[q.size()-1]

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


//linearWithParabola��˺����Ĳ�֮ͬ������:\
1.ǰ������ȷ�����ɶμ��ٶȣ��ɼ��ٶ�ȷ�����ɶ�ʱ�䣻�˺������ǲ��������㷨ȷ�����ɶ�ʱ�䣬Ȼ���ɹ��ɶ�ʱ��ȷ�����ɶμ��ٶ�:ai=��(v)/ti\
2.ǰ��Ŀǰֻ�������Ե��ؽڵĹ켣�滮���˺��������6��켣�滮\
3.ǰ����Թ��ɶμ��ٶ����ù�С������ֱ�߶�ʱ��<=0�����ֻ�����������˺�������check_timeSeqWith_amax������td��������ʹ��main_axis�ļ��ٶ�<=amax\
4.ǰ�ߵ����Ŀ���ǲ���speedj�����Ƿ������Ԥ�ڵġ��������ߵ�ֱ�߲岹���Ĺ켣�滮�����ǰ�߿��Ը������������speedj�����ԣ��������㷨ʹ�õ����˶����ӷ������ǵ�Ԥ�ڡ�\
��������˵���˺�����������6���е�۵ġ��������ߵ�ֱ�߲岹���Ĺ켣�滮��������speedj���ƽű�
string linearWithParabola_multiAxis(double v_cartesian,double amax) {
	vector<double> td = get_timeSeqWith_constantVcartesian(v_cartesian);
	td = check_timeSeqWith_amax(td, amax);

	/*��ʱ����td�ֶ���ֵ*/
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
	/*��ʱ����*/

	ofstream ofp_a_6axis("C:/Users/XY/Desktop/path/trajectoryPlanning/a_6axis.txt");//���Դ�Ÿ�ʱ�̵��Ӧ�ġ�6�����Ӧ�ļ��ٶȡ�
	checkfp(ofp_a_6axis);

	int i_temp;
	double d_temp;
	vector<double> vd_temp;//vector<double>����ʱ����

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

	//����ֱ�߶��ٶȣ�����q.size�����·������q.size-1��ֱ�߶�
	vector<vector<double>> v(q.size() - 1);
	for (int j = 0; j < q.size()-1; j++) {//j��
		v[j].resize(6);
		for (int i = 0; i < 6; i++) {
			v[j][i] = (q[j + 1][i] - q[j][i]) / td[j];
		}
	}
	
	//����������ٶ�,ÿ���㶼��һ��Ӧ���ٶȣ������q.size��������ٶ�
	vector<double> a(q.size());
	vector<double> main_axis_index(q.size());//��¼ÿ����aʱ����Ӧ�������±�
	vd_temp.resize(6);//��ǰ���¸�����ٶ�
	d_temp = 0;//��ǰ���¸�����ٶ��е����ֵ����Ϊ������ٶ�
	i_temp = 0;//��ǰ���¸�����ٶ��е����ֵ���±�
	//ʼ��
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
	assert(d_temp <= amax+0.1);//+0.1�Է�ֹd_temp�Դ���amax������
	a[0] = vd_temp[i_temp];
	main_axis_index[0] = i_temp;
	ofp_a_6axis << vd_temp[0] << " " << vd_temp[1] << " " << vd_temp[2] << " " << vd_temp[3] << " " << vd_temp[4] << " " << vd_temp[5] << endl;
	//
	//�м��
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
	//ĩ��
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
	
	//������ɶ�ʱ��:ti=|��v/ai|
	//ÿ���㶼��һ��Ӧ���ɶΡ��ر�ģ�����ʼ���ĩ�㣬����ɶθ����0������v01 �Լ� ��v(n-1)(n)������0
	//���ڶ����˶�������������ͬһʱ�̵�Ĺ��ɶ�ʱ������ȵģ�������ǿ��Լ��������ؽڶ�Ӧ�Ĺ��ɶ�ʱ�䣬����Ϊ��ʱ�̵��Ӧ�Ĺ��ɶ�ʱ�䡣���У����Ǽ��������Ӧ�Ĺ��ɶ�ʱ�䡣\
	  ��һ���ģ����ڹ��ɶ�ʱ����ȣ����ֱ�߶�ʱ��Ҳ��ȣ�Ҳֻ����ؽ�1��ֱ�߶�ʱ�䡣
	vector<double> t(q.size());
	t[0] = absd((v[0][main_axis_index[0]] - 0) / a[0]);//ʼ
	t[q.size() - 1] = absd((0 - v[q.size() - 2][main_axis_index[q.size() - 1]]) / a[q.size() - 1]);//ĩ
	for (int i = 1; i < q.size() - 1; i++) {
		if (a[i] == 0) {
			t[i] = 0.25 * (td[i] + td[i - 1]);
		}
		else {
			t[i] = absd((v[i][main_axis_index[i]] - v[i - 1][main_axis_index[i]]) / a[i]);
		}
	}

	//����ֱ�߶�ʱ��
	vector<double> tl(q.size() - 1);
	for (int i = 0; i < q.size() - 1; i++) {
		tl[i] = 2 * INSPIRE_FACTOR * td[i];
	}

	/*����speedj�ű�*/
	//now we have
	//q[0][...]--0�����Ƕ�
	//v[0][...]--01ֱ�߶θ����ٶ�
	//a[0]--0��Ĺ��ɶμ��ٶ�
	//t[0]--0��Ĺ��ɶ�ʱ�䡣ֵ��ע����ǣ�ʼĩ��Ĺ��ɶξ�����һ·���Ρ��м��Ĺ��ɶ����Ƿֲ������ĵ����ߵ�·���Ρ�
	//tl[0]--01ֱ�߶�ʱ��
	write_IntoFile_imp(q, v, a, t, tl);

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




/*
Function: ����get_timeSeqWith_constantVcartesian�õ���td(���ڵ��ʱ��������:tdij=DisOF2p/v)���м��������
i:�������������vector<double> td������������amax
o:ʹ��a<=amax��td����
m :���Ƚ��td���к�IK��(���ؽ���ĽǶ�����),�����6���ؽ���һ��ͬʱ��td[i]��Ӧ��6�����ٶ�a(a.size=6),ȡ����a����a<=amax�������������һʱ�̣���a>amax,����һ������ϵ������td(i-1)(i)��td(i)(i+1),ʹ��a=amax,Ȼ�������һ��
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
	
	/*���������:����ͬһʱ���£����ؽ����Ӧ�ļ��ٶ�a,ѡȡ����a����amax���бȽ�*/
	vector<double> a(6);
	double k, k2;//k2:td(i-1)(i)��td(i)(i+1)�ķ���ϵ��
	//ʼ��
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
	//�м��
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
	//ĩ��
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



//���ٶ�v�£�����ĩ�����ڵ�࣬���linearWithParabola���β�td: t=d/v
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
	vector<double> td(p.size() - 1);//����һ��p.size�����·�������Ӧ�ģ���p.size-1��·����
	for (int i = 0; i < p.size()-1; i++) {
		td[i] = calDisOF_2points(p[i], p[i + 1]) / v;
	}

	return td;
}

/*
��ʽ:
q[0]
v[0]:q.size-1��
a[0]
t[0]
tl[0]:q.size-1�� 

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
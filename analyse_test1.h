/*
Function:  ����test1.txt�ļ����õ�����ֱ���νӴ�
i
o
m
*/
#pragma once
#include <iostream>
#include <Eigen/Dense>
#include<math.h>
#include<fstream>
#include<vector>
#include "trans.h"
#include"Point.h"
using namespace Eigen;
using namespace std;

#define TEST1_FILE_ADDR "C:/Users/XY/Desktop/path/test1_afterHandle.txt"
#define LINE_NUMBER_OF_TEST1 209776
#define TEST1_AFTER_ANALYSED_FILE_ADDR "C:/Users/XY/Desktop/path/test1.FULLanalysed.txt"
#define LINE_NUMBER_OF_TEST1_AFTER_ANALYSED 8299

double normOFvector(Vector3d v1);//����ģ��
double CalAngleOFvector(Vector3d v1, Vector3d v2);//���������н�
Vector3d calNormline(Point_Struct& s, Point_Struct& e);//�õ���λģ������
typedef enum { L, C } LINETYPE;

void analyse_test1() {
	ifstream test1_fp;
	ofstream fp;//test1.analysed.txt
	test1_fp.open(TEST1_FILE_ADDR);
	fp.open(TEST1_AFTER_ANALYSED_FILE_ADDR);

	/*read into ram*/
	vector<Vector3d> p;//test1�ڵ����е����p
	p.resize(LINE_NUMBER_OF_TEST1);
	for (int i = 0; i < LINE_NUMBER_OF_TEST1; i++) {
		test1_fp >> p[i][0] >> p[i][1] >> p[i][2];
	}

	Vector3d vs;
	Vector3d ve;
	LINETYPE statusCur = L;
	fp << "L" << endl;
	fp << p[0][0] << " " << p[0][1] << " " << p[0][2] << endl;
	for (int i = 1; i < LINE_NUMBER_OF_TEST1 - 2; i++) {
		vs = p[i + 1] - p[i];
		ve = p[i + 2] - p[i + 1];
		if (CalAngleOFvector(vs, ve) == 0) {
			if (statusCur == L) {
				continue;
			}
			else if (statusCur == C) {
				fp << p[i][0] << " " << p[i][1] << " " << p[i][2] << endl;
				fp << p[i + 1][0] << " " << p[i + 1][1] << " " << p[i + 1][2] << endl;//end of C
				statusCur = L;
				fp << "L" << endl;
				fp << p[i][0] << " " << p[i][1] << " " << p[i][2] << endl;//start of L
			}
			else
			{
				cout << "ERROR IN analyse_test1 _" << __LINE__ << ": status is not L neither C" << endl;
			}
		}
		else {//Angle��0
			if (statusCur == L) {
				fp << p[i + 1][0] << " " << p[i + 1][1] << " " << p[i + 1][2] << endl;//end of L
				statusCur = C;
				fp << "C" << endl;
				fp << p[i][0] << " " << p[i][1] << " " << p[i][2] << endl;//start of C
			}
			else if (statusCur == C) {
				fp << p[i][0] << " " << p[i][1] << " " << p[i][2] << endl;
			}
			else {
				cout << "ERROR IN analyse_test1 _" << __LINE__ << ": status is not L neither C" << endl;
			}
		}
	}

	if (statusCur == L) {//�켣�����Ϊһ��ֱ��
		fp << p[LINE_NUMBER_OF_TEST1 - 1][0] << " " << p[LINE_NUMBER_OF_TEST1 - 1][1] << " " << p[LINE_NUMBER_OF_TEST1 - 1][2] << endl;//end of L
	}
	if (statusCur == C) {
		fp << p[LINE_NUMBER_OF_TEST1 - 2][0] << " " << p[LINE_NUMBER_OF_TEST1 - 2][1] << " " << p[LINE_NUMBER_OF_TEST1 - 2][2] << endl;
		fp << p[LINE_NUMBER_OF_TEST1 - 1][0] << " " << p[LINE_NUMBER_OF_TEST1 - 1][1] << " " << p[LINE_NUMBER_OF_TEST1 - 1][2] << endl;//end of C
	}
	test1_fp.close();
	fp.close();
}

string generateCodeForTest1() {
	vector<Point_Struct> s;//start point of a line
	vector<Point_Struct> e;
	Point_Struct ptemp;
	string linetype;
	ifstream fp;//test1.analysed.txt
	fp.open(TEST1_AFTER_ANALYSED_FILE_ADDR);

	/*read into ram*/
	int lineAlreadyRead = 0;//�Ѷ�����
	while (lineAlreadyRead < LINE_NUMBER_OF_TEST1_AFTER_ANALYSED) {
		do {//till read L
			getline(fp, linetype);
			lineAlreadyRead++;
		} while (linetype != "L" && lineAlreadyRead <= LINE_NUMBER_OF_TEST1_AFTER_ANALYSED);

		if (lineAlreadyRead > LINE_NUMBER_OF_TEST1_AFTER_ANALYSED) {
			break;
		}
		fp >> ptemp.x >> ptemp.y >> ptemp.z;
		s.push_back(ptemp);
		fp >> ptemp.x >> ptemp.y >> ptemp.z;
		e.push_back(ptemp);
		lineAlreadyRead += 2;
		getline(fp, linetype);//��ʱָ��λ�ڵ�ǰ�е�ĩβ����ʱʹ��getline�õ�����""����������ڶ����֮��ͨ��getline��ָ���ƶ�����һ������
	}
	fp.close();
	/*�õ�movel�ĸ�������*/
	vector<Point_Struct> pset;
	Vector3d lineb, linec;//line before,line current
	lineb = calNormline(s[0], e[0]);//ֱ��0�ĵ�λʸ��
	for (int i = 1; i < e.size(); i++) {//�󡰵�i��ֱ�ߵ�ͷ���͡���i-1��ֱ�ߵ�β���������ĸ�λ����ΪĿ���
		linec = calNormline(s[i], e[i]);
		if (e[i - 1].z == s[i].z) {//��ֱ�ߵĹ�����û�г��֡���z��������
			if (lineb == Vector3d(1, 0, 0)) {
				if (linec == Vector3d(0, 1, 0) || linec == Vector3d(0, -1, 0)) {//1 ˳ʱ�� || ��ʱ��
					ptemp.x = s[i].x;
					ptemp.y = e[i - 1].y;
					ptemp.z = s[i].z;
					pset.push_back(ptemp);
				}
				else if (linec == Vector3d(-1, 0, 0)) {	//6581
					pset.push_back(e[i - 1]);
					pset.push_back(s[i]);
				}
				else {
					cout << "ERROR IN generateCodeForTest1 _" << __LINE__ << ": δ���Ǵ����" << endl;
				}
			}
			else if (lineb == Vector3d(0, 1, 0)) {
				if (linec == Vector3d(1, 0, 0) || linec == Vector3d(-1, 0, 0)) {//1
					ptemp.x = e[i - 1].x;
					ptemp.y = s[i].y;
					ptemp.z = s[i].z;
					pset.push_back(ptemp);
				}
				else {
					cout << "ERROR IN generateCodeForTest1 _" << __LINE__ << ": δ���Ǵ����" << endl;
				}
			}
			else if (lineb == Vector3d(-1, 0, 0)) {
				if (linec == Vector3d(0, -1, 0) || linec == Vector3d(0, 1, 0)) {//1 ˳ʱ�� || ��ʱ��
					ptemp.x = s[i].x;
					ptemp.y = e[i - 1].y;
					ptemp.z = s[i].z;
					pset.push_back(ptemp);
				}
				else if (linec == Vector3d(1, 0, 0)) {	//6991
					pset.push_back(e[i - 1]);
					pset.push_back(s[i]);
				}
				else {
					cout << "ERROR IN generateCodeForTest1 _" << __LINE__ << ": δ���Ǵ����" << endl;
				}
			}
			else if (lineb == Vector3d(0, -1, 0)) {
				if (linec == Vector3d(1, 0, 0) || linec == Vector3d(-1, 0, 0)) {//1
					ptemp.x = e[i - 1].x;
					ptemp.y = s[i].y;
					ptemp.z = s[i].z;
					pset.push_back(ptemp);
				}
				else {
					cout << "ERROR IN generateCodeForTest1 _" << __LINE__ << ": δ���Ǵ����" << endl;
				}
			}
			else {
				cout << "ERROR IN generateCodeForTest1 _" << __LINE__ << ": δ���Ǵ����" << endl;
			}
		}
		else {//b.e.z��c.s.z
			pset.push_back(e[i - 1]);
			pset.push_back(s[i]);
		}
		lineb = linec;
	}
	pset.push_back(e[e.size() - 1]);//���һ��ֱ�ߵ�ĩβ

	//if (s.size() % 2) {//������
	//	for (int i = 0; i < s.size()-1; i=i++) {
	//		ptemp.x = s[i + 1].x;
	//		ptemp.y = e[i].y;
	//		ptemp.z = e[i].z;
	//		pset.push_back(ptemp);
	//		i++;
	//		ptemp.x = e[i].x;
	//		ptemp.y = s[i+1].y;
	//		ptemp.z = e[i].z;
	//		pset.push_back(ptemp);
	//	}
	//	pset.push_back(e[s.size() - 1]);
	//}
	//else {//ż����
	//	for (int i = 0; i < s.size()-2; i++) {
	//		ptemp.x = s[i + 1].x;
	//		ptemp.y = e[i].y;
	//		ptemp.z = e[i].z;
	//		pset.push_back(ptemp);
	//		i++;
	//		ptemp.x = e[i].x;
	//		ptemp.y = s[i + 1].y;
	//		ptemp.z = e[i].z;
	//		pset.push_back(ptemp);
	//	}
	//	ptemp.x = s[s.size()-1].x;
	//	ptemp.y = e[s.size()-2].y;
	//	ptemp.z = e[s.size()-2].z;
	//	pset.push_back(ptemp);
	//	pset.push_back(e[s.size() - 1]);
	//}
	/*���ɴ���*/
	URPose workPose(0.2, -0.1, 0.2, 2.2214, -2.2214, 0);
	URPose urpose;
	urpose = workPose;
	urpose.x += 0.5 * mm2m;
	urpose.y += 0.5 * mm2m;
	urpose.z += 0.3 * mm2m;
	double blendRadius = 1 * mm2m;//���ڰ뾶

	std::string cmd_str;
	char buf[128];
	cmd_str = "def driverProg():\n";
	cmd_str += "\tmovej(p[" + to_string(urpose.x) + "," + to_string(urpose.y) +
		"," + to_string(urpose.z) + "," + to_string(urpose.rx) + "," +
		to_string(urpose.ry) + "," + to_string(urpose.rz) +
		"],a=2,v=2)\n";

	for (int i = 0; i < pset.size(); i++) {
		urpose.x = workPose.x + pset[i].x * mm2m;
		urpose.y = workPose.y + pset[i].y * mm2m;
		urpose.z = workPose.z + pset[i].z * mm2m;
		cmd_str += "\tmovel(p[" + to_string(urpose.x) + "," + to_string(urpose.y) +
			"," + to_string(urpose.z) + "," + to_string(urpose.rx) + "," +
			to_string(urpose.ry) + "," + to_string(urpose.rz) +
			"],a=1.2,v=0.25)\n";
	}
	cmd_str += "end\n";

	return cmd_str;
}


double CalAngleOFvector(Vector3d v1, Vector3d v2) {
	return r2d(atan2(normOFvector(v1.cross(v2)), v1.dot(v2)));
}

double normOFvector(Vector3d v1) {
	return sqrt(v1(0) * v1(0) + v1(1) * v1(1) + v1(2) * v1(2));
}

/*
Function:  ��������ά�㣬�õ���Ӧ��ģ��Ϊ1������
i
o
m
*/
Vector3d calNormline(Point_Struct& s, Point_Struct& e) {
	Vector3d se(e.x - s.x, e.y - s.y, e.z - s.z);
	se = se / normOFvector(se);
	return se;
}
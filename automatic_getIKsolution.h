//X

#pragma once
#include"URPose.h"
#include"ur_kinetic.h"
#include"trans.h"
#include"fileOpration.h"
#include"absd.h"
#include<iostream>
#include<math.h>
#include<vector>
#include<fstream>
#include"getLineNumber.h"
using namespace std;
/*
	Function:  used to trans "file after interploted" into "file composed by sol[q1-q6]".
	��ǰ"Ŀ����̬�ģ�RX,RY,RZ��"��ȷ��Ϊƽ���µ��˶���Ӧ�Ĳ��ԣ��̶�ʹ��workPose�ģ�RX,RY,RZ����
	i
	o
	m��
	*/
void automatic_getIKsolution() {
	string poseFileAddress = "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated.txt";
	string qFileAddress = "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt";
	int lineNum = getLineNumber(poseFileAddress, "");
	/*1.��ʼ������*/
		/*1.1.һ���ɱ�*/
	//int lineNum =609 ;//lineNum of file after interploted
	//string poseFileAddress = "C:/Users/XY/Desktop/path/hand2_interploted.txt";//hand2_interploted.txt
	//string qFileAddress = "C:/Users/XY/Desktop/path/hand2_afterIK.txt";
	/*1.2.�����ɱ�*/
	URPose workPose(0.2, -0.1, 0.1704, 2.2253, -2.2201, -0.0028);//������̬:��ʼ��̬֮ǰ����̬����ʼ��̬Ϊ�㼯��һ��IK��
	//workPose.x += -0.010366667 * mm2m;//ʵ�ʻ�е���л����̶���������
	//workPose.y += -1.18115 * mm2m;
	//workPose.z += 0.678333333 * mm2m;

	URPose targetPose(0, 0, 0, 2.2253, -2.2201, -0.0028);//rx ry rz��ֵΪwork pose��rx ry rz
	vector<double> workq;//workPose��Ӧ��q
	workq.resize(6);//��λΪ��
	workq[0] = 3.535760721;
	workq[1] = -79.16359917;
	workq[2] = -144.6433765;
	workq[3] = -46.18445328;
	workq[4] = 90.14414986;
	workq[5] = 3.401728554;
	//for (int i = 0; i < 6; i++) {
	//	workq[i] = d2r(workq[i]);
	//}

		/*1.3.����Ҫ����ֵ*/
	vector<vector<double>> sol; //solution of IK
	vector<double> lastSol;
	lastSol.resize(6);
	//init sol
	sol.resize(8);
	for (int i = 0; i < 8; i++) {
		sol[i].resize(6);
		for (int j = 0; j < 6; j++) {
			sol[i][j] = 0;
		}
	}
	/*2.�Ե�һ��target pose����IK����*/
		/*2.1.poseFromFileΪ���λ�ã�work Pose + poseFromFile��Ϊ����Ŀ��λ��*/
	URPose poseFromFile;
	fileOpration fp_test(poseFileAddress);
	poseFromFile = fp_test.getNexp();
	targetPose.x = workPose.x + poseFromFile.x * mm2m;
	targetPose.y = workPose.y + poseFromFile.y * mm2m;
	targetPose.z = workPose.z + poseFromFile.z * mm2m;

	/*2.2.cal IK
	����theta6ΪworkPoseʱ"��Ӧ��q��theta6.����֤�����죬ʵ�ʲ�Ӱ��3-D��ӡ�Ľ�6����"*/
	sol = IK(targetPose, workq[5]);

	/*2.3.��sol��ѡ�����workq����λ������Ľ��*/
	double disMin = 2 * PI * 6;
	double indexMin = 0;
	double disMinTemp = 0;
	lastSol = workq;

	/*2.3.1.����һ�����ڼӻ��360�Ⱥ���һ��Ч�ǣ���Ҫ����Ч�ǿ��ǽ�ȥ*/
	vector<double> lastSol2(6);//lastSol�ĵ�Ч��
	for (int i = 0; i < 6; i++) {
		if (lastSol[i] > 0) {
			lastSol2[i] = lastSol[i] - 360;
		}
		else if (lastSol[i] < 0) {
			lastSol2[i] = lastSol[i] + 360;
		}
		else//lastSol[i]==0;
		{
			lastSol2[i] = 360;
		}
	}
	/*2.3.2.�ҵ����뵱ǰ����λ������Ľ������(�ҵ�indexMin)*/
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 6; j++) {
			disMinTemp += min(absd(r2d(sol[i][j]) - lastSol[j]), absd(r2d(sol[i][j]) - lastSol2[j]));
		}
		if (disMin > disMinTemp) {
			disMin = disMinTemp;
			indexMin = i;
		}
		disMinTemp = 0;
	}
	/*2.3.3.���ý���ĸ�ֵ����Ϊ��lastSol�Ͻ��ĵ�Ч�ǡ���(����sol[indexMin])*/
	double equAngle;
	for (int j = 0; j < 6; j++) {
		if (sol[indexMin][j] > 0) {
			equAngle = sol[indexMin][j] - 2 * PI;
			if (absd(r2d(sol[indexMin][j]) - lastSol[j]) > absd(r2d(equAngle) - lastSol[j])) {
				sol[indexMin][j] = equAngle;
			}
		}
		else {//sol[indexMin][j] <= 0
			equAngle = sol[indexMin][j] + 2 * PI;
			if (absd(r2d(sol[indexMin][j]) - lastSol[j]) > absd(r2d(equAngle) - lastSol[j])) {
				sol[indexMin][j] = equAngle;
			}
		}
	}
	/*3.lastSol��ֵ*/
	lastSol = sol[indexMin];
	//put result into qfileAfterIK
	ofstream fpo(qFileAddress);
	if (!fpo) {
		cout << "WARN at interplote:fail to open qfileAfterIK" << endl;
	}
	fpo << sol[indexMin][0] << " " << sol[indexMin][1] << " " << sol[indexMin][2] << " " <<
		sol[indexMin][3] << " " << sol[indexMin][4] << " " << sol[indexMin][5] << endl;

	/*for each target pose*/
	for (int i = 0; i < lineNum - 1; i++) {
		poseFromFile = fp_test.getNexp();
		targetPose.x = workPose.x + poseFromFile.x * mm2m;
		targetPose.y = workPose.y + poseFromFile.y * mm2m;
		targetPose.z = workPose.z + poseFromFile.z * mm2m;

		sol = IK(targetPose, lastSol[5]);

		/*select sol[i] from sol according to lastSol*/
			/*����һ�����ڼӻ��360�Ⱥ���һ��Ч�ǣ���Ҫ����Ч�ǿ��ǽ�ȥ*/
		for (int i = 0; i < 6; i++) {
			if (lastSol[i] > 0) {
				lastSol2[i] = lastSol[i] - 360;
			}
			else if (lastSol[i] < 0) {
				lastSol2[i] = lastSol[i] + 360;
			}
			else//lastSol[i]==0;
			{
				lastSol2[i] = 360;
			}
		}
		/*�ҵ�����lastSol����λ������Ľ������(�ҵ�indexMin)*/
		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 6; j++) {
				disMinTemp += min(absd(r2d(sol[i][j]) - lastSol[j]), absd(r2d(sol[i][j]) - lastSol2[j]));
			}
			if (disMin > disMinTemp) {
				disMin = disMinTemp;
				indexMin = i;
			}
			disMinTemp = 0;
		}
		/*���ý���ĸ�ֵ����Ϊ��lastSol�Ͻ��ĵ�Ч�ǡ���(����sol[indexMin])*/
		double equAngle;
		for (int j = 0; j < 6; j++) {
			if (sol[indexMin][j] > 0) {
				equAngle = sol[indexMin][j] - 2 * PI;
				if (absd(sol[indexMin][j] - lastSol[j]) > absd(equAngle - lastSol[j])) {
					sol[indexMin][j] = equAngle;
				}
			}
			else {//sol[indexMin][j] <= 0
				equAngle = sol[indexMin][j] + 2 * PI;
				if (absd(sol[indexMin][j] - lastSol[j]) > absd(equAngle - lastSol[j])) {
					sol[indexMin][j] = equAngle;
				}
			}
		}

		lastSol = sol[indexMin];
		fpo << sol[indexMin][0] << " " << sol[indexMin][1] << " " << sol[indexMin][2] << " " <<
			sol[indexMin][3] << " " << sol[indexMin][4] << " " << sol[indexMin][5] << endl;
	}

	fpo.close();
	return;
}
//X


/*
Function: ���������
i:
�����󿴵ĳ���(mm)
���岹�㼯�����õļ��(mm)
��Ŀ��㼯�ļ�(file after interploted)��ʵ�ʵ㼯�ļ�(monitorLog.txt)

o:�����=�������/Ŀ�����
m:Ҫ��ȷ�ж��Ƿ񵽴�Ŀ��㣬�жϵ�ʱ����Ҫ���С��ֲ��ԡ�
*/
#pragma once
#include<iostream>
#include<string>
#include<vector>
#include<fstream>
#include<math.h>
#include"trans.h"
#include"Point.h"
#include"getLineNumber.h"

using namespace std;

///*File Addr*/
#define TARGET_POINT_SET_FILE_ADDR "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt"
#define ACTUAL_POINT_SET_FILE_ADDR "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_actual.txt"
#define MONITOR_FILE_ADDR "C:/Users/XY/Desktop/path/monitorLog.txt"

/*Target File Line Number*/
int ACTUAL_FILE_LINE_NUMBER = 0; //monitorLog.txt������/4
int TARGET_FILE_LINE_NUMBER = 0;

/*work pose*/
#define WORK_X 200
#define WORK_Y -100
#define WORK_Z 200

#define	ZERO_THRESH_calReachPercent 0.2//�������<0.2mm����Ϊ�غ�
#define CheckZeroThresh_calReachPercent(a)\
	(abs(a)<ZERO_THRESH_calReachPercent)

double calDis(Point_Struct p1, Point_Struct p2);
void getActual();
vector<double> calReachPercent(double lookAheadDistance = 20, double gap = 1);

vector<double> automatic_calReachPercent() {
	ACTUAL_FILE_LINE_NUMBER = getLineNumber(MONITOR_FILE_ADDR, "") / 4.0;
	TARGET_FILE_LINE_NUMBER = getLineNumber(TARGET_POINT_SET_FILE_ADDR, "");

	getActual();
	return calReachPercent(20, 1);
}

/*
Function:  ����actual.txt��target.txt�ļ�����result[0]��result[1]��ͬʱ��δ��Ŀ���д��Missed.txt
result[0]:������
result[1]:( ��calDis(�����Ӧ��ʵ�ʵ㣬�����Ӧ��Ŀ���) )/������
i
o
m
*/
vector<double> calReachPercent(double lookAheadDistance, double gap) {
	ifstream targetFile, actualFile;
	ofstream missedFile;
	targetFile.open(TARGET_POINT_SET_FILE_ADDR);
	actualFile.open(ACTUAL_POINT_SET_FILE_ADDR);
	missedFile.open("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_Missed.txt");//��¼û�е����Ŀ��������͸�������Ӧ��Ŀ���
	if (targetFile.fail()) {
		cout << "ERROR IN open target file _" << __LINE__ << "" << endl;
		system("pause");
		exit(0);
	}
	if (actualFile.fail()) {
		cout << "ERROR IN open actual file _" << __LINE__ << "" << endl;
		system("pause");
		exit(0);
	}
	if (missedFile.fail()) {
		cout << "ERROR IN open missed file _" << __LINE__ << "" << endl;
		system("pause");
		exit(0);
	}
	vector<Point_Struct> target_pointSet(TARGET_FILE_LINE_NUMBER);
	vector<Point_Struct> actual_pointSet(ACTUAL_FILE_LINE_NUMBER);
	
	/*Read into Point Set into Ram*/
	for (int i = 0; i < ACTUAL_FILE_LINE_NUMBER; i++) {
		actualFile >> actual_pointSet[i].x;
		actualFile >> actual_pointSet[i].y;
		actualFile >> actual_pointSet[i].z;
	}
	for (int i = 0; i < TARGET_FILE_LINE_NUMBER; i++) {
		targetFile >> target_pointSet[i].x;
		targetFile >> target_pointSet[i].y;
		targetFile >> target_pointSet[i].z;
	}
	targetFile.close();
	actualFile.close();

	double unReachCount = 0;//����Ϊdouble����ֹreturn��ʱ����е��Ǵ�int�ʹ���
	int n = lookAheadDistance / gap;//�˳�����n��������ǰհ����ĵ��������������󿴶��ٸ��㡣
	int i = 0, j = 0, curj = 0;//j��ʾ�������λ�ã�curj�ű�ʾ��ǰ�Ѿ��Ƚϵ����ĸ�Ŀ���
	double sumOfDis = 0;//��calDis(�����Ӧ��ʵ�ʵ㣬�����Ӧ��Ŀ���)
	while (i <= ACTUAL_FILE_LINE_NUMBER - 1) {
		/*���ڹ켣���һ��ʣ��Ŀ�����С��ǰհ������ʱ��n��ֵΪʣ����������������켣�� ǰհ��Ŀ��㼯���һ����
		��n-1 ����n-1����
		��curj ����"���Ƚ�Ŀ���"�±�
		��curj+1 ����"���Ƚ�Ŀ���"+����ɱȽϵ�Ŀ��������
		��curj+1-1 ����ɱȽϵ�Ŀ��������*/
		if (curj + n > TARGET_FILE_LINE_NUMBER) {//������"���Ƚ�Ŀ���"+����ɱȽϵ�Ŀ����������+���󿴵ĵ� = (curj+1)+(n-1) > ��Ŀ�����
			n = TARGET_FILE_LINE_NUMBER - curj;
		}
		/*����ʵ�ʵ㼯[i]��ǰհ�θ�����벢�ж��Ƿ��غ�*/
		for (j = 0; j < n && CheckZeroThresh_calReachPercent(calDis(actual_pointSet[i], target_pointSet[curj + j])) == 0; j++);//ֱ���� j to j+n �����ҵ��غϵ�����������j to j+nһ������ƥ�䣬�����˳�
		if (j != n) {//�� j to j+n �����С���ʵ�ʵ㼯[i]�غϵ�Ŀ��㡱
			sumOfDis += calDis(actual_pointSet[i], target_pointSet[curj + j]);//calDis(�����Ӧ��ʵ�ʵ㣬�����Ӧ��Ŀ���)
		}
		i++;
		if (j == n) {//�� j to j+n ����û�С���ʵ�ʵ㼯[i]�غϵ�Ŀ��㡱
			continue;
		}
		else
		{
			unReachCount += j;//����forѭ���ĵ�һ�����У�j=0��ʱ��ͷ����غϣ�˵��û��δ���㡣
			for (int k = 0; k < j; k++) {//curj+jΪ��Ӧ�غϵ㣬��δ����curj+0��curj+j-1д���ļ�
				missedFile << curj + k + 1 << " " << target_pointSet[curj + k].x << " " << target_pointSet[curj + k].y << " " << target_pointSet[curj + k].z << endl;//curj+k��ʾ�±ꡣ����Ϊ�˱�ʾ���������curj+k֮��Ҫ��1��
			}
			curj += j + 1;
		}
		if (curj >= TARGET_FILE_LINE_NUMBER)//��Ŀ����ļ���϶�����
			break;
	}

	if (i > ACTUAL_FILE_LINE_NUMBER - 1) { //��Ϊ������i <= ACTUAL_FILE_LINE_NUMBER���˳�,�账��Ŀ������û�Ƚ���Ĳ��֡�
		unReachCount += (TARGET_FILE_LINE_NUMBER - curj);
		for (int k = 0; k < TARGET_FILE_LINE_NUMBER - curj; k++) {
			missedFile << curj + k + 1 << " " << target_pointSet[curj + k].x << " " << target_pointSet[curj + k].y << " " << target_pointSet[curj + k].z << endl;//curj+k��ʾ�±ꡣ����Ϊ�˱�ʾ���������curj+k֮��Ҫ��1��
		}

	}
	missedFile.close();
	vector<double> result(2);//result[0]Ϊ�����ʣ�result[1]Ϊ( ��calDis(�����Ӧ��ʵ�ʵ㣬�����Ӧ��Ŀ���) )/������
	result[0] = (TARGET_FILE_LINE_NUMBER - unReachCount) / TARGET_FILE_LINE_NUMBER;
	result[1] = sumOfDis / double((TARGET_FILE_LINE_NUMBER - unReachCount));
	return result;
}

/*
Function:  ��monitorLog.txt����õ�ֻ��pose��(x,y,z)��ʵ�ʵ㼯����λΪmm
i
o
m:����file after interploted�еĵ㼯�����λ�ã����Ϊ�໥��Ӧ����monitorLog.txt�е�base�ӽ�תΪwork�ӽ��µ����꣬��ҲΪ���λ��
*/
void getActual() {
	ifstream monitorFile;
	ofstream actualFile;
	monitorFile.open(MONITOR_FILE_ADDR);
	actualFile.open(ACTUAL_POINT_SET_FILE_ADDR);
	if (monitorFile.fail()) {
		cout << "ERROR IN getActual _" << __LINE__ << ": fail to open monitorFile" << endl;
		system("pause");
		exit(0);
	}
	if (actualFile.fail()) {
		cout << "ERROR IN getActual _" << __LINE__ << ": fail to open actual file" << endl;
		system("pause");
		exit(0);
	}
	Point_Struct p1;
	double d1;

	for (int i = 0; i < ACTUAL_FILE_LINE_NUMBER; i++) {
		for (int j = 0; j < 7; j++) {
			monitorFile >> d1;
		}
		monitorFile >> p1.x >> p1.y >> p1.z;
		actualFile << (p1.x - WORK_X) << " " << (p1.y - WORK_Y) << " " << (p1.z - WORK_Z) << endl;
		for (int j = 0; j < 9; j++) {
			monitorFile >> d1;
		}
	}
	monitorFile.close();
	actualFile.close();
}

double calDis(Point_Struct p1, Point_Struct p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}
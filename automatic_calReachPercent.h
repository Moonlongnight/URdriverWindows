//X


/*
Function: 计算至点度
i:
·往后看的长度(mm)
·插补点集的所用的间距(mm)
·目标点集文件(file after interploted)与实际点集文件(monitorLog.txt)

o:至点度=到达点数/目标点数
m:要正确判断是否到达目标点，判断的时候需要具有“局部性”
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
int ACTUAL_FILE_LINE_NUMBER = 0; //monitorLog.txt的行数/4
int TARGET_FILE_LINE_NUMBER = 0;

/*work pose*/
#define WORK_X 200
#define WORK_Y -100
#define WORK_Z 200

#define	ZERO_THRESH_calReachPercent 0.2//两点距离<0.2mm则认为重合
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
Function:  根据actual.txt和target.txt文件计算result[0]和result[1]，同时将未至目标点写入Missed.txt
result[0]:至点率
result[1]:( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数
i
o
m
*/
vector<double> calReachPercent(double lookAheadDistance, double gap) {
	ifstream targetFile, actualFile;
	ofstream missedFile;
	targetFile.open(TARGET_POINT_SET_FILE_ADDR);
	actualFile.open(ACTUAL_POINT_SET_FILE_ADDR);
	missedFile.open("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_Missed.txt");//记录没有到达的目标点行数和该行数对应的目标点
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

	double unReachCount = 0;//声明为double：防止return的时候进行的是纯int型触发
	int n = lookAheadDistance / gap;//此程序中n的意义是前瞻区域的点数，而不是往后看多少个点。
	int i = 0, j = 0, curj = 0;//j表示的是相对位置，curj才表示当前已经比较到了哪个目标点
	double sumOfDis = 0;//∑calDis(至点对应的实际点，至点对应的目标点)
	while (i <= ACTUAL_FILE_LINE_NUMBER - 1) {
		/*由于轨迹最后一段剩余目标点数小于前瞻数，此时将n赋值为剩余点数——即在最后轨迹段 前瞻至目标点集最后一个点
		·n-1 往后看n-1个点
		·curj 最新"待比较目标点"下标
		·curj+1 最新"待比较目标点"+已完成比较的目标点的数量
		·curj+1-1 已完成比较的目标点的数量*/
		if (curj + n > TARGET_FILE_LINE_NUMBER) {//（最新"待比较目标点"+已完成比较的目标点的数量）+往后看的点 = (curj+1)+(n-1) > 总目标点数
			n = TARGET_FILE_LINE_NUMBER - curj;
		}
		/*计算实际点集[i]与前瞻段各点距离并判断是否重合*/
		for (j = 0; j < n && CheckZeroThresh_calReachPercent(calDis(actual_pointSet[i], target_pointSet[curj + j])) == 0; j++);//直到在 j to j+n 区域找到重合点跳出；或者j to j+n一个都不匹配，于是退出
		if (j != n) {//在 j to j+n 区域有“与实际点集[i]重合的目标点”
			sumOfDis += calDis(actual_pointSet[i], target_pointSet[curj + j]);//calDis(至点对应的实际点，至点对应的目标点)
		}
		i++;
		if (j == n) {//在 j to j+n 区域没有“与实际点集[i]重合的目标点”
			continue;
		}
		else
		{
			unReachCount += j;//若在for循环的第一次运行：j=0的时候就发现重合，说明没有未至点。
			for (int k = 0; k < j; k++) {//curj+j为对应重合点，将未至点curj+0到curj+j-1写入文件
				missedFile << curj + k + 1 << " " << target_pointSet[curj + k].x << " " << target_pointSet[curj + k].y << " " << target_pointSet[curj + k].z << endl;//curj+k表示下标。不过为了表示行数，因此curj+k之后要加1。
			}
			curj += j + 1;
		}
		if (curj >= TARGET_FILE_LINE_NUMBER)//因目标点文件完毕而跳出
			break;
	}

	if (i > ACTUAL_FILE_LINE_NUMBER - 1) { //因为不满足i <= ACTUAL_FILE_LINE_NUMBER而退出,需处理目标点最后没比较完的部分。
		unReachCount += (TARGET_FILE_LINE_NUMBER - curj);
		for (int k = 0; k < TARGET_FILE_LINE_NUMBER - curj; k++) {
			missedFile << curj + k + 1 << " " << target_pointSet[curj + k].x << " " << target_pointSet[curj + k].y << " " << target_pointSet[curj + k].z << endl;//curj+k表示下标。不过为了表示行数，因此curj+k之后要加1。
		}

	}
	missedFile.close();
	vector<double> result(2);//result[0]为至点率，result[1]为( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数
	result[0] = (TARGET_FILE_LINE_NUMBER - unReachCount) / TARGET_FILE_LINE_NUMBER;
	result[1] = sumOfDis / double((TARGET_FILE_LINE_NUMBER - unReachCount));
	return result;
}

/*
Function:  从monitorLog.txt里面得到只有pose的(x,y,z)的实际点集。单位为mm
i
o
m:由于file after interploted中的点集是相对位置，因此为相互对应，将monitorLog.txt中的base视角转为work视角下的坐标，即也为相对位置
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
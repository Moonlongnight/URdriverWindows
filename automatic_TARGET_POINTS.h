#pragma once
#include <iostream>
#include <Eigen/Dense>
#include<math.h>
#include<fstream>
#include<vector>

#include"trans.h"
#include"getLineNumber.h"
#include"checkfp.h"
#include"vectorOperation.h"
using namespace Eigen;
using namespace std;

//读入目标点集，分析并保存各点特征
class automatic_TARGETS_POINTS {
private:
	vector<Vector3d> p;
	vector<double> turnAngle;//转折角度(°)
	vector<double> disWithBefore;//与前一点的距离
public:
	/*构造函数*/
	automatic_TARGETS_POINTS() {
		int lineNum = getLineNumber("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt", "");

		ifstream ifp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt");
		checkfp(ifp);
		//读点
		p.resize(lineNum);
		for (int i = 0; i < lineNum; i++) {
			ifp >> p[i][0] >> p[i][1] >> p[i][2];
		}
		ifp.close();
		//计算各点的转折角度与disWithBefore
		turnAngle.resize(lineNum);
		disWithBefore.resize(lineNum);
		
		turnAngle[0] = -1;//开始点和结尾点的转折角度置为-1
		turnAngle[lineNum - 1] = -1;
		disWithBefore[0] = -1;//开始点的disWithBefore置为-1
		disWithBefore[lineNum - 1] = calDisOF_2points(p[lineNum - 1], p[lineNum - 2]);
		
		for (int i = 1; i < lineNum-1; i++) {//i为点的下标
			disWithBefore[i] = calDisOF_2points(p[i], p[i - 1]);
			turnAngle[i] = r2d(calAngleOFvector(p[i + 1] - p[i], p[i] - p[i - 1]));
		}
	}
	/*用于与automatic_sheet联动:首先getMaxTurnAngle(curPoint,L)得到最大弯折角。然后matchWithTurnAngle得到该点的速度和前瞻时间参数
	返回从下标为s的点开始往后，对相邻点距进行累加，直到当到第n个点时，点距和大于等于L。比较i点到n点的弯折角度，返回s点到n点的最大弯折角度*/
	double getMaxTurnAngle(int s, double L) {
		double sum_dis = 0, max_curvature = 0;
		for (int i = s; i < p.size() - 1 && sum_dis < L; i++) {//若i最后一个点即p.size()-1，则返回的就是0;若在往后并点距累加的过程中，点距和尚未超过L，点就读完了，直接返回当前最大弯折角度
			sum_dis += disWithBefore[i + 1];
			if (turnAngle[i + 1] > max_curvature) {
				max_curvature = turnAngle[i + 1];
			}
		}
		return max_curvature;
	}
};
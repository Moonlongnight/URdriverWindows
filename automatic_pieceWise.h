#pragma once
#include<iostream>
#include<fstream>
#include <Eigen/Dense>
#include<math.h>
#include<vector>

#include"vectorOperation.h"
#include "trans.h"
#include"Point.h"
#include"checkfp.h"
#include"getLineNumber.h"

using namespace Eigen;
using namespace std;

typedef struct {
	double theta1, theta2, delta_theta;//单位为弧度
	double d1,d2;
}pathPointCharacteristic;//轨迹点特征

//X

//提取轨迹的特征并对轨迹进行分段
void pieceWise() {
	int lineNum = getLineNumber("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt","");

	ifstream ifp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt");
	checkfp(ifp);
	ofstream ofp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_pieceWisePath.txt");
	checkfp(ofp);
	
	Vector3d a0, a1, a2;
	int checkPoint=0;//检验点，也可以叫做转折点
	ifp >> a0[0] >> a0[1] >> a0[2];
	ifp >> a1[0] >> a1[1] >> a1[2];
	Vector3d v1, v2;
	Vector3d x_axis;//x轴
	x_axis[0] = 1; x_axis[1] = 0; x_axis[2] = 0;
	
	vector<pathPointCharacteristic> pathCharacteristicExtract(lineNum);//轨迹特征提取
	pathPointCharacteristic temp;
	//开始点和结尾点的特征置为-1
	temp.d1 = temp.d2 = temp.delta_theta = temp.theta1 = temp.theta2 = -1;
	pathCharacteristicExtract[0] = temp;
	pathCharacteristicExtract[lineNum-1] = temp;
	
	int startP = 0, endP;//当前段的开始点下标和结束点下标
	
	for (int i = 1; i < lineNum-1; i++) {//当前a1对应的下标为i
		ifp >> a2[0] >> a2[1] >> a2[2];
		v1 = a1 - a0;
		v2 = a2 - a1;

		//a1对应的特征参数
		temp.d1 = calDisOF_2points(a0, a1);
		temp.d2 = calDisOF_2points(a1, a2);
		temp.delta_theta = calAngleOFvector(v2, v1);
		temp.theta1 = calAngleOFvector(v1, x_axis);
		temp.theta2 = calAngleOFvector(v2, x_axis);
		pathCharacteristicExtract[i] = temp;

		//若当前a1的delta_theta==0，则将当前a1加入当前段
		if (i == 1 || pathCharacteristicExtract[i].delta_theta == 0) {
			if (i == 1) {
				if (pathCharacteristicExtract[i].delta_theta != 0) {
					checkPoint = i;
				}
			}
		}
		else {//若当前a1的delta_theta!=0，则需要另起一段。此时的i为新的转折点下标
			//在另起一段之前，先将当前段写入文件
			endP = i;//结束点为新发现的转折点
			ofp << startP << " " << endP << endl;
			ofp << checkPoint << endl;
			
			//重置startP和当前段的点
			startP = checkPoint;
			checkPoint = i;//为新的checkPoint赋值
		}
		a0 = a1;
		a1 = a2;
	}
	//最后一段
	ofp << startP << " " << lineNum - 1 << endl;
	ofp << checkPoint << endl;

	ifp.close();
	ofp.close();


	/*轨迹特征记录*/
	ofstream ofp_pathCharacteristic("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_pathCharacteristicExtract.txt");
	checkfp(ofp_pathCharacteristic);
	for (int i = 0; i < lineNum; i++) {
		ofp_pathCharacteristic << i << endl;
		ofp_pathCharacteristic << pathCharacteristicExtract[i].d1 << " " << pathCharacteristicExtract[i].d2 << endl;
		ofp_pathCharacteristic << r2d(pathCharacteristicExtract[i].theta1) << " " << r2d(pathCharacteristicExtract[i].theta2) << " " << r2d(pathCharacteristicExtract[i].delta_theta) << endl;
	}
}
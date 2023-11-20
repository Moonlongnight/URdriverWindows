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
	double theta1, theta2, delta_theta;//��λΪ����
	double d1,d2;
}pathPointCharacteristic;//�켣������

//X

//��ȡ�켣���������Թ켣���зֶ�
void pieceWise() {
	int lineNum = getLineNumber("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt","");

	ifstream ifp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt");
	checkfp(ifp);
	ofstream ofp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_pieceWisePath.txt");
	checkfp(ofp);
	
	Vector3d a0, a1, a2;
	int checkPoint=0;//����㣬Ҳ���Խ���ת�۵�
	ifp >> a0[0] >> a0[1] >> a0[2];
	ifp >> a1[0] >> a1[1] >> a1[2];
	Vector3d v1, v2;
	Vector3d x_axis;//x��
	x_axis[0] = 1; x_axis[1] = 0; x_axis[2] = 0;
	
	vector<pathPointCharacteristic> pathCharacteristicExtract(lineNum);//�켣������ȡ
	pathPointCharacteristic temp;
	//��ʼ��ͽ�β���������Ϊ-1
	temp.d1 = temp.d2 = temp.delta_theta = temp.theta1 = temp.theta2 = -1;
	pathCharacteristicExtract[0] = temp;
	pathCharacteristicExtract[lineNum-1] = temp;
	
	int startP = 0, endP;//��ǰ�εĿ�ʼ���±�ͽ������±�
	
	for (int i = 1; i < lineNum-1; i++) {//��ǰa1��Ӧ���±�Ϊi
		ifp >> a2[0] >> a2[1] >> a2[2];
		v1 = a1 - a0;
		v2 = a2 - a1;

		//a1��Ӧ����������
		temp.d1 = calDisOF_2points(a0, a1);
		temp.d2 = calDisOF_2points(a1, a2);
		temp.delta_theta = calAngleOFvector(v2, v1);
		temp.theta1 = calAngleOFvector(v1, x_axis);
		temp.theta2 = calAngleOFvector(v2, x_axis);
		pathCharacteristicExtract[i] = temp;

		//����ǰa1��delta_theta==0���򽫵�ǰa1���뵱ǰ��
		if (i == 1 || pathCharacteristicExtract[i].delta_theta == 0) {
			if (i == 1) {
				if (pathCharacteristicExtract[i].delta_theta != 0) {
					checkPoint = i;
				}
			}
		}
		else {//����ǰa1��delta_theta!=0������Ҫ����һ�Ρ���ʱ��iΪ�µ�ת�۵��±�
			//������һ��֮ǰ���Ƚ���ǰ��д���ļ�
			endP = i;//������Ϊ�·��ֵ�ת�۵�
			ofp << startP << " " << endP << endl;
			ofp << checkPoint << endl;
			
			//����startP�͵�ǰ�εĵ�
			startP = checkPoint;
			checkPoint = i;//Ϊ�µ�checkPoint��ֵ
		}
		a0 = a1;
		a1 = a2;
	}
	//���һ��
	ofp << startP << " " << lineNum - 1 << endl;
	ofp << checkPoint << endl;

	ifp.close();
	ofp.close();


	/*�켣������¼*/
	ofstream ofp_pathCharacteristic("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_pathCharacteristicExtract.txt");
	checkfp(ofp_pathCharacteristic);
	for (int i = 0; i < lineNum; i++) {
		ofp_pathCharacteristic << i << endl;
		ofp_pathCharacteristic << pathCharacteristicExtract[i].d1 << " " << pathCharacteristicExtract[i].d2 << endl;
		ofp_pathCharacteristic << r2d(pathCharacteristicExtract[i].theta1) << " " << r2d(pathCharacteristicExtract[i].theta2) << " " << r2d(pathCharacteristicExtract[i].delta_theta) << endl;
	}
}
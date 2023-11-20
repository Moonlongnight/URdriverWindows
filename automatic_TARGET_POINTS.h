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

//����Ŀ��㼯�������������������
class automatic_TARGETS_POINTS {
private:
	vector<Vector3d> p;
	vector<double> turnAngle;//ת�۽Ƕ�(��)
	vector<double> disWithBefore;//��ǰһ��ľ���
public:
	/*���캯��*/
	automatic_TARGETS_POINTS() {
		int lineNum = getLineNumber("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt", "");

		ifstream ifp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt");
		checkfp(ifp);
		//����
		p.resize(lineNum);
		for (int i = 0; i < lineNum; i++) {
			ifp >> p[i][0] >> p[i][1] >> p[i][2];
		}
		ifp.close();
		//��������ת�۽Ƕ���disWithBefore
		turnAngle.resize(lineNum);
		disWithBefore.resize(lineNum);
		
		turnAngle[0] = -1;//��ʼ��ͽ�β���ת�۽Ƕ���Ϊ-1
		turnAngle[lineNum - 1] = -1;
		disWithBefore[0] = -1;//��ʼ���disWithBefore��Ϊ-1
		disWithBefore[lineNum - 1] = calDisOF_2points(p[lineNum - 1], p[lineNum - 2]);
		
		for (int i = 1; i < lineNum-1; i++) {//iΪ����±�
			disWithBefore[i] = calDisOF_2points(p[i], p[i - 1]);
			turnAngle[i] = r2d(calAngleOFvector(p[i + 1] - p[i], p[i] - p[i - 1]));
		}
	}
	/*������automatic_sheet����:����getMaxTurnAngle(curPoint,L)�õ�������۽ǡ�Ȼ��matchWithTurnAngle�õ��õ���ٶȺ�ǰհʱ�����
	���ش��±�Ϊs�ĵ㿪ʼ���󣬶����ڵ������ۼӣ�ֱ��������n����ʱ�����ʹ��ڵ���L���Ƚ�i�㵽n������۽Ƕȣ�����s�㵽n���������۽Ƕ�*/
	double getMaxTurnAngle(int s, double L) {
		double sum_dis = 0, max_curvature = 0;
		for (int i = s; i < p.size() - 1 && sum_dis < L; i++) {//��i���һ���㼴p.size()-1���򷵻صľ���0;�������󲢵���ۼӵĹ����У�������δ����L����Ͷ����ˣ�ֱ�ӷ��ص�ǰ������۽Ƕ�
			sum_dis += disWithBefore[i + 1];
			if (turnAngle[i + 1] > max_curvature) {
				max_curvature = turnAngle[i + 1];
			}
		}
		return max_curvature;
	}
};
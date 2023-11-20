#pragma once
#include<vector>
#include<iostream>
#include<fstream>

#include"getLineNumber.h"
#include"checkfp.h"
using namespace std;
//ÿһ�����ǰհʱ����ٶȣ���Ҫ������������ɵ�����۽Ƕ����ֵȷ��.
//�������ɵ�����Ǽ����㣬����ʹ�þ������L��ȷ��������n�����뵱ǰ�������ڵ���L����ӵ�ǰ�㵽���n����ģ������е�������۽Ƕ�����ȷ�ϵ�ǰ���ǰհʱ����ٶ�
class automatic_sheet {
private:
	double L;
	vector<vector<double>> table;
public:
	/*���캯��*/
	automatic_sheet() {
		readFILE_automatic_sheet();
	}

	/*��ȡautomatic_sheet�ļ�*/
	void readFILE_automatic_sheet() {
		int lineNum = getLineNumber("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_sheet.txt");
		
		table.resize(lineNum - 1);//txt�ļ���һ��ΪL����˹���lineNum-1�����ڱ��
		for (int i = 0; i < table.size(); i++) {
			table[i].resize(3);//table��ÿ�����������ֱ�Ϊ��������۽Ƿ�Χ���ҽ��� | v | lookahead_time
		}

		ifstream ifp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_sheet.txt");
		ifp >> L;
		//table���и�ʽΪ��������۽Ƿ�Χ���ҽ��� | v | lookahead_time
		for (int i = 0; i < table.size(); i++) {
			ifp >> table[i][0] >> table[i][1] >> table[i][2];
		}

		ifp.close();
	}

	/*����������۽ǣ�ƥ���Ӧ��table�в����ظ���*/
	vector<double> matchWithTurnAngle(double turnAngle) {
		for (int i = 0; i < table.size(); i++) {
			if (turnAngle <= table[i][0]) {
				return table[i];
			}
		}
		cout << "ERROR IN automatic_sheet _" << __LINE__ << ": no match angle scope" << endl;
		system("pause");
		exit;
	}

	double getL() {
		return L;
	}
};
#pragma once
#include<vector>
#include<iostream>
#include<fstream>

#include"getLineNumber.h"
#include"checkfp.h"
using namespace std;
//每一个点的前瞻时间和速度，需要根据其后面若干点的弯折角度最大值确定.
//至于若干点具体是几个点，我们使用距离参数L来确定，当第n个点与当前点距离大于等于L，则从当前点到这第n个点的，各点中的最大弯折角度用来确认当前点的前瞻时间和速度
class automatic_sheet {
private:
	double L;
	vector<vector<double>> table;
public:
	/*构造函数*/
	automatic_sheet() {
		readFILE_automatic_sheet();
	}

	/*读取automatic_sheet文件*/
	void readFILE_automatic_sheet() {
		int lineNum = getLineNumber("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_sheet.txt");
		
		table.resize(lineNum - 1);//txt文件第一行为L，因此共有lineNum-1行属于表格
		for (int i = 0; i < table.size(); i++) {
			table[i].resize(3);//table的每行有三项，三项分别为：最大弯折角范围的右界限 | v | lookahead_time
		}

		ifstream ifp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_sheet.txt");
		ifp >> L;
		//table各行格式为：最大弯折角范围的右界限 | v | lookahead_time
		for (int i = 0; i < table.size(); i++) {
			ifp >> table[i][0] >> table[i][1] >> table[i][2];
		}

		ifp.close();
	}

	/*根据最大弯折角，匹配对应的table行并返回该行*/
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
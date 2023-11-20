#pragma once
#include<iostream>
#include"fileOpration.h"
#include"trans.h"
#include"absd.h"
#include"getLineNumber.h"
#include<vector>
double auto_dis2p(const URPose& p1, const URPose& p2);
/*
//X

Function:	use skip and interplote operation to make a pose set cut as standard distance
Input:      lineNum - number of poses
Output:     poses after handling
Main Idea:  record start pose , start skip until two poses' dis >= sdis , then interplote between two pose
			and push last pose . record last pose as start pose and start skip untill ......
*/
void automatic_interplote_p(const double sdis) {
	string iaddr, oaddr;
	int lineNum;
	iaddr = "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose.txt";
	oaddr= "C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated.txt";
	lineNum = getLineNumber(iaddr, "");

	fileOpration fp(iaddr);
	//const double sdis = 0.05;//standard dis (mm)
	/*put q into ram*/
	vector<URPose> p(lineNum);
	for (int i = 0; i < lineNum; i++) {
		p[i] = fp.getNexp();
	}
	/*若sdis为0，则表示不进行插补*/
	if (sdis == 0) {
		ofstream ofp_interpolate("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated.txt");
		checkfp(ofp_interpolate);
		for (int i = 0; i < lineNum; i++) {
			ofp_interpolate << p[i].x << " " << p[i].y << " " << p[i].z << endl;
		}
		ofp_interpolate.close();
		return;
	}


	/*interplote and skip opration*/
	vector<URPose> pResult;
	URPose pTemp, p1, p2;
	p1 = p[0];
	pResult.push_back(p1);
	double distemp;
	for (int i = 1; i < lineNum; i++)
	{
		if ((distemp = auto_dis2p(p1, p[i])) > sdis) {//interplote
			p2 = p[i];
			for (int j = 1; j < (((int)(distemp * m2um)) / ((int)(sdis * m2um))); j++) {
				pTemp.x = p1.x - (p1.x - p2.x) * absd(sdis / distemp) * j;
				pTemp.y = p1.y - (p1.y - p2.y) * absd(sdis / distemp) * j;
				pTemp.z = p1.z - (p1.z - p2.z) * absd(sdis / distemp) * j;
				//pTemp.rx = p1.rx - (p1.rx - p2.rx) * absd(sdis / distemp) * j;
				//pTemp.ry = p1.ry - (p1.ry - p2.ry) * absd(sdis / distemp) * j;
				//pTemp.rz = p1.rz - (p1.rz - p2.rz) * absd(sdis / distemp) * j;
				pResult.push_back(pTemp);
			}
			pResult.push_back(p2);
			p1 = p2;
		}
		else
		{
			continue;
		}
	}
	pResult.push_back(p[lineNum - 1]);//in case not push last q because not reach last gap

	//put result into file
	ofstream fpo(oaddr);
	if (!fpo) {
		cout << "WARN at interplote:fail to open IKresultSgap" << endl;
	}
	for (int i = 0; i < pResult.size(); i++)
	{
		fpo << pResult[i].x << " " << pResult[i].y << " " << pResult[i].z << endl;
		//pResult[i].rx << " " << pResult[i].ry << " " << pResult[i].rz << endl;
	}
	fpo.close();

	//check pResult -- [1,2 )
	for (int i = 0; i < pResult.size() - 1; i++) {
		cout << "i:" << i << "  " << auto_dis2p(pResult[i], pResult[i + 1]) << endl;;
	}
}


double auto_dis2p(const URPose& p1, const URPose& p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}
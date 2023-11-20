#pragma once
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "trans.h"
using namespace std;
typedef struct {
	double x;
	double y;
	double z;
} Point_Position;

void print_Point_Position(Point_Position p) {
	cout << "[" << p.x << "," << p.y << "," << p.z << "]" << endl;
}

class fileOpration {
private:
	ifstream infile;
	int lineNum = 0;
	Point_Position p;
	vector<double> q;     // record 6 joint pos
	vector<string> qstr;  // read from file directly

public:
	fileOpration(string fileAddr) {
		infile.open(fileAddr);
		if (!infile) {
			cout << "WARN at fileOpration:" << "fail to open file" << endl;
			exit(0);
		}
		q.resize(6);
		qstr.resize(6);
	}
	~fileOpration() { infile.close(); }

	URPose getNexp() {
		URPose p;
		double data;
		infile >> data;
		p.x = data;
		infile >> data;
		p.y = data;
		infile >> data;
		p.z = data;

		//infile >> data;
		//p.rx = data;
		//infile >> data;
		//p.ry = data;
		//infile >> data;
		//p.rz = data;

		lineNum++;
		return p;
	}

	Point_Position getNexPos() {
		double data;
		infile >> data;
		p.x =data;
		infile >> data;
		p.y = data;
		infile >> data;
		p.z = data;
		lineNum++;
		infile.get();  // eat /n
		return p;
	}
	string getNexPos_str() {
		getNexPos();
		string s;
		s.clear();
		s += "p[" + to_string(p.x) + "," + to_string(p.y) + "," + to_string(p.z) +
			",0,0,0]";

		return s;
	}
	vector<double> getNexq() {
		infile >> q[0];
		infile >> q[1];
		infile >> q[2];
		infile >> q[3];
		infile >> q[4];
		infile >> q[5];
		lineNum++;
		//infile.get();  // eat /n
		return q;
	}
	string getNexq_str() {
		infile >> qstr[0];
		infile >> qstr[1];
		infile >> qstr[2];
		infile >> qstr[3];
		infile >> qstr[4];
		infile >> qstr[5];
		lineNum++;
		infile.get();//eat \n
		string s = "[" + qstr[0] + "," + qstr[1] + "," + qstr[2] + "," + qstr[3] +
			"," + qstr[4] + "," + qstr[5] + "]";
		return s;
	}

	bool isEnd() {
		if (infile.peek() == '\n') return 1;
		return 0;
	}
	int getlineNum() { return lineNum; }
//¸¨Öú´òÓ¡
public:
	/*
	Function:  used to transfer qstr like : [1.57,1,2,1,2,1] to vector<double> q_radius
	i
	o
	m
	*/
	void qstrToDegree(const string& qstr) {
		int offset = 0;
		offset++;//[
		string qs;
		vector<double> q_;
		for (int i = 0; i < 6; i++)
		{
			qs.clear();
			while (qstr[offset] != ',' && qstr[offset] != ']')
			{
				qs += qstr[offset];
				offset++;
			}
			q_.push_back(stod(qs));
			offset++;
		}
		cout << r2d(q_[0])<<"  " << r2d(q_[1]) << "  " << r2d(q_[2]) << "  " << r2d(q_[3]) << "  " << r2d(q_[4]) << "  " << r2d(q_[5]) << endl;
	}
	/*
	Function:  used to print q_radius in format as q_degree
	i
	o
	m
	*/
	void qToDegree(const vector<double>& q_) {
		cout << r2d(q_[0]) << "  " << r2d(q_[1]) << "  " << r2d(q_[2]) << "  " << r2d(q_[3]) << "  " << r2d(q_[4]) << "  " << r2d(q_[5]) << endl;
	}
};

#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include"checkfp.h"
using namespace std;

int getLineNumber(string filename)
{
    ifstream ReadFile;
    int n = 0;
    string tmp;
    ReadFile.open(filename, ios::in);//ios::in ��ʾ��ֻ���ķ�ʽ��ȡ�ļ�
    checkfp(ReadFile);

    while (getline(ReadFile, tmp, '\n'))
    {
        n++;
    }
    ReadFile.close();
    return n;
}

int getLineNumber(string filename,string endLine)
{
    ifstream ReadFile;
    int n = 0;
    string tmp;
    ReadFile.open(filename, ios::in);//ios::in ��ʾ��ֻ���ķ�ʽ��ȡ�ļ�
    checkfp(ReadFile);

    while (getline(ReadFile, tmp, '\n') && tmp != endLine)//endLine��Ϊ�գ���ÿ��getline���������и���tmp�����������У���tmp��endLineһ�ȣ�������ͬ���˳���
    {
        n++;
    }
    ReadFile.close();
    return n;
}
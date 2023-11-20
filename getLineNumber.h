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
    ReadFile.open(filename, ios::in);//ios::in 表示以只读的方式读取文件
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
    ReadFile.open(filename, ios::in);//ios::in 表示以只读的方式读取文件
    checkfp(ReadFile);

    while (getline(ReadFile, tmp, '\n') && tmp != endLine)//endLine若为空，则：每次getline将读到的行赋给tmp，当读到空行，将tmp与endLine一比，发现相同就退出。
    {
        n++;
    }
    ReadFile.close();
    return n;
}
#pragma once
typedef struct{
	double look_ahead_time, speed, reachPercent;
	int missedNumTotal;
	double calReachPercent_result1;//����Ӧ��calReachPercent()[1]=( ��calDis(�����Ӧ��ʵ�ʵ㣬�����Ӧ��Ŀ���) )/������
	string cmd_str;
}automatic_pieceResult_Struct;


#pragma once
typedef struct{
	double look_ahead_time, speed, reachPercent;
	int missedNumTotal;
	double calReachPercent_result1;//即对应的calReachPercent()[1]=( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数
	string cmd_str;
}automatic_pieceResult_Struct;


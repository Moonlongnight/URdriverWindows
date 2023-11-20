#pragma once	
#include "URPose.h"
#include "ur_driver.h"
#include <iostream>
#include <string>
#include <vector>
#include <conio.h>
#include "fileOpration.h"
#include "trans.h"
#include <math.h>
#include <functional> 
#include "ur_kinetic.h"
#include "analyse_test1.h"
#include "getLineNumber.h"
#include"vectorOperation.h"
#include"automatic_calReachPercent.h"
#include"automatic_interpolate_p.h"
#include"automatic_getIKsolution.h"
#include"automatic_pieceResult_Struct.h"
#include"automatic_pieceWise.h"
#include"automatic_SHEET.h"
#include "automatic_TARGET_POINTS.h"
#include"linearWithParabola.h"
#include"LSPB.h"
#include"movel_Test.h"
using namespace std;
class controlReal {
private:
	UrDriver urd;               // start rt in urd's constructor
	URPose urpose;				// used for cin
	vector<double> urpose_cur;
	vector<double> urposeq;      // used for cin and take getq in workWindows
	vector<double> urposeq_cur;  // used for record lateset urq
	vector<double> urposepv_cur; //used for record latest end-tool's pose velocity
	string cmd_str;              // used for cin as well as send to ur3

	URPose target;				//target of IK：x y z rx ry rz
	vector<vector<double>> sol; //solution of IK

	bool executeTraj;
public:
	controlReal() {
		// init urpose's rx ry rz as work pose's rx ry rz
		urpose.rx = 2.2214;
		urpose.ry = -2.2214;
		urpose.rz = 0;
		// init urposeq
		urposeq.resize(6);
		urposeq[0] = 0;
		urposeq[1] = 0;
		urposeq[2] = 0;
		urposeq[3] = 0;
		urposeq[4] = 0;
		urposeq[5] = 0;

		executeTraj = 0;

		//init sol
		sol.resize(8);
		for (int i = 0; i < 8; i++) {
			sol[i].resize(6);
			for (int j = 0; j < 6; j++) {
				sol[i][j] = 0;
			}
		}

		cout << "Welcome use controlReal" << endl;
	}
	~controlReal() { cout << "Thanks for using controlReal" << endl; };

	void openworkWindows() {
		thread* update_loop_thread = new thread(bind(&controlReal::update_loop, this));
		thread* workWindows_thread = new thread(bind(&controlReal::workWindows, this));
		workWindows_thread->join();
		update_loop_thread->join();

	}
	void workWindows() {
		char opt_ch;
		int opt_int;
		double opt_double;
		while (1) {
			cout << "Input cmd" << endl;
			cin >> cmd_str;
			if (cmd_str == "home") {
				home();
			}
			else if (cmd_str == "up") {
				up();
			}
			else if (cmd_str == "work") {
				work();
			}
			else if (cmd_str == "mjq") {
				cout << "Input q1,q2,q3,q4,q5,q6 [Degree]" << endl;
				/*set arm_jointValueTarget*/
				cin >> urposeq[0] >> urposeq[1] >> urposeq[2] >> urposeq[3] >>
					urposeq[4] >> urposeq[5];
				urposeq = { d2r(urposeq[0]), d2r(urposeq[1]), d2r(urposeq[2]),
						   d2r(urposeq[3]), d2r(urposeq[4]), d2r(urposeq[5]) };
				mjq("empty", urposeq);
				Sleep(2);
			}
			else if (cmd_str == "mjqr") {
				cout << "Input q1,q2,q3,q4,q5,q6 [rad]" << endl;
				cin >> urposeq[0] >> urposeq[1] >> urposeq[2] >> urposeq[3] >>
					urposeq[4] >> urposeq[5];
				urd.setMonitorFlag();
				mjq("empty", urposeq);
				Sleep(2);
			}
			else if (cmd_str == "mlp") {
				cout << "Input x, y, z[mm] and Rx Ry Rz " << endl;
				cin >> urposeq[0] >> urposeq[1] >> urposeq[2] >> urposeq[3] >>
					urposeq[4] >> urposeq[5];
				urposeq[0] *= mm2m;
				urposeq[1] *= mm2m;
				urposeq[2] *= mm2m;
				urd.setMonitorFlag();
				mlp(urposeq);
				Sleep(2);
			}
			else if (cmd_str == "movep") {
				cout << "Input x, y, z[mm] and Rx Ry Rz " << endl;
				cin >> urposeq[0] >> urposeq[1] >> urposeq[2] >> urposeq[3] >>
					urposeq[4] >> urposeq[5];
				urposeq[0] *= mm2m;
				urposeq[1] *= mm2m;
				urposeq[2] *= mm2m;
				urd.setMonitorFlag();
				movep(urposeq);
				Sleep(2);
			}
			else if (cmd_str == "uploadProg_movep" || cmd_str == "ug") {
				uploadProg_movep();
			}
			else if (cmd_str == "mjp") {
				cout << "Input x,y,z [mm] Default Rx Ry Rz=" << urpose.rx << " " << urpose.ry << " " << urpose.rz << endl;
				cin >> urpose.x;
				cin >> urpose.y;
				cin >> urpose.z;
				urpose.x = urpose.x * mm2m;
				urpose.y = urpose.y * mm2m;
				urpose.z = urpose.z * mm2m;
				mjp(urpose);
				Sleep(2);
			}
			else if (cmd_str == "mjpFull" || cmd_str == "mjpf") {
				cout << "Input x,y,z [mm] and Rx Ry Rz" << endl;
				cin >> urpose.x;
				cin >> urpose.y;
				cin >> urpose.z;
				cin >> urpose.rx >> urpose.ry >> urpose.rz;
				urpose.x = urpose.x * mm2m;
				urpose.y = urpose.y * mm2m;
				urpose.z = urpose.z * mm2m;
				mjp(urpose);
				Sleep(2);
			}
			else if (cmd_str == "moveRelative" || cmd_str == "mr") {
				cout << "Input Relative Distance Delta_X,Delta_Y,Delta_Z [mm] "<< endl;
				double Delta_X, Delta_Y, Delta_Z;
				cin >> Delta_X >> Delta_Y >> Delta_Z;
				urposeq = urd.getp();
				urposeq[0] += Delta_X;
				urposeq[1] += Delta_Y;
				urposeq[2] += Delta_Z;
				urposeq[0] *= mm2m;
				urposeq[1] *= mm2m;
				urposeq[2] *= mm2m;
				mjp2(urposeq);
			}

			else if (cmd_str == "dotraj") {
				dotraj(0);//use 0.008
				cout << "Finish sendCmd" << endl;
			}
			else if (cmd_str == "dotrajmj") {
				cout << "Start do traj" << endl;
				dotraj(1);
				cout << "Finish sendCmd" << endl;
			}
			else if (cmd_str == "uploadProg_servoj" || cmd_str == "uservoj") {
				uploadProg_servoj2();
			}
			else if (cmd_str == "uploadProg_servoj3" || cmd_str == "uservoj3") {
				uploadProg_servoj3();
			}
			else if (cmd_str == "uploadProg_servoj4" || cmd_str == "uservoj4") {
				uploadPorg_servoj4();
			}
			else if (cmd_str == "automatic_adjust_parameter" || cmd_str == "autoap") {
				//automatic_adjust_parameter();
				automatic_adjust_parameter_OFpieceWise(0);
			}
			else if (cmd_str == "checkIK") {
				checkIK();
			}
			else if (cmd_str == "exit") {
				break;
			}
			else if (cmd_str == "getq") {
				urposeq_cur = urd.getq();
				cout << "[" << urposeq_cur[0] << " , " << urposeq_cur[1] << " , "
					<< urposeq_cur[2] << " , " << urposeq_cur[3] << " , "
					<< urposeq_cur[4] << " , " << urposeq_cur[5] << "]" << endl;
			}
			else if (cmd_str == "getp") {
				urpose_cur = urd.getp();
				cout << "[" << urpose_cur[0] << " , " << urpose_cur[1] << " , "
					<< urpose_cur[2] << " , " << urpose_cur[3] << " , "
					<< urpose_cur[4] << " , " << urpose_cur[5] << "]" << endl;
			}
			else if (cmd_str == "settcpProg") {
				cout << "Input Rx Ry Rz" << endl;
				cin >> urpose.rx;
				cin >> urpose.ry;
				cin >> urpose.rz;
			}
			else if (cmd_str == "settcp") {
				cout << "Input x y z and Rx Ry Rz" << endl;
				string settcp_x, settcp_y, settcp_z, settcp_Rx, settcp_Ry, settcp_Rz;
				cin >> settcp_x >> settcp_y >> settcp_z >> settcp_Rx >> settcp_Ry >> settcp_Rz;
				//set_tcp(p[0.,.2,.3,0.,3.14,0.])
				cmd_str = "set_tcp(p[" + settcp_x + "," + settcp_y + "," + settcp_z + "," + settcp_Rx + "," + settcp_Ry + "," + settcp_Rz + "])\n";
				urd.sendCmd(cmd_str);
			}
			else if (cmd_str == "sendcmd") {
				cout << "Input cmd you want to send" << endl;
				cin >> cmd_str;
				cmd_str += "\n";
				urd.sendCmd(cmd_str);
			}
			else if (cmd_str == "openMonitor" || cmd_str=="om") {
				openMonitor();
			}
			else if (cmd_str == "interplote_p" || cmd_str == "ip") {
				cout << "input lineNum of  file being interploted, '0' as default:1000" << endl;
				cin >> opt_int;
				if (opt_int == 0) {
					opt_int = 1000;
				}
				interplote_p(opt_int);
			}
			else if (cmd_str == "analyse_test1" || cmd_str == "atest1") {
				//analyse_test1();
				cmd_str=generateCodeForTest1();
				urd.sendCmd(cmd_str);
				urd.setMonitorFlag();
			}
			else if (cmd_str == "ls") {
				cout << "home          up          work" << endl;
				cout << "mjq           mjqr        mjp" << endl;
				cout << "dotraj size   dotrajmj size" << endl;
				cout << "getq          getp	       settcp" << endl;
				cout << "exit" << endl;
			}
			else if (cmd_str == "testIK" ) {
				testIK();
			}
			else if (cmd_str == "trajectory_planning" || cmd_str == "lwp") {
				cout << "input a_normal" << endl;
				double anormal_lwp = 0;
				cin >> anormal_lwp;
				cout << "input v in cartesian" << endl;
				double v_lwp = 0;
				cin >> v_lwp;

				vector<double> td = get_timeSeqWith_constantVcartesian(v_lwp);
				string cmd_str_lwp = linearWithParabola(td, anormal_lwp, 5);
				
				cout << "move to satrt q" << endl;
				ifstream qfile_lwp("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
				checkfp(qfile_lwp);
				vector<double> startq_lwp(6);
				qfile_lwp >> startq_lwp[0] >> startq_lwp[1] >> startq_lwp[2] >> startq_lwp[3] >> startq_lwp[4] >> startq_lwp[5];
				qfile_lwp.close();
				mjq("empty", startq_lwp);
				Sleep(2000);

				cout << "start move with speedj" << endl;
				urd.setMonitorFlag();
				urd.sendCmd(cmd_str_lwp);
				while (urd.getMonitorFlag() == 1) {};//wait till ur3 stop
			}
			else if (cmd_str == "trajectory_planning_multi" || cmd_str == "lwp_multi") {
				cout << "input max a" << endl;
				double amax_lwp = 0;
				cin >> amax_lwp;
				cout << "input v in cartesian" << endl;
				double v_lwp = 0;
				cin >> v_lwp;

				string cmd_str_lwp = linearWithParabola_multiAxis(v_lwp, amax_lwp);

				cout << "move to satrt q" << endl;
				ifstream qfile_lwp("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
				checkfp(qfile_lwp);
				vector<double> startq_lwp(6);
				qfile_lwp >> startq_lwp[0] >> startq_lwp[1] >> startq_lwp[2] >> startq_lwp[3] >> startq_lwp[4] >> startq_lwp[5];
				qfile_lwp.close();
				mjq("empty", startq_lwp);
				Sleep(2000);

				cout << "start move with speedj" << endl;
				urd.setMonitorFlag();
				urd.sendCmd(cmd_str_lwp);
				while (urd.getMonitorFlag() == 1) {};//wait till ur3 stop

				//calReachPercent
				int lineNum;
				Vector3d pc;

				vector<double> ResultOF_calReachPercent(2);
				ResultOF_calReachPercent = automatic_calReachPercent();
				cout << "Reach Percent at v=" << v_lwp << ",amax=" << amax_lwp << " is:" << ResultOF_calReachPercent[0] << endl;
				cout << "( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数=" << ResultOF_calReachPercent[1] << endl;
				//显示Miss点
				lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt", "");
				cout << "There are " << lineNum << " Miss Points" << endl;
				if (lineNum > 0) {
					ifstream ifp_missed("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt");
					checkfp(ifp_missed);
					for (int i = 0; i < lineNum; i++) {
						ifp_missed >> pc[0] >> pc[1] >> pc[2];
						cout << pc[0] << " " << pc[1] << " " << pc[2] << endl;
					}
				}
			}
			else if (cmd_str == "normal_lspb" || cmd_str == "nlspb") {

				string cmd_str_lwp = Normal_LSPB_6D();

				cout << "move to satrt q" << endl;
				ifstream qfile_lwp("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
				checkfp(qfile_lwp);
				vector<double> startq_lwp(6);
				qfile_lwp >> startq_lwp[0] >> startq_lwp[1] >> startq_lwp[2] >> startq_lwp[3] >> startq_lwp[4] >> startq_lwp[5];
				qfile_lwp.close();
				mjq("empty", startq_lwp);
				Sleep(2000);

				cout << "start move with speedj" << endl;
				urd.setMonitorFlag();
				urd.sendCmd(cmd_str_lwp);
				while (urd.getMonitorFlag() == 1) {};//wait till ur3 stop
			}
			else if (cmd_str == "normal_lspb_use_td" || cmd_str == "nlspb2") {
				vector<double> normal_lspb_td_seq{0.032,0.064 };
				vector<double> normal_lspb_td = Normal_LSPB_get_td(normal_lspb_td_seq, 5);
				string cmd_str_lwp = Normal_LSPB_6D_use_td(normal_lspb_td);

				cout << "move to satrt q" << endl;
				ifstream qfile_lwp("C:/Users/XY/Desktop/path/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
				checkfp(qfile_lwp);
				vector<double> startq_lwp(6);
				qfile_lwp >> startq_lwp[0] >> startq_lwp[1] >> startq_lwp[2] >> startq_lwp[3] >> startq_lwp[4] >> startq_lwp[5];
				qfile_lwp.close();
				mjq("empty", startq_lwp);
				Sleep(2000);

				cout << "start move with speedj" << endl;
				urd.setMonitorFlag();
				urd.sendCmd(cmd_str_lwp);
				while (urd.getMonitorFlag() == 1) {};//wait till ur3 stop
			}
			else if (cmd_str == "getIKparam") {
				getIKparam();
			}
			else if (cmd_str == "movel_Test" || cmd_str == "mltest") {
				string cmd_str_mltest = generate_movel_code();
				urd.setMonitorFlag();
				urd.sendCmd(cmd_str_mltest);
				while (urd.getMonitorFlag() == 1) {};//wait till ur3 stop
			}
			else {
				cout << "No such cmd,input again plz" << endl;
			}
		}
	}

	void openMonitor() {
		urd.setMonitorFlag2();
		char ch = 'r';
		while(1){
			if (_kbhit()) {//如果有按键按下，则_kbhit()函数返回真
				ch = _getch();//使用_getch()函数获取按下的键值
				if (ch == 'c') { break; }//当按下ESC时循环，ESC键的键值时27.
			}
		}
		urd.resetMonitorFlag2();
	}

	//static void* workWindowsHelper(void* contex) {
	//	return ((controlReal*)contex)->workWindows();
	//}
	void home() {
		urd.sendCmd("movej([0, 0, 0, 0, 0, 0], a = 2, v = 2)\n");
		Sleep(2);
	}
	void up() {
		urd.sendCmd("movej([0, d2r(-90), 0, d2r(-90), 0, 0], a = 2, v = 2)\n");
	}
	void work() {
		urd.sendCmd(
			"movej([d2r(3.6), d2r(-70.74), d2r(-137.23), d2r(-62.03),d2r(90),d2r(3.6)], a = 2, v =2)\n");
		Sleep(2);
	}
	// send servoj
	void servoj(const string& q_str = "empty", const string& t = "0.008",
		const vector<double>& q = { 0, 0, 0, 0, 0, 0 }) {
		if (q_str == "empty") {
			cout << "WARN at servoj:" << "servoj:q_str 为 empty，vector<double>类型的q的命令生成还没写" << endl;
			return;
		}
		else {
			cmd_str = "servoj(" + q_str + ",t=" + t + ",lookahead_time=0.1)\n";
			urd.sendCmd(cmd_str);
		}
	}
	// send movej
	void mjq(const string& q_str = "empty",
		const vector<double>& urposeq = { 0, 0, 0, 0, 0, 0 }) {
		if (q_str == "empty") {
			cmd_str = "movej([" + to_string(urposeq[0]) + "," + to_string(urposeq[1]) +
				"," + to_string(urposeq[2]) + "," + to_string(urposeq[3]) + "," +
				to_string(urposeq[4]) + "," + to_string(urposeq[5]) +
				"],a=2,v=2)\n";
			urd.sendCmd(cmd_str);
		}
		else {
			cmd_str = "movej(" + q_str + ",a=2,v=2)\n";
			urd.sendCmd(cmd_str);
		}
	}

	void mlp(const vector<double>& urpose = { 0, 0, 0, 0, 0, 0 }) {
			cmd_str = "movel(p[" + to_string(urpose[0]) + "," + to_string(urpose[1]) +
				"," + to_string(urpose[2]) + "," + to_string(urpose[3]) + "," +
				to_string(urpose[4]) + "," + to_string(urpose[5]) +
				"],a=0.5,v=0.02)\n";
			urd.sendCmd(cmd_str);
	}
	void movep(const vector<double>& urpose = { 0, 0, 0, 0, 0, 0 }) {
		cmd_str = "movep(p[" + to_string(urpose[0]) + "," + to_string(urpose[1]) +
			"," + to_string(urpose[2]) + "," + to_string(urpose[3]) + "," +
			to_string(urpose[4]) + "," + to_string(urpose[5]) +
			"],a=0.5,v=0.02)\n";
		urd.sendCmd(cmd_str);
	}

	/*
	Function: 发送关于movep的程序给UR3
	i
	o
	m
	*/
	void uploadProg_movep() {

		//URPose workPose(0.2, -0.1, 0.2, 2.225, -2.2194, 0);
		URPose workPose(0.2, -0.1, 0.2, 2.2214, -2.2214, 0);
		URPose urpose;
		urpose = workPose;
		urpose.x += 0.5 * mm2m;
		urpose.y += 0.5 * mm2m;
		urpose.z += 0.3 * mm2m;
		double blendRadius=1*mm2m;//交融半径

		std::string cmd_str;
		char buf[128];
		cmd_str = "def driverProg():\n";
		cmd_str += "\tmovej(p[" + to_string(urpose.x) + "," + to_string(urpose.y) +
			"," + to_string(urpose.z) + "," + to_string(urpose.rx) + "," +
			to_string(urpose.ry) + "," + to_string(urpose.rz) +
			"],a=2,v=2)\n";
		//mp p1
		urpose.x = 374.9*mm2m;
		urpose.y = -99.5*mm2m;
		urpose.z = 200.3*mm2m;
		cmd_str += "\tmovep(p[" + to_string(urpose.x) + "," + to_string(urpose.y) +
			"," + to_string(urpose.z) + "," + to_string(urpose.rx) + "," +
			to_string(urpose.ry) + "," + to_string(urpose.rz) +
			"],a=1.2,v=0.25,r=0.001)\n";


		cmd_str += "end\n";
		urd.setMonitorFlag();
		urd.sendCmd(cmd_str);
	}

	void mjp(const URPose& urpose) {
		cmd_str = "movej(p[" + to_string(urpose.x) + "," + to_string(urpose.y) +
			"," + to_string(urpose.z) + "," + to_string(urpose.rx) + "," +
			to_string(urpose.ry) + "," + to_string(urpose.rz) +
			"],a=1.2,v=0.25)\n";
		urd.sendCmd(cmd_str);
		Sleep(2);
	}

	//将urposeq作为形参
	void mjp2(const vector<double> & urposeq) {
		cmd_str = "movej(p[" + to_string(urposeq[0]) + "," + to_string(urposeq[1]) +
			"," + to_string(urposeq[2]) + "," + to_string(urposeq[3]) + "," +
			to_string(urposeq[4]) + "," + to_string(urposeq[5]) +
			"],a=2,v=2)\n";
		urd.sendCmd(cmd_str);
		Sleep(2);
	}
	

	/*
	Function:  used to trans "file after interploted" into "file composed by sol[q1-q6]"
	i
	o
	m：
	*/
	void testIK() {


		/*先赋值为work Pose*/
		target.x = 200;
		target.y = -100;
		target.z = 200;
		target.rx = 2.225;
		target.ry = -2.2194;
		target.rz = 0;

		/*urpose_test为相对位置，work Pose + urpose_test即为最终目标位置*/
		URPose urpose_test;
		fileOpration fp_test("C:/Users/fx53v/Desktop/Do/test1.1000.txt");
		urpose_test=fp_test.getNexp();
		target.x += urpose_test.x;
		target.y += urpose_test.y;
		target.z += urpose_test.z;
		target.x *= mm2m;
		target.y *= mm2m;
		target.z *= mm2m;

		/*cal IK*/
		sol = IK(target, urd.getq()[5]);

		/*从sol中选择距离当前各轴位置最近的结果*/
		double disMin_test = 2 * PI * 6;
		double indexMin_test = 0;
		double disTemp_test = 0; 
		vector<double> qcur_test(6);
		qcur_test = urd.getq();
		
			/*由于一个角在加或减360度后有一等效角，需要将等效角考虑进去*/
		vector<double> qcur_test2(6);
		for (int i = 0; i < 6; i++) {
			if (qcur_test[i] > 0) {
				qcur_test2[i] = qcur_test[i] - 360;
			}
			else if (qcur_test[i] < 0) {
				qcur_test2[i] = qcur_test[i] + 360;
			}
			else//qcur_test[i]==0;
			{
				qcur_test2[i] = 360;
			}
		}
			/*找到距离当前各轴位置最近的结果*/
		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 6; j++) {
				disTemp_test += min(absd(r2d(sol[i][j]) - qcur_test[j]), absd(r2d(sol[i][j]) - qcur_test2[j]));
			}
			if (disMin_test > disTemp_test) {
				disMin_test = disTemp_test;
				indexMin_test = i;
			}
			disTemp_test = 0;
		}
			/*将该结果的各值更新为和qcur较近的等效角*/
		double equAngle;
		for (int j = 0; j < 6; j++) {
			if (sol[indexMin_test][j] > 0) {
				equAngle = sol[indexMin_test][j] - 2 * PI;
				if (absd(r2d(sol[indexMin_test][j]) - qcur_test[j]) > absd(r2d(equAngle) - qcur_test[j])) {
					sol[indexMin_test][j] = equAngle;
				}
			}
			else {//sol[indexMin_test][j] <= 0
				equAngle = sol[indexMin_test][j] + 2 * PI;
				if (absd(r2d(sol[indexMin_test][j]) - qcur_test[j]) > absd(r2d(equAngle) - qcur_test[j])) {
					sol[indexMin_test][j] = equAngle;
				}
			}
		}

		/*Print Result*/
		for (int j = 0; j < 6; j++) {
			if (CheckZeroThresh(sol[indexMin_test][j])) {
				sol[indexMin_test][j] = 0;
			}
			printf("%12.3f", r2d(sol[indexMin_test][j]));
		}
		printf("\n");

		mjq("empty", sol[indexMin_test]);
		return;
	}

	void dotrajNew(std::vector<double> inp_timestamps,
		std::vector<std::vector<double> > inp_positions,
		std::vector<std::vector<double> > inp_velocities) {
		fileOpration fp("/home/linux80/path/IKresult10000.txt");
		cout << "dotraj:move to start pos" << endl;
		string q_str = fp.getNexq_str();
		mjq(q_str);
		Sleep(5);
		urd.doTraj(inp_timestamps, inp_positions, inp_velocities);
	}
	//稀疏化(skip some q):once one joint reached deta 1 degree first,take
	//密集化(interplote between q):the joint that has biggest deta degree as the rule to be cut
	void interplote(int lineNum) {
		//‪C:\Users\XY\Desktop\path\IKresult200000.txt
		fileOpration fp("C:/Users/fx53v/Desktop/Do/IKresult200000.txt");
		const double sgap = 0.0085;//standard gap = 1degree
		/*put q into ram*/
		vector<vector<double>> q(lineNum);
		for (int i = 0; i < lineNum; i++) {
			q[i].resize(6);
			q[i] = fp.getNexq();
		}
		/*interplote and skip opration*/
		vector<vector<double>> q_result;
		vector<double> q_temp(6);//used in pushback
		vector<double> q1_(6);
		vector<double> q2_(6);
		q1_ = q[0];
		q_result.push_back(q1_);
		double gapTemp;
		for (int i = 1; i < lineNum; i++)
		{
			if ((gapTemp = gap2q(q1_, q[i])) > sgap) {//interplote
				q2_ = q[i];
				for (int j = 1; j < (((int)(gapTemp * m2um)) / ((int)(sgap * m2um))); j++) {
					q_temp = { q1_[0] - (q1_[0] - q2_[0]) * absd(sgap / gapTemp) * j,
							   q1_[1] - (q1_[1] - q2_[1]) * absd(sgap / gapTemp) * j,
							   q1_[2] - (q1_[2] - q2_[2]) * absd(sgap / gapTemp) * j,
							   q1_[3] - (q1_[3] - q2_[3]) * absd(sgap / gapTemp) * j,
							   q1_[4] - (q1_[4] - q2_[4]) * absd(sgap / gapTemp) * j,
							   q1_[5] - (q1_[5] - q2_[5]) * absd(sgap / gapTemp) * j };
					q_result.push_back(q_temp);
				}
				q_result.push_back(q2_);
				if (i + 1 < lineNum)//update q1_
					q1_ = q2_;
			}
			else
			{
				continue;
			}
		}
		q_result.push_back(q[lineNum - 1]);//in case not push last q because not reach last gap

		//put result into file
		ofstream fpo("C:/Users/fx53v/Desktop/Do/IKresultSgap.txt");
		if (!fpo) {
			cout << "WARN at interplote:fail to open IKresultSgap" << endl;
		}
		for (int i = 0; i < q_result.size(); i++)
		{
			fpo << q_result[i][0] << " " << q_result[i][1] << " " << q_result[i][2] << " " <<
				q_result[i][3] << " " << q_result[i][4] << " " << q_result[i][5] << endl;
		}
		fpo.close();

		//check q_result
		for (int i = 0; i < q_result.size() - 1; i++) {
			cout << "i:" << i << "  " << gap2q(q_result[i], q_result[i + 1]) << endl;;
		}
	}

	double gap2q(vector<double> q1, vector<double> q2) {
		double max = absd(q1[0] - q2[0]);
		for (int i = 1; i < 6; i++) {
			if (max < absd(q1[i] - q2[i])) {
				max = absd(q1[i] - q2[i]);
			}
		}
		return max;
	}
	double absd(const double& d1) {
		if (d1 < 0) {
			return -d1;
		}
		else
			return d1;
	}


	/*
	Function:	use skip and interplote operation to make a pose set cut as standard distance
	Input:      lineNum - number of poses
	Output:     poses after handling
	Main Idea:  record start pose , start skip until two poses' dis >= sdis , then interplote between two pose
				and push last pose . record last pose as start pose and start skip untill ......
	*/
	void interplote_p(int lineNum) {
		fileOpration fp("C:/Users/fx53v/Desktop/Do/hand2.txt");
		const double sdis = 0.1;//standard dis (mm)
		/*put q into ram*/
		vector<URPose> p(lineNum);
		for (int i = 0; i < lineNum; i++) {
			p[i] = fp.getNexp();
		}
		/*interplote and skip opration*/
		vector<URPose> pResult;
		URPose pTemp, p1, p2;
		p1 = p[0];
		pResult.push_back(p1);
		double distemp;
		for (int i = 1; i < lineNum; i++)
		{
			if ((distemp = dis2p(p1, p[i])) > sdis) {//interplote
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
		ofstream fpo("C:/Users/fx53v/Desktop/Do/hand2_interploted.txt");
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
			cout << "i:" << i << "  " << dis2p(pResult[i], pResult[i + 1]) << endl;;
		}
	}


	double dis2p(const URPose& p1, const URPose& p2) {
		return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
	}

	// read q pos file and send servoj at 125HZ
	//falg_mj is used to check if our ik is right
	void dotraj(bool flag_mj = 0, double t = 0.008) {//改t暂时要去ur_driver的upload_prog里改
		fileOpration fp("C:/Users/fx53v/Desktop/Do/hand2_afterIK.txt");
		/*INPUT 行数*/
		int size_ = 17041;
		//cin >> size_;
		/*INPUT servoj time*/
		cout << "INPUT servoj time" << endl;
		cin >> t;
		urd.setServojTime(t);
		/*INPUT lookAheadTime*/
		double lookAheadTime = 0.2;
		//cout << "INPUT lookAheadTime" << endl;
		//cin >> lookAheadTime;
		urd.setLookaheadTime(lookAheadTime);
		/*INPUT sleep time*/
		cout << "INPUT sleepTime" << endl;
		double sleepTime = 1;
		cin >> sleepTime;
		/*movej to start pose*/
		cout << "dotraj:move to start pos" << endl;
		string q_str = fp.getNexq_str();
		mjq(q_str);
		Sleep(2000);
		/*upload prog*/
		urd.uploadProg();
		vector<double> q_d(6);
		/*send servoj or movej*/
		executeTraj = 1;
		if (flag_mj == 0) {
			// start send servoj
			cout << "dotraj:start send servoj" << endl;
			//while (fp.isEnd() != 1) {
			for (int i = 0; i < size_ - 1; i++) {
				q_d = fp.getNexq();
				fp.qToDegree(q_d);//辅助打印函数
				urd.servoj(q_d, 1);
				//q_str = fp.getNexq_str();
				//fp.qstrToDegree(q_str);
				//servoj(q_str,t);
				Sleep(sleepTime);
			}
			Sleep(10); 
		}
		else//flag_mj==1
		{
			// start send mj
			cout << "dotraj:start send mj" << endl;
			//while (fp.isEnd() != 1) {
			for (int i = 0; i < size_ - 1; i++) {
				q_str = fp.getNexq_str();
				mjq(q_str);
				Sleep(2);
			}
		}
		cout << "total ik pos :" << fp.getlineNum() << endl;
		executeTraj = 0;
	}

	void uploadProg_servoj() {
		int lineNum = 529;
		ifstream ifp("C:/Users/fx53v/Desktop/Do/hand2_afterIK.txt");
		ifstream ifp2("C:/Users/fx53v/Desktop/Do/hand2_interploted.txt");//hand2_interploted.txt:IK解对应的pose文档
		double dis = 0.1;//dis of interploted
		if (ifp.fail()) {
			cout << "ERROR IN uploadProgram _" << __LINE__ << ": fail to open ifp" << endl;
			system("pause");
			exit(0);
		}
		if (ifp2.fail()) {
			cout << "ERROR IN uploadProgram _" << __LINE__ << ": fail to open ifp2" << endl;
			system("pause");
			exit(0);
		}

		cout << "input v in tool space" << endl;
		double v=0,t,a;//v=a/(theta+1)。a is the speed when theta=0
		cin >> v;
		a = v;
		vector<double> q;
		Vector3d pb,pc,pn;//pbefore pcurrent pnext
		q.resize(6);

		cout << "input look ahead time" << endl;
		double lookahead_time;
		cin >> lookahead_time;

		cout << "input gain between 100 and 2000,input 0 to not use gain" << endl;
		double gain;
		cin >> gain;

		cout << "move to satrt q" << endl;
		ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
		ifp2 >> pb[0] >> pb[1] >> pb[2];
		ifp2 >> pc[0] >> pc[1] >> pc[2];
		mjq("empty", q);
		Sleep(2000);

		cmd_str = "def driverProg():\n";
		for (int i = 1; i < lineNum-1; i++) {//最后一个点对应的q[lineNum-1]不读了，因为此时已经把hand2_interpolated.txt的最后一个点读进pn了。
			ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
			ifp2 >> pn[0] >> pn[1] >> pn[2];
			//v = a / (3*d2r(calAngleOFvector(pc - pb, pn - pc)) + 1);//根据夹角计算速度。
			t = sqrt((pc[0] - pb[0]) * (pc[0] - pb[0]) + (pc[1] - pb[1]) * (pc[1] - pb[1]) + (pc[2] - pb[2]) * (pc[2] - pb[2])) / v;//t=d/v
			//t = 0.008;//来自monitorLog的时间间隔
			pb = pc;
			pc = pn;
			if (gain == 0) {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + ")\n";
			}
			else {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + "," + "gain=" + to_string(gain) + ")\n";
			}




		}
		cmd_str += "end\n";

		cout << "start move with servoj" << endl;
		urd.sendCmd(cmd_str);
		openMonitor();
	}


	//功能和uploadProg_servoj()一致，但是不需要手动调整行数了
	//目标点集写入automatic_targetPose.txt
	void uploadProg_servoj2() {
		//pieceWise();
		//double dis = 0.1;//dis of interploted
		automatic_interplote_p(0);//插补
		automatic_getIKsolution();//IK

		//打开文件
		ifstream ifp("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
		ifstream ifp2("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated.txt");//hand2_interploted.txt:IK解对应的pose文档
		if (ifp.fail()) {
			cout << "ERROR IN uploadProgram _" << __LINE__ << ": fail to open ifp" << endl;
			system("pause");
			exit(0);
		}
		if (ifp2.fail()) {
			cout << "ERROR IN uploadProgram _" << __LINE__ << ": fail to open ifp2" << endl;
			system("pause");
			exit(0);
		}
		int lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt", "");


		cout << "input v in tool space" << endl;
		double v = 0, t;
		cin >> v;
		vector<double> q;
		Vector3d pb, pc;//pbefore pcurrent
		q.resize(6);

		cout << "input look ahead time" << endl;
		double lookahead_time;
		cin >> lookahead_time;

		cout << "input gain between 100 and 2000,input 0 to not use gain" << endl;
		double gain;
		cin >> gain;

		cout << "move to satrt q" << endl;
		ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
		ifp2 >> pb[0] >> pb[1] >> pb[2];
		mjq("empty", q);
		Sleep(2000);

		cmd_str = "def driverProg():\n";
		for (int i = 1; i < lineNum; i++) {
			ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
			ifp2 >> pc[0] >> pc[1] >> pc[2];
			t = sqrt((pc[0] - pb[0]) * (pc[0] - pb[0]) + (pc[1] - pb[1]) * (pc[1] - pb[1]) + (pc[2] - pb[2]) * (pc[2] - pb[2])) / v;//t=d/v
			//t = 0.008;//来自monitorLog的时间间隔
			pb = pc;
			if (gain == 0) {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + ")\n";
			}
			else {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + "," + "gain=" + to_string(gain) + ")\n";
			}
		
		}
		cmd_str += "end\n";

		cout << "start move with servoj" << endl;
		urd.setMonitorFlag();
		urd.sendCmd(cmd_str);
		while (urd.getMonitorFlag() == 1) {};//wait till ur3 stop
		
		//calReachPercent
		vector<double> ResultOF_calReachPercent(2);
		ResultOF_calReachPercent = automatic_calReachPercent();
		cout << "Reach Percent at v=" << v << ",lookahead_time=" << lookahead_time << " is:" << ResultOF_calReachPercent[0] <<endl;
		cout << "( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数=" << ResultOF_calReachPercent[1] << endl;
		//显示Miss点
		lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt", "");
		cout << "There are "<<lineNum<<" Miss Points" << endl;
		if (lineNum > 0) {
			ifstream ifp_missed("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt");
			checkfp(ifp_missed);
			for (int i = 0; i < lineNum; i++) {
				ifp_missed >> pc[0] >> pc[1] >> pc[2];
				cout << pc[0] << " " << pc[1] << " " << pc[2] << endl;
			}
		}
	}

	//增加了对pieceWise_control文件的读取
	//目标点集写入automatic_targetPose.txt
	void uploadProg_servoj3() {
		//pieceWise();
		//double dis = 0.1;//dis of interploted
		automatic_interplote_p(0);//插补
		automatic_getIKsolution();//IK

		//打开文件
		ifstream ifp("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
		ifstream ifp2("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated.txt");//hand2_interploted.txt:IK解对应的pose文档
		if (ifp.fail()) {
			cout << "ERROR IN uploadProgram _" << __LINE__ << ": fail to open ifp" << endl;
			system("pause");
			exit(0);
		}
		if (ifp2.fail()) {
			cout << "ERROR IN uploadProgram _" << __LINE__ << ": fail to open ifp2" << endl;
			system("pause");
			exit(0);
		}
		int lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt", "");

		/*q和p的读取*/
		vector<double> q;
		q.resize(6);
		Vector3d pb, pc;//pbefore pcurrent
		cout << "move to satrt q" << endl;
		ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
		ifp2 >> pb[0] >> pb[1] >> pb[2];
		mjq("empty", q);
		Sleep(2000);

		/*piece Control读取*/
		ifstream ifp_pieceWiseControl("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/pieceWise_control.txt");
		checkfp(ifp_pieceWiseControl);
		int piece_Start, piece_End;
		double piece_v, piece_lookAhead_time;
		ifp_pieceWiseControl >> piece_Start >> piece_End >> piece_v >> piece_lookAhead_time;
		
		double v = piece_v, t;
		double lookahead_time=piece_lookAhead_time;
		double gain = 0;

		cmd_str = "def driverProg():\n";
		for (int i = 1; i < lineNum; i++) {
			//piece control
			if (i > piece_End) {//注意pieceWiseControl文件中的piece_start和piece_end指的是下标！！！
				ifp_pieceWiseControl >> piece_Start >> piece_End >> piece_v >> piece_lookAhead_time;
				v = piece_v;
				lookahead_time = piece_lookAhead_time;
			}

			ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
			ifp2 >> pc[0] >> pc[1] >> pc[2];
			t = sqrt((pc[0] - pb[0]) * (pc[0] - pb[0]) + (pc[1] - pb[1]) * (pc[1] - pb[1]) + (pc[2] - pb[2]) * (pc[2] - pb[2])) / v;//t=d/v
			//t = 0.008;//来自monitorLog的时间间隔
			pb = pc;
			if (gain == 0) {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + ")\n";
			}
			else {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + "," + "gain=" + to_string(gain) + ")\n";
			}

		}
		cmd_str += "end\n";

		cout << "start move with servoj" << endl;
		urd.setMonitorFlag();
		urd.sendCmd(cmd_str);
		while (urd.getMonitorFlag() == 1) {};//wait till ur3 stop

		//calReachPercent
		vector<double> ResultOF_calReachPercent(2);
		ResultOF_calReachPercent = automatic_calReachPercent();
		cout << "Reach Percent at v=" << v << ",lookahead_time=" << lookahead_time << " is:" << ResultOF_calReachPercent[0] << endl;
		cout << "( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数=" << ResultOF_calReachPercent[1] << endl;
		//显示Miss点
		lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt", "");
		cout << "There are " << lineNum << " Miss Points" << endl;
		if (lineNum > 0) {
			ifstream ifp_missed("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt");
			checkfp(ifp_missed);
			for (int i = 0; i < lineNum; i++) {
				ifp_missed >> pc[0];//点对应的行号
				cout << pc[0] << " ";
				ifp_missed >> pc[0] >> pc[1] >> pc[2];
				cout << pc[0] << " " << pc[1] << " " << pc[2] << endl;
			}
		}
	}

	
	//1.首先将automatic_sheet.txt读入automatic_sheet类。然后由该表指导各点参数的生成
	//2.每一个点的前瞻时间和速度，需要根据其后面若干点的弯折角度最大值确定，至于若干点具体是几个点，我们使用距离参数L来确定，从当前点开始往后，对相邻点距进行累加，
	//直到当到第n个点时，点距和大于等于L。然后比较当前点到n点各点的弯折角度，返回当前点到n点的最大弯折角度，各点中的最大弯折角度用来确认当前点的前瞻时间和速度
	void uploadPorg_servoj4() {
		//读表：automatic_SHEET
		//读目标点:automatic_TARGET_POINTS
		automatic_sheet paramSheet;
		automatic_TARGETS_POINTS targetPoints;
		
		//double dis = 0.1;//dis of interploted
		automatic_interplote_p(0);//形参输入为0，表示不插补
		automatic_getIKsolution();//IK

		//打开文件
		int lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt", "");
		ifstream ifp("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
		ifstream ifp2("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated.txt");//hand2_interploted.txt:IK解对应的pose文档
		checkfp(ifp);
		checkfp(ifp2);

		/*q和p的读取*/
		vector<double> q;
		q.resize(6);
		Vector3d pb, pc, pn;//pbefore pcurrent pnext
		cout << "move to satrt q" << endl;
		ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
		ifp2 >> pb[0] >> pb[1] >> pb[2];
		mjq("empty", q);
		Sleep(2000);

		double v, t, lookahead_time, gain=0;
		vector<double> tableLine(3);//表的一行（表的一行有三项
		double max_turnAngle;
		double L = paramSheet.getL();

		cmd_str = "def driverProg():\n";
		for (int i = 1; i < lineNum; i++) {
			ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
			ifp2 >> pc[0] >> pc[1] >> pc[2];

			/*确定v和lookahead_time*/
			max_turnAngle = targetPoints.getMaxTurnAngle(i, L);
			tableLine = paramSheet.matchWithTurnAngle(max_turnAngle);
			v = tableLine[1];
			lookahead_time = tableLine[2];

			t = sqrt((pc[0] - pb[0]) * (pc[0] - pb[0]) + (pc[1] - pb[1]) * (pc[1] - pb[1]) + (pc[2] - pb[2]) * (pc[2] - pb[2])) / v;//t=d/v
			//t = 0.008;//来自monitorLog的时间间隔
			pb = pc;
			if (gain == 0) {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + ")\n";
			}
			else {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + "," + "gain=" + to_string(gain) + ")\n";
			}

		}
		cmd_str += "end\n";

		cout << "start move with servoj" << endl;
		urd.setMonitorFlag();
		urd.sendCmd(cmd_str);
		while (urd.getMonitorFlag() == 1) {};//wait till ur3 stop

		//calReachPercent
		vector<double> ResultOF_calReachPercent(2);
		ResultOF_calReachPercent = automatic_calReachPercent();
		cout << "Reach Percent at v=" << v << ",lookahead_time=" << lookahead_time << " is:" << ResultOF_calReachPercent[0] << endl;
		cout << "( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数=" << ResultOF_calReachPercent[1] << endl;
		//显示Miss点
		lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt", "");
		cout << "There are " << lineNum << " Miss Points" << endl;
		if (lineNum > 0) {
			ifstream ifp_missed("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt");
			checkfp(ifp_missed);
			for (int i = 0; i < lineNum; i++) {
				ifp_missed >> pc[0];//点对应的行号
				cout << pc[0] << " ";
				ifp_missed >> pc[0] >> pc[1] >> pc[2];
				cout << pc[0] << " " << pc[1] << " " << pc[2] << endl;
			}
		}
	}



	/*
	Function:  used in automatic adjust parameter
	i:lineNum of targetPose_interpolated_afterIK_file
	o
	m:cmd_str在作为形参调用进来之前，要先cmd_str-="end\n"
	*/
	string automatic_uploadProg_servoj(double v,double lookahead_time,double gain,string cmd_str) {
		//int lineNum = 529;
		ifstream ifp("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
		ifstream ifp2("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated.txt");//hand2_interploted.txt:IK解对应的pose文档
		int lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt", "");
		double dis = 0.1;//dis of interploted
		if (ifp.fail()) {
			cout << "ERROR IN uploadProgram _" << __LINE__ << ": fail to open ifp" << endl;
			system("pause");
			exit(0);
		}
		if (ifp2.fail()) {
			cout << "ERROR IN uploadProgram _" << __LINE__ << ": fail to open ifp2" << endl;
			system("pause");
			exit(0);
		}

		/*cout << "input v in tool space" << endl;*/
		double t, a;//v=a/(theta+1)。a is the speed when theta=0
		a = v;
		vector<double> q;
		Vector3d pb, pc, pn;//pbefore pcurrent pnext
		q.resize(6);

		/*cout << "input look ahead time" << endl;*/
		/*cout << "input gain between 100 and 2000,input 0 to not use gain" << endl;*/

		cout << "move to satrt q" << endl;
		ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
		ifp2 >> pb[0] >> pb[1] >> pb[2];
		ifp2 >> pc[0] >> pc[1] >> pc[2];
		urd.setMonitorFlag();
		mjq("empty", q);
		while (urd.getMonitorFlag() == 1) {};//wait till finish move

		if (cmd_str == "")
			cmd_str = "def driverProg():\n";
		assert(cmd_str[cmd_str.size() - 4] != 'e' && cmd_str[cmd_str.size() - 3] != 'n' && cmd_str[cmd_str.size() - 2] != 'd');//cmd_str在作为形参调用进来之前，要先cmd_str-="end\n"
		for (int i = 1; i < lineNum - 1; i++) {//最后一个点对应的q[lineNum-1]不读了，因为此时已经把hand2_interpolated.txt的最后一个点读进pn了。
			ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
			ifp2 >> pn[0] >> pn[1] >> pn[2];
			//v = a / (3*d2r(calAngleOFvector(pc - pb, pn - pc)) + 1);//根据夹角计算速度。
			t = sqrt((pc[0] - pb[0]) * (pc[0] - pb[0]) + (pc[1] - pb[1]) * (pc[1] - pb[1]) + (pc[2] - pb[2]) * (pc[2] - pb[2])) / v;//t=d/v
			//t = 0.008;//来自monitorLog的时间间隔
			pb = pc;
			pc = pn;
			if (gain == 0) {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + ")\n";
			}
			else {
				cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
					"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
					to_string(q[4]) + "," + to_string(q[5]) +
					"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + "," + "gain=" + to_string(gain) + ")\n";
			}
		}
		/*最后一个点对应的q[lineNum-1]*/
		ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
		t = sqrt((pn[0] - pc[0]) * (pn[0] - pc[0]) + (pn[1] - pc[1]) * (pn[1] - pc[1]) + (pn[2] - pc[2]) * (pn[2] - pc[2])) / v;//t=d/v
		if (gain == 0) {
			cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
				"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
				to_string(q[4]) + "," + to_string(q[5]) +
				"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + ")\n";
		}
		else {
			cmd_str += "\tservoj([" + to_string(q[0]) + "," + to_string(q[1]) +
				"," + to_string(q[2]) + "," + to_string(q[3]) + "," +
				to_string(q[4]) + "," + to_string(q[5]) +
				"]," + "t=" + to_string(t) + "," + "lookahead_time=" + to_string(lookahead_time) + "," + "gain=" + to_string(gain) + ")\n";
		}
		
		cmd_str += "end\n";

		cout << "start move with servoj" << endl;
		urd.setMonitorFlag();
		urd.sendCmd(cmd_str);
		return cmd_str;
	}
	
	/*
	Function:  找到“一段轨迹的最优”。测试startv[mm/s]到endv[mm/s]的速度下的最优前瞻参数
	i：该段轨迹在automatic_targetPose.txt文件中
	o
	m：cmd_str_original在作为形参调用进来之前，要先cmd_str-="end\n"
	新增：形参startLineNumber，指明target.txt的第几行开始是沿用段，从而实现让automatic_adjust_parameter只将沿用段追加写入automatic_targetPose_calReach Percent.txt
	*/
	vector<automatic_pieceResult_Struct> automatic_adjust_parameter(double startv,double endv,string cmd_str_original,int startLineNumber) {
		//1.人工将目标点写入automatic_targetPose.txt
		string targetPosefile_addr = "C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose.txt";
		int targetPosefile_lineNumber = getLineNumber(targetPosefile_addr,"");
		string targetPosefile_interpolated_addr = "C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated.txt";
		string targetPosefile_interpolated__IK_addr = "C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt";
		string actualPosefile_addr = "C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_actual.txt";
		string monitorFile_addr = "C:/Users/fx53v/Desktop/Do/monitorLog.txt";
		//将目标点集根据具体情况，写入automatic_targetPose_calReachPercent.txt（calReachPercent的ifp）。
		ifstream ifp(targetPosefile_addr);
		checkfp(ifp);
		ofstream ofp;
		int temp_i = getLineNumber(targetPosefile_addr,"");
		Vector3d temp_v3;
		if (cmd_str_original == "") {//若起始命令为空，则覆盖写入
			ofp.open("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_calReachPercent.txt");
			checkfp(ofp);
			for (int i = 0; i < temp_i; i++) {
				ifp >> temp_v3[0] >> temp_v3[1] >> temp_v3[2];
				ofp << temp_v3[0] << " " << temp_v3[1] << " " << temp_v3[2] << endl;
			}
		}
		else {//若起始命令非空，则续写写入
			ofp.open("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_calReachPercent.txt",ios::app);
			checkfp(ofp);
			for (int i = 0; i < startLineNumber-1; i++) {//先将目标段读掉
				ifp >> temp_v3[0] >> temp_v3[1] >> temp_v3[2];
			}
			for (int i = 0; i < temp_i-startLineNumber+1; i++) {//再将沿用段追加
				ifp >> temp_v3[0] >> temp_v3[1] >> temp_v3[2];
				ofp << temp_v3[0] << " " << temp_v3[1] << " " << temp_v3[2] << endl;
			}
		}
		ifp.close();
		ofp.close();

		//2.插补
		automatic_interplote_p(0.05);

		//3.calIK(目标点文件地址，插补文件的行数)
		int targetPosefile_interpolated_lineNumber = getLineNumber(targetPosefile_interpolated_addr,"");
		automatic_getIKsolution();

		//4.进行uservoj,使用monitorFlag1。并计算至点率
		/*对monitorFlag1的修改：
		（1）增加了类成员notChangeCount_Max，从而可以更灵活地调整过多久机械臂不动才认为是结束运动了。
		（2）由于在上传脚本程序后，机械臂会经过若干秒才会运动，因此对于monitorFlag=1刚置1的一段时间，机械臂一定是静止的，
		所以增加了类成员startMoveFlag，最初由于startMoveFlag=0，因此机械臂无论花多少时间去处理脚本程序，都不会判定为结束运动。
		*/
		urd.setNotChangeCount_Max(0.5);
		//double startv = 10, endv = 20;//测试startv[mm/s]到endv[mm/s]的速度
		const double gapv=1;
		//vector<string> str_result;//存放结果
		vector<automatic_pieceResult_Struct> resultOF_function;//存放结果

		for (double v = startv; v <= endv; v += gapv) {
			//调节look_ahead_time
			/*变量定义*/
			double s = 0.03, e = 0.2, gap1 = 0.02;//设置gap1=0.02,gap2=0.01,最多只需13次即可完成寻找最优look_ahead_time。优于只有gap1=0.01的20次。
			double maxReachPercent = 0, maxReachPercent_result1 = 10, maxReachPercent_lookAheadTime = s;//maxReachPercent_result1即maxReachPercent对应的( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数。
			int maxReachPercent_missNumTotal=0;
			string maxReachPercent_cmd_str = "", temp_s = "";
			automatic_pieceResult_Struct temp_result;
			vector<double> temp;//临时记录automatic_calReachPercent()的结果
			/*找最佳前瞻时间*/
			for (double i = s; i <= e; i = i + gap1) {//找到gap1下[0.03,0.2]之间的最佳前瞻时间
				temp_s=automatic_uploadProg_servoj(v, i, 0,cmd_str_original);
				while (urd.getMonitorFlag() == 1) {};
				temp = automatic_calReachPercent();
				if (temp[0] > maxReachPercent) {
					maxReachPercent = temp[0];
					maxReachPercent_result1 = temp[1];
					maxReachPercent_lookAheadTime = i;
					maxReachPercent_cmd_str = temp_s;
					maxReachPercent_missNumTotal = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt", "");
				}
				//当至点度相同时，比较( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数。
				else if (temp[0] == maxReachPercent && temp[1] < maxReachPercent_result1) {
					maxReachPercent = temp[0];
					maxReachPercent_result1 = temp[1];
					maxReachPercent_lookAheadTime = i;
					maxReachPercent_cmd_str = temp_s;
					maxReachPercent_missNumTotal = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt", "");
				}
				else {}
			}
			//找到gap2下[当前最佳前瞻时间 - gap1 , 当前最佳前瞻时间 + gap1]之间的最佳前瞻时间
			double gap2 = 0.01;
			double gap1_maxReachPercent = maxReachPercent_lookAheadTime;//gap1下的最佳前瞻时间
			double temp_d = maxReachPercent_lookAheadTime - gap1;//temp_d作为下面for循环i的初始值
			if (temp_d < s)
				temp_d = s;
			for (double i = temp_d; i <= maxReachPercent_lookAheadTime + gap1; i = i + gap2) {
				if (i == gap1_maxReachPercent) {//若为gap1下的最佳前瞻时间，由于在gap1的测试中已经试过了，因此跳过
					continue;
				}
				temp_s = automatic_uploadProg_servoj(v, i, 0, cmd_str_original);
				while (urd.getMonitorFlag() == 1) {};
				temp = automatic_calReachPercent();
				if (temp[0] > maxReachPercent) {
					maxReachPercent = temp[0];
					maxReachPercent_result1 = temp[1];
					maxReachPercent_lookAheadTime = i;
					maxReachPercent_cmd_str = temp_s;
					maxReachPercent_missNumTotal = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt", "");
				}
				else if (temp[0] == maxReachPercent && temp[1] < maxReachPercent_result1) {//当至点度相同时，比较( ∑calDis(至点对应的实际点，至点对应的目标点) )/至点数。
					maxReachPercent = temp[0];
					maxReachPercent_result1 = temp[1];
					maxReachPercent_lookAheadTime = i;
					maxReachPercent_cmd_str = temp_s;
					maxReachPercent_missNumTotal = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_Missed.txt", "");

				}
				else {}
			}
			/*记录本轮循环结果*/
			//str_result.push_back(to_string(v) + "mm/s:" + to_string(maxReachPercent) + " when look_ahead_time=" + to_string(maxReachPercent_lookAheadTime));
			temp_result.speed = v;
			temp_result.look_ahead_time = maxReachPercent_lookAheadTime;
			temp_result.reachPercent = maxReachPercent;
			temp_result.missedNumTotal = maxReachPercent_missNumTotal;
			temp_result.calReachPercent_result1 = maxReachPercent_result1;
			temp_result.cmd_str = maxReachPercent_cmd_str;
			resultOF_function.push_back(temp_result);
		}

		assert(resultOF_function.size() == ((endv - startv) / gapv + 1));
		//for (int i = 0; i < str_result.size(); i++) {
			//cout << str_result[i] << endl;
		//}
		return resultOF_function;
	}
	

	/*
	Function:  找到“分段轨迹的最优”
	i
	o
	m：每次在当前段最优分析完以后，对最优cmd_str进行cmt_str-="end\n"，来进行续接下一段轨迹的servoj命令。
	新增：在进行使用深度学习进行调参的时候，为了实现数据集的获得，需要对函数进行一定修改。因此新增flag形参，当flag=1的时候，表示此时是在进行数据集的获得
	*/
	void automatic_adjust_parameter_OFpieceWise(const int flag=0) {
		pieceWise();//对whole_path.txt分段
		int lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_pieceWisePath.txt", "");//分段文件行数
		ifstream ifp("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_pieceWisePath.txt");//各分段的点集文件
		checkfp(ifp);
		ofstream ofp1;//进行单段分析的目标点集
		ofstream ofp2("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_result.txt");//各段采用的最佳前瞻时间，速度，至点率，未至点，未至点数，总未至点数
		checkfp(ofp2);
		ofstream ofp3("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_resultLog.txt");//记录各段的各速度下对应的最佳前瞻时间
		checkfp(ofp3);

		int spi, epi;//startPoseindex, endPoseIndex
		Vector3d p;
		vector<automatic_pieceResult_Struct> temp_result;
		string temp_s = "";

		assert(lineNum > 1);
		int lineAlready_Read = 0;//已读行数
		long posOF_file;//用来保存文件指针位置
		int startLineNumberOF_ContinueToUse = 1;//下一个目标点集沿用段的起始行数。automatic_targetPose_calReach Percent.txt文件在追加的时候，直到target.txt文件的该行才开始写入
		Vector3d startPointOF_ContinueToUse;//当前目标点集沿用段起始点
		int lineNumOF_interpolatedFile;//插补点集行数
		int temp_i;
		Vector3d temp_v;
		ifstream temp_ifp;
		/*逐段分析*/
		do
		{
			/*将当前分段写入"进行单段分析的目标点集目标点集"*/
			ifp >> spi >> epi;
			ofp1.open("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose.txt");//进行单段分析的目标点集
			checkfp(ofp1);
			if (flag == 1 && lineAlready_Read != 0) {
				ofp1 << startPointOF_ContinueToUse[0] << startPointOF_ContinueToUse[1] << startPointOF_ContinueToUse[2];
			}
			for (int j = 0; j < epi - spi; j++) {//将当前分段写入"进行单段分析的目标点集文件"
				ifp >> p[0] >> p[1] >> p[2];
				ofp1 << p[0] << " " << p[1] << " " << p[2] << endl;
			}
			if (flag != 1) {//当前轨迹分段的末尾点，如果flag=1即进行深度学习数据集获取，那么就应该是沿用段的首点
				ifp >> p[0] >> p[1] >> p[2];
				ofp1 << p[0] << " " << p[1] << " " << p[2] << endl;
			}
			//将沿用段写入目标点集文件，并置回文件指针
			if (flag == 1) {
				ifp >> p[0] >> p[1] >> p[2];
				ofp1 << p[0] << " " << p[1] << " " << p[2] << endl;
				startPointOF_ContinueToUse = p;//沿用段起始点的赋值

				lineAlready_Read += 1 + epi - spi + 1;//该分段的描述行+该分段点数
				if (lineAlready_Read >= lineNum) {//在继续读沿用段之前先判断文件是否已经读完
					break;
				}

				posOF_file = ifp.tellg();//保存当前分段轨迹文件指针位置
				ifp >> spi >> epi;
				for (int j = 0; j < epi - spi+1; j++) {//将沿用段写入目标点集文件
					ifp >> p[0] >> p[1] >> p[2];
					ofp1 << p[0] << " " << p[1] << " " << p[2] << endl;
				}
				
				ifp.seekg(posOF_file);
			}
			ofp1.close();
			
			/*调用函数获得从startv到endv各速度下的最佳前瞻时间*/
			temp_result = automatic_adjust_parameter(10, 10, temp_s, startLineNumberOF_ContinueToUse);//首次要将目标点和沿用段都加入，因此startLineNumberOF_ContinueToUse初始值为1
			if (flag == 1) {
				startLineNumberOF_ContinueToUse = epi - spi + 1+1+1;//1+1+1即表示 沿用首点，沿用段点数（不包括沿用首点），下一行
			}
			/*将各个速度下对应的结果写入automatic_resultLog.txt*/
			ofp3 << spi << " " << epi << " " << endl;
			for (int i = 0; i < temp_result.size(); i++) {
				ofp3 << temp_result[i].speed << "mm/s ";
				ofp3 << temp_result[i].look_ahead_time << "s ";
				ofp3 << temp_result[i].reachPercent * 100.0 << "% ";
				ofp3 << temp_result[i].missedNumTotal << " ";
				ofp3 << temp_result[i].calReachPercent_result1 << "mm" << endl;
			}
			/*对automatic_adjust_parameter函数得到的结果进行分析，选择合适的速度对应的cmd_str，然后用该cmd_str来作为uservoj的形参*/
			temp_s = temp_result[0].cmd_str;//先直接选10mm/s
			temp_s.resize(temp_s.size() - 4);//temp_s-="end\n"
			//将temp_s中沿用段的部分删除
			if (flag == 1) {
				//由于cmd_str是插补点集对应的cmd,因此要删除多少个servoj，应该看在插补点集中，沿用段的点数
				//要看在插补点集中沿用段对应的个数，直接找到“插补.txt文件中最接近底部的”与“沿用段第一个点相等的行号”，“插补文件总行数-该行号”即沿用段在插补点集中的个数，即要删除的servoj个数
				//1.首先获得要删除的servoj个数
				lineNumOF_interpolatedFile = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated.txt","");
				temp_i = 0;
				temp_ifp.open("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated.txt");
				checkfp(temp_ifp);
				for (int i = 1; i <= lineNumOF_interpolatedFile; i++) {//找到“插补.txt文件中最接近底部的”与“沿用段第一个点相等的行号”
					temp_ifp >> temp_v[0] >> temp_v[1] >> temp_v[2];
					if (temp_v == startPointOF_ContinueToUse) {
						temp_i = i;
					}
				}
				temp_ifp.close();
				assert(temp_i != 0);
				//2.删除对应个数的servoj
				for (int i = 0; i < lineNumOF_interpolatedFile - temp_i; i++) {
					temp_s.resize(temp_s.rfind("servoj"));
				}
			}

			lineAlready_Read += 1 + epi - spi + 1;//该分段的描述行+该分段点数
		} while (lineAlready_Read < lineNum);
		
		ifp.close();
		ofp2.close();
		ofp3.close();
	}



	/*
	Function:  检查IK运算是否正确。
	i
	o
	m：通过对一个个点进行movej来检查，打断点并步进发送movej指令
	*/
	void checkIK() {
		int lineNum = getLineNumber("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt", "");
		ifstream ifp("C:/Users/fx53v/Desktop/Do/automaticAdjustParameter/automatic_targetPose_interpolated_afterIK.txt");
		ifstream ifp2("C:/Users/fx53v/Desktop/Do/hand2_interploted.txt");
		if (ifp.fail()) {
			cout << "ERROR IN checkIK _" << __LINE__ << ": fail to open ifp" << endl;
			system("pause");
			exit(0);
		}
		if (ifp2.fail()) {
			cout << "ERROR IN checkIK _" << __LINE__ << ": fail to open ifp2" << endl;
			system("pause");
			exit(0);
		}
		URPose workPose(0.2, -0.1, 0.2, 2.225, -2.2194, 0);//起始姿态:startPose
		workPose.x *= m2mm;
		workPose.y *= m2mm;
		workPose.z *= m2mm;

		vector<double> q,pc,pt;//p current,p target
		q.resize(6);
		pc.resize(6);
		pt.resize(6);
		for (int i = 0; i < lineNum; i++) {
			ifp >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5];
			mjq("empty", q);
			Sleep(2000);
			pc = urd.getp();//current p
			ifp2 >> pt[0] >> pt[1] >> pt[2];//target p
			pt[0] += workPose.x;
			pt[1] += workPose.y;
			pt[2] += workPose.z;
			cout <<i<<": "<< sqrt((pt[0] - pc[0]) * (pt[0] - pc[0]) + (pt[1] - pc[1]) * (pt[1] - pc[1]) + (pt[2] - pc[2]) * (pt[2] - pc[2])) << endl;
		}
	}

	void update_loop() {
		while (1) {
			urd.update_qp();
		}
	}
	//得到d1，a2，a3，d4，d5，d6
	vector<double> getIKparam() {
		vector<double> IKparam(6);
		vector<double> temp(6);

		urd.sendCmd("movej([0, 0, 0, d2r(-90), 0, 0], a = 2, v = 2)\n");
		Sleep(4000);
		temp = urd.getp();
		IKparam[0] = absd(temp[2]);

		urd.sendCmd("movej([0, 0, 0, 0, d2r(90), 0], a = 2, v = 2)\n");
		Sleep(4000);
		temp = urd.getp();
		IKparam[3] = absd(temp[1]);

		urd.sendCmd("movej([0, 0, d2r(-90), 0, 0, 0], a = 2, v = 2)\n");
		Sleep(4000);
		temp = urd.getp();
		IKparam[2] = absd(absd(temp[2]) - IKparam[0]);

		urd.sendCmd("movej([0, 0, 0, 0, 0, 0], a = 2, v = 2)\n");//home
		Sleep(4000);
		temp = urd.getp();
		IKparam[1] = absd(absd(temp[0]) - IKparam[2]);
		IKparam[5] = absd(absd(temp[1]) - IKparam[3]);
		IKparam[4] = absd(IKparam[0] - absd(temp[2]));

		cout << IKparam[0] * mm2m << " " << IKparam[1] * mm2m << " " << IKparam[2] * mm2m << " " << IKparam[3] * mm2m << " " << IKparam[4] * mm2m << " " << IKparam[5] * mm2m << endl;
		return IKparam;
	}
	//static void* update_loopHelper(void* contex) {
	//	return ((controlReal*)contex)->update_loop();
	//}
};
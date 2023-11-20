#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <math.h>
#include <sys/types.h>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "socket.h"
#include "trans.h"
#include "fileOpration.h"
#include <mutex>
#include <condition_variable>
#include <vector>
#include <math.h>
#include <string>
#include <sys/types.h>
#include <chrono>
#include <fstream>
#include <inttypes.h>
#include <stdlib.h>
#include "checkfp.h"

using namespace std;
// UrDriver负责通信接口，轨迹插补发送和I/O操作
class UrDriver {
private:
	//string ip_addr_ = "10.66.74.247";//host ip addr
	//string ip_addr_ = "192.168.1.104";//host ip addr
	//string ip_addr_ = "192.168.52.12";//host ip addr  MacBook
	string ip_addr_ = "192.168.17.122";//host ip addr   PC
	//string ip_addr_ = "192.168.1.3";//host ip addr
	int sRT = 0;        // socket rt client

	vector<double> q_target;
	vector<double> qd_target;
	vector<double> qdd_target;
	vector<double> qcur;  // record qcur and updated when thread1:unpack
	vector<double> qvcur;
	vector<double> pcur;
	vector<double> pvcur;//TCP speed actual
	double timeSinceStart;//Time elapsed since the controller was started
	mutex vari_lock;  // lock in case clash when get vari and set vari same time

	vector<double> qbefore;//used in update_qp to record data before the current one
	vector<double> pbefore;
	vector<double> pvbefore;
	int startMoveFlag=0;//由于在上传脚本程序后，机械臂会经过若干秒才会运动，因此对于monitorFlag1=1刚置1的一段时间，机械臂一定是静止的，所以增加了类成员startMoveFlag，最初由于startMoveFlag=0，因此机械臂无论花多少时间去处理脚本程序，都不会判定为结束运动。
	int notChangeCount=0;
	int notChangeCount_Max = 200;//当notChangeCount超过notChangeCount_Max的时候就置monitorFlag=0

	const int MULT_JOINTSTATE_ = 1000000;
	const int MULT_TIME_ = 1000000;
	const int REVERSE_PORT_ = 50001;
	double servoj_time_ = 0.05;
	double lookAheadTime = 0.2;
	int sREclient = 0;//assign a client to connect with ur on 50001
	int sRE = 0;//socket reverse server
	bool executing_traj_ = 0;

	int monitorFlag = 0;//when monitorFlag==1,print info in update_qp
	int monitorFlag2 = 0;//when monitorFlag2==1,print info in update_qp and not use notChangeCount
	int monitorFlag3 = 0;//same as monitorFlag except put qvcur into monitorLog as well
	ofstream fpo;
	ofstream fpo_qv;//存放qv,q_target,qd_target,qdd_target信息的monitorLog_qv.txt，因为要避免对原代码的修改，因此增加一个文件
	int fpoAlreadyOpenFlag = 0;//when monitorFlag==1, if fpoAlreadyOpenFlag==0, then open the file and set fpoAlreadyOpenFlag
	int fpoAlreadyOpenFlag2 = 0;//when setMonitorFlag2(), reset fpoAlreadyOpenFlag2=0。when resetMonitorFlag2(),close fpo
public:
	UrDriver() {
		sRT = init_connect();  //开rt口
		if (sRT <= 0) {
			cout << "WARN at UrDriver:" << "failed to open RT" << endl;
			Sleep(2);
			exit(0);
		}

		sRE = init_listen();//监听reverse口
		if (sRE <= 0) {
			cout << "WARN at UrDriver:" << "failed to open RE" << endl;
			Sleep(2);
			exit(0);
		}
		/*init qcur and pcur*/
		q_target.resize(6);
		qd_target.resize(6);
		qdd_target.resize(6);
		qcur.resize(6);
		qvcur.resize(6);
		pcur.resize(6);
		pvcur.resize(6);
		qbefore.resize(6);
		pbefore.resize(6);
		pvbefore.resize(6);

	}

	~UrDriver() {
		//close(sRT);
		//close(sRE);
		//close(sREclient);
	}

	void setServojTime(double t) {
		servoj_time_ = t;
	}
	void setLookaheadTime(double t) {
		lookAheadTime = t;
	}
	void sendCmd(string cmd_str) {
		if (sRT <= 0) {
			cout << "connect rt first" << endl;
			return;
		}
		send(sRT, cmd_str.c_str(), strlen(cmd_str.c_str()), 0);
	}
	//进行对两点进行cubic interpolation，需要给定两点的位置，速度，以及两点的时间
	std::vector<double> interp_cubic(double t, double T,
		std::vector<double> p0_pos,
		std::vector<double> p1_pos,
		std::vector<double> p0_vel,
		std::vector<double> p1_vel) {
		/*Returns positions of the joints at time 't' */
		std::vector<double> positions;
		for (unsigned int i = 0; i < p0_pos.size(); i++) {
			double a = p0_pos[i];
			double b = p0_vel[i];
			double c =
				(-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i] - T * p1_vel[i]) /
				pow(T, 2);
			double d =
				(2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i] + T * p1_vel[i]) /
				pow(T, 3);
			positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
		}
		return positions;
	}
	void update_qp() {
		/*
		v3.6 1108
		v3.2 1060
		  */
		char buf[1060];  //数据传送的缓冲区

		memset(buf, 0, 1060);

		int bytes_read = recv(sRT, buf, 1060, 0);

		int ms;  // message size
		memcpy(&ms, &buf[0], sizeof(ms));
		ms = ntohl(ms);

		uint64_t q;

		vari_lock.lock();

		memcpy(&q, &buf[4], 8);//Read Time
		timeSinceStart = ntohd(q);

		memcpy(&q, &buf[12], 8);
		q_target[0] = (ntohd(q));
		memcpy(&q, &buf[20], 8);
		q_target[1] = (ntohd(q));
		memcpy(&q, &buf[28], 8);
		q_target[2] = (ntohd(q));
		memcpy(&q, &buf[36], 8);
		q_target[3] = (ntohd(q));
		memcpy(&q, &buf[44], 8);
		q_target[4] = (ntohd(q));
		memcpy(&q, &buf[52], 8);
		q_target[5] = (ntohd(q));

		memcpy(&q, &buf[60], 8);
		qd_target[0] = (ntohd(q));
		memcpy(&q, &buf[68], 8);
		qd_target[1] = (ntohd(q));
		memcpy(&q, &buf[76], 8);
		qd_target[2] = (ntohd(q));
		memcpy(&q, &buf[84], 8);
		qd_target[3] = (ntohd(q));
		memcpy(&q, &buf[92], 8);
		qd_target[4] = (ntohd(q));
		memcpy(&q, &buf[100], 8);
		qd_target[5] = (ntohd(q));

		memcpy(&q, &buf[108], 8);
		qdd_target[0] = (ntohd(q));
		memcpy(&q, &buf[116], 8);
		qdd_target[1] = (ntohd(q));
		memcpy(&q, &buf[124], 8);
		qdd_target[2] = (ntohd(q));
		memcpy(&q, &buf[132], 8);
		qdd_target[3] = (ntohd(q));
		memcpy(&q, &buf[140], 8);
		qdd_target[4] = (ntohd(q));
		memcpy(&q, &buf[148], 8);
		qdd_target[5] = (ntohd(q));

		memcpy(&q, &buf[252], 8);
		qcur[0] = (ntohd(q));
		memcpy(&q, &buf[260], 8);
		qcur[1] = (ntohd(q));
		memcpy(&q, &buf[268], 8);
		qcur[2] = (ntohd(q));
		memcpy(&q, &buf[276], 8);
		qcur[3] = (ntohd(q));
		memcpy(&q, &buf[284], 8);
		qcur[4] = (ntohd(q));
		memcpy(&q, &buf[292], 8);
		qcur[5] = (ntohd(q));

		memcpy(&q, &buf[300], 8);
		qvcur[0] = (ntohd(q));
		memcpy(&q, &buf[308], 8);
		qvcur[1] = (ntohd(q));
		memcpy(&q, &buf[316], 8);
		qvcur[2] = (ntohd(q));
		memcpy(&q, &buf[324], 8);
		qvcur[3] = (ntohd(q));
		memcpy(&q, &buf[332], 8);
		qvcur[4] = (ntohd(q));
		memcpy(&q, &buf[340], 8);
		qvcur[5] = (ntohd(q));

		memcpy(&q, &buf[444], 8);
		pcur[0] = ntohd(q) * m2mm;
		memcpy(&q, &buf[452], 8);
		pcur[1] = ntohd(q) * m2mm;
		memcpy(&q, &buf[460], 8);
		pcur[2] = ntohd(q) * m2mm;
		memcpy(&q, &buf[468], 8);
		pcur[3] = ntohd(q);
		memcpy(&q, &buf[476], 8);
		pcur[4] = ntohd(q);
		memcpy(&q, &buf[484], 8);
		pcur[5] = ntohd(q);

		memcpy(&q, &buf[492], 8);
		pvcur[0] = ntohd(q) * m2mm;
		memcpy(&q, &buf[500], 8);
		pvcur[1] = ntohd(q) * m2mm;
		memcpy(&q, &buf[508], 8);
		pvcur[2] = ntohd(q) * m2mm;
		memcpy(&q, &buf[516], 8);
		pvcur[3] = ntohd(q);
		memcpy(&q, &buf[524], 8);
		pvcur[4] = ntohd(q);
		memcpy(&q, &buf[532], 8);
		pvcur[5] = ntohd(q);

		if (monitorFlag == 1) {
			printf("________________________________________________________________________\n");
			printf("Time elapsed since started: %8.3f\n", timeSinceStart);
			printf("qcur :| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", qcur[0], qcur[1], qcur[2], qcur[3], qcur[4], qcur[5]);
			printf("qvcur :| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", qvcur[0], qvcur[1], qvcur[2], qvcur[3], qvcur[4], qvcur[5]);
			printf("pcur :| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", pcur[0], pcur[1], pcur[2], pcur[3], pcur[4], pcur[5]);
			printf("pvcur:| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", pvcur[0], pvcur[1], pvcur[2], pvcur[3], pvcur[4], pvcur[5]);

			printf("q_target:| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
			printf("qd_target:| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", qd_target[0], qd_target[1], qd_target[2], qd_target[3], qd_target[4], qd_target[5]);
			printf("qdd_target:| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", qdd_target[0], qdd_target[1], qdd_target[2], qdd_target[3], qdd_target[4], qdd_target[5]);
			printf("------------------------------------------------------------------------\n");
		
			//写文件
			if (fpoAlreadyOpenFlag == 0) {
				fpo.open("C:/Users/fx53v/Desktop/Do/monitorLog.txt");
				fpo_qv.open("C:/Users/fx53v/Desktop/Do/monitorLog_qv_Targets.txt");
				checkfp(fpo_qv);
				if (!fpo) {
					cout << "WARN at interplote:fail to open monitorLog" << endl;
					system("pause");
					exit(0);
				}
				fpoAlreadyOpenFlag = 1;
			}
			fpo << timeSinceStart << endl;
			for (int i = 0; i < 6; i++) {
				fpo << qcur[i] << " ";
			}
			fpo << endl;
			for (int i = 0; i < 6; i++) {
				fpo << pcur[i] << " ";
			}
			fpo << endl;
			for (int i = 0; i < 6; i++) {
				fpo << pvcur[i] << " ";
			}
			fpo << endl;
			
			for (int i = 0; i < 6; i++) {
				fpo_qv << qvcur[i] << " ";
			}
			fpo_qv << endl;
			for (int i = 0; i < 6; i++) {
				fpo_qv << q_target[i] << " ";
			}
			fpo_qv << endl;
			for (int i = 0; i < 6; i++) {
				fpo_qv << qd_target[i] << " ";
			}
			fpo_qv << endl;
			for (int i = 0; i < 6; i++) {
				fpo_qv << qdd_target[i] << " ";
			}
			fpo_qv << endl;
			//判断是否仍在运动
			//if (qbefore == qcur && pbefore == pcur && pvbefore == pvcur) {
			if (pvbefore == pvcur) {
				notChangeCount++;
			}
			else {
				if (startMoveFlag == 0) {
					notChangeCount = 0;
					startMoveFlag = 1;
				}
			}
			if (startMoveFlag == 1) {
				if (notChangeCount > notChangeCount_Max) {
					monitorFlag = 0;
					notChangeCount = 0;
					fpo.close();
					fpo_qv.close();
					fpoAlreadyOpenFlag = 0;
					startMoveFlag = 0;
				}
			}
		}
		/*
		setMonitorFlag2()：monitorFlag2=1;fpoAlreadyOpenFlag2=0（使update_qp中进入if语句块，并打开文件）
		resetMonitorFlag2()：monitorFlag2=0;fpo.close（关闭文件）
		*/
		if (monitorFlag2 == 1) {
			printf("________________________________________________________________________\n");
			printf("Time elapsed since started: %8.3f\n", timeSinceStart);
			printf("qcur :| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", qcur[0], qcur[1], qcur[2], qcur[3], qcur[4], qcur[5]);
			printf("pcur :| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", pcur[0], pcur[1], pcur[2], pcur[3], pcur[4], pcur[5]);
			printf("pvcur:| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", pvcur[0], pvcur[1], pvcur[2], pvcur[3], pvcur[4], pvcur[5]);
			printf("------------------------------------------------------------------------\n");
			//写文件
			if (fpoAlreadyOpenFlag2 == 0) {
				fpo.open("C:/Users/fx53v/Desktop/Do/monitorLog.txt");
				if (!fpo) {
					cout << "WARN at interplote:fail to open monitorLog" << endl;
					system("pause");
					exit(0);
				}
				fpoAlreadyOpenFlag2 = 1;
			}
			fpo << timeSinceStart << endl;
			for (int i = 0; i < 6; i++) {
				fpo << qcur[i] << " ";
			}
			fpo << endl;
			for (int i = 0; i < 6; i++) {
				fpo << pcur[i] << " ";
			}
			fpo << endl;
			for (int i = 0; i < 6; i++) {
				fpo << pvcur[i] << " ";
			}
			fpo << endl;
		}

		if (monitorFlag3 == 1) {
			printf("________________________________________________________________________\n");
			printf("Time elapsed since started: %8.3f\n", timeSinceStart);
			printf("qcur :| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", qcur[0], qcur[1], qcur[2], qcur[3], qcur[4], qcur[5]);
			printf("qcur :| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", qvcur[0], qvcur[1], qvcur[2], qvcur[3], qvcur[4], qvcur[5]);
			printf("pcur :| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", pcur[0], pcur[1], pcur[2], pcur[3], pcur[4], pcur[5]);
			printf("pvcur:| %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f |\n", pvcur[0], pvcur[1], pvcur[2], pvcur[3], pvcur[4], pvcur[5]);
			printf("------------------------------------------------------------------------\n");

			//写文件
			if (fpoAlreadyOpenFlag == 0) {
				fpo.open("C:/Users/fx53v/Desktop/Do/monitorLog.txt");
				if (!fpo) {
					cout << "WARN at interplote:fail to open monitorLog" << endl;
					system("pause");
					exit(0);
				}
				fpoAlreadyOpenFlag = 1;
			}
			fpo << timeSinceStart << endl;
			for (int i = 0; i < 6; i++) {
				fpo << qcur[i] << " ";
			}
			fpo << endl;
			for (int i = 0; i < 6; i++) {
				fpo << pcur[i] << " ";
			}
			fpo << endl;
			for (int i = 0; i < 6; i++) {
				fpo << qvcur[i] << " ";
			}
			fpo << endl;
			for (int i = 0; i < 6; i++) {
				fpo << pvcur[i] << " ";
			}
			fpo << endl;

			//判断是否仍在运动
			if (qbefore == qcur && pbefore == pcur && pvbefore == pvcur) {
				notChangeCount++;
			}
			else {
				if (startMoveFlag == 0) {
					notChangeCount = 0;
					startMoveFlag = 1;
				}
			}
			if (startMoveFlag == 1) {
				if (notChangeCount > notChangeCount_Max) {
					monitorFlag3 = 0;
					notChangeCount = 0;
					fpo.close();
					fpoAlreadyOpenFlag = 0;
					startMoveFlag = 0;
				}
			}
		}
		
		qbefore = qcur;
		pbefore = pcur;
		pvbefore = pvcur;

		vari_lock.unlock();
	}

	/*private functions*/
	private:
	//double ntohd(long long nf) {
	//	double x;
	//	nf = be64toh(nf);
	//	memcpy(&x, &nf, sizeof(x));
	//	return x;
	//}
	public:
		/*doTraj()先调用uploadProg()启动示教器上的servoj thread。然后进入循环：
		调用interp_cubic()来进行cubic interpolation，并将插值点传到示教器去。
		为保证oversample（sample多于执行以避免无命令可执行的情况），
		设置the loop Sleep time is a fourth of the servoj time.
*/
		bool doTraj(std::vector<double> inp_timestamps,
			std::vector<std::vector<double> > inp_positions,
			std::vector<std::vector<double> > inp_velocities) {

			////use for windows to record the interpolate result
			//ofstream file1("C:/Users/XY/Desktop/path/IKresultTest.txt");
			//if (!file1) {
			//	cout << "open file error" << endl;
			//}
			int flag = 1;//test

			std::chrono::high_resolution_clock::time_point t0, t;//t0:start time. t1:current time	
			std::vector<double> positions;//take result of interp_cubic and use in servoj(positions)
			unsigned int j;//current point's index in traj

			if (!UrDriver::uploadProg()) {
				return false;
			}
			executing_traj_ = true;
			t0 = std::chrono::high_resolution_clock::now();
			t = t0;
			j = 0;

			while ((inp_timestamps[inp_timestamps.size() - 1] >=//timestamp应该是相对时间--以0为开始时间的
				std::chrono::duration_cast<std::chrono::duration<double>>(t - t0)
				.count()) and
				executing_traj_) {
				//因为我们是有插补点的，所以不是每次循环都只要下一个timestamp点就行了。因此需确认当前轮到哪个点了
				while (inp_timestamps[j] <=
					std::chrono::duration_cast<std::chrono::duration<double>>(t - t0)
					.count() &&
					j < inp_timestamps.size() - 1) {
					j += 1;
				}
				//interp_cubic的插补的点到达时间和timestamp一样---从0开始
				positions = UrDriver::interp_cubic(
					std::chrono::duration_cast<std::chrono::duration<double>>(
						t - t0).count() - inp_timestamps[j - 1],
					inp_timestamps[j] - inp_timestamps[j - 1],
					inp_positions[j - 1], inp_positions[j], inp_velocities[j - 1], inp_velocities[j]);
				//发送servoj指令
				cout << std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count() << ":" <<
					r2d(positions[0]) << "," << r2d(positions[1]) << "," << r2d(positions[2]) << "," << r2d(positions[3]) << "," << r2d(positions[4]) << "," << r2d(positions[5]) << endl;
				//file1 << (positions[0]) << " " << (positions[1]) << " " << (positions[2]) << " " << (positions[3]) << " " << (positions[4]) << " " << (positions[5]) << endl;


				if (positions[0] > 0) {//test
					flag = 1;
				}
				if (flag) {//test
					UrDriver::servoj(positions, 1);
				}

				//servoj的dt默认是0.008,那么我们就等0.002然后就开始下一次插补与发送servoj
				// oversample with 4 * sample_time
				std::this_thread::sleep_for(
					std::chrono::milliseconds((int)((servoj_time_ * 1000) / 4.)));
				t = std::chrono::high_resolution_clock::now();
			}
			executing_traj_ = false;
			//file1 << endl;
			//file1.close();
			return true;
		}
	/*uploadProg()会向示教器发送servoj thread脚本并启动。
	该脚本会启动一client/server对PC传来的数据unpack并产生产生servoj指令让ur运行。*/
	bool uploadProg() {
		std::string cmd_str;
		char buf[128];
		cmd_str = "def driverProg():\n";

		sprintf_s(buf, "\tMULT_jointstate = %i\n", MULT_JOINTSTATE_);
		cmd_str += buf;

		cmd_str += "\tSERVO_IDLE = 0\n";
		cmd_str += "\tSERVO_RUNNING = 1\n";
		cmd_str += "\tcmd_servo_state = SERVO_IDLE\n";
		cmd_str += "\tcmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";

		cmd_str += "\tdef set_servo_setpoint(q):\n";//def set_servo_setpoint
		cmd_str += "\t\tenter_critical\n";
		cmd_str += "\t\tcmd_servo_state = SERVO_RUNNING\n";
		cmd_str += "\t\tcmd_servo_q = q\n";
		cmd_str += "\t\texit_critical\n";
		cmd_str += "\tend\n";

		cmd_str += "\tthread servoThread():\n";//thread
		cmd_str += "\t\tstate = SERVO_IDLE\n";
		cmd_str += "\t\twhile True:\n";
		cmd_str += "\t\t\tenter_critical\n";
		cmd_str += "\t\t\tq = cmd_servo_q\n";
		cmd_str += "\t\t\tdo_brake = False\n";
		cmd_str += "\t\t\tif (state == SERVO_RUNNING) and ";
		cmd_str += "(cmd_servo_state == SERVO_IDLE):\n";
		cmd_str += "\t\t\t\tdo_brake = True\n";
		cmd_str += "\t\t\tend\n";
		cmd_str += "\t\t\tstate = cmd_servo_state\n";//state = cmd_servo_state
		cmd_str += "\t\t\tcmd_servo_state = SERVO_IDLE\n";
		cmd_str += "\t\t\texit_critical\n";
		cmd_str += "\t\t\tif do_brake:\n";
		cmd_str += "\t\t\t\tstopj(1.0)\n";
		cmd_str += "\t\t\t\tsync()\n";
		cmd_str += "\t\t\telif state == SERVO_RUNNING:\n";//elif state == SERVO_RUNNING

		sprintf_s(buf, "\t\t\t\tservoj(q, t=%.4f, lookahead_time=%f)\n", servoj_time_,lookAheadTime);//servoJ
		//sprintf(buf, "\t\t\t\tmovej(q,a=0.4,v=0.5)\n");//movej

		cmd_str += buf;

		cmd_str += "\t\t\telse:\n";
		cmd_str += "\t\t\t\tsync()\n";
		cmd_str += "\t\t\tend\n";
		cmd_str += "\t\tend\n";
		cmd_str += "\tend\n";

		sprintf_s(buf, "\tsocket_open(\"%s\", %i)\n", ip_addr_.c_str(), REVERSE_PORT_);//open socket
		cmd_str += buf;

		cmd_str += "\tthread_servo = run servoThread()\n";
		cmd_str += "\tkeepalive = 1\n";
		cmd_str += "\twhile keepalive > 0:\n";//main thread
		cmd_str += "\t\tparams_mult = socket_read_binary_integer(6+1)\n";//read from reverse port
		cmd_str += "\t\tif params_mult[0] > 0:\n";
		cmd_str += "\t\t\tq = [params_mult[1] / MULT_jointstate, ";
		cmd_str += "params_mult[2] / MULT_jointstate, ";
		cmd_str += "params_mult[3] / MULT_jointstate, ";
		cmd_str += "params_mult[4] / MULT_jointstate, ";
		cmd_str += "params_mult[5] / MULT_jointstate, ";
		cmd_str += "params_mult[6] / MULT_jointstate]\n";
		cmd_str += "\t\t\tkeepalive = params_mult[7]\n";//第7个参数是拿来判断是否继续发送的
		cmd_str += "\t\t\tset_servo_setpoint(q)\n";
		cmd_str += "\t\tend\n";
		cmd_str += "\tend\n";
		cmd_str += "\tsleep(.1)\n";
		cmd_str += "\tsocket_close()\n";
		cmd_str += "\tkill thread_servo\n";
		cmd_str += "end\n";

		cout << "send program to UR3 through RT port" << endl;
		int bytes_written = send(sRT, cmd_str.c_str(), cmd_str.length(), 0);

		return UrDriver::openServo();
	}
	/*UrDriver开了server用来接下来与ur进行reverse口的连接
	openServo即在upload prog给ur后，ur开启了client并进行连接请求的时候我们assign一个client来accept
	从而创建连接*/
	bool openServo() {
		struct sockaddr_in cli_addr;
		int clilen;
		clilen = sizeof(cli_addr);
		cout << "waiting for connect request on RE PORT from UR3" << endl;
		sREclient = accept(sRE, (struct sockaddr*) & cli_addr, &clilen);
		if (sREclient < 0) {
			cout << "ERROR on accepting reverse communication" << endl;
			return false;
		}
		cout << "connect on reverse port" << endl;
		return true;
	}



	/*servoj接收各关节目标位置并打包到buf。
	同时其有keep alive flag形参来确认是否应该继续让ur进行servoj运动*/
	void servoj(std::vector<double> positions, int keepalive) {
		unsigned int bytes_written;
		int tmp;
		char buf[28];
		for (int i = 0; i < 6; i++) {
			tmp = htonl((int)(positions[i] * MULT_JOINTSTATE_));
			buf[i * 4] = tmp & 0xff;
			buf[i * 4 + 1] = (tmp >> 8) & 0xff;
			buf[i * 4 + 2] = (tmp >> 16) & 0xff;
			buf[i * 4 + 3] = (tmp >> 24) & 0xff;
		}
		tmp = htonl((int)keepalive);
		buf[6 * 4] = tmp & 0xff;
		buf[6 * 4 + 1] = (tmp >> 8) & 0xff;
		buf[6 * 4 + 2] = (tmp >> 16) & 0xff;
		buf[6 * 4 + 3] = (tmp >> 24) & 0xff;
		bytes_written = send(sREclient, buf, 28, 0);
	}
	public:
	//monitorFlag = 1;
	void setMonitorFlag() {
		monitorFlag = 1;
	}
	int getMonitorFlag() {
		return monitorFlag;
	}
	//monitorFlag3 = 1
	void setMonitorFlag3() {
		monitorFlag3 = 1;
	}
	int getMonitorFlag3() {
		return monitorFlag3;
	}
	//输入机械臂保持多少时间静止，则认为已结束运动。t的单位为秒
	void setNotChangeCount_Max(double t) {
		notChangeCount_Max = t / 0.008;
	}
	int getNotChangeCount_Max() {
		return notChangeCount_Max;
	}
	/*
	setMonitorFlag2()：monitorFlag2=1;fpoAlreadyOpenFlag2=0（使update_qp中进入if语句块，并打开文件）
	resetMonitorFlag2()：monitorFlag2=0;fpo.close（关闭文件）
	*/
	void setMonitorFlag2() {
		monitorFlag2 = 1;
		fpoAlreadyOpenFlag2 = 0;
	}
	void resetMonitorFlag2() {
		monitorFlag2 = 0;
		fpo.close();
	}
	vector<double> getp() {
		// vari_lock.lock();
		return pcur;
		// vari_lock.unlock();
	}
	vector<double> getq() {
		// vari_lock.lock();
		return qcur;
		// vari_lock.unlock();
	}
	vector<double> getpv() {
		return pvcur;
	}
	double getTime() {
		return timeSinceStart;
	}
};



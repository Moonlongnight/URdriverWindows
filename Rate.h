#pragma once
#include <chrono>
#include<thread>
#include <iostream>
#include"trans.h"
using namespace std;
namespace ros {
	class Rate {
	private:
	public:
		Rate(const double& frequency) {

		}
		~Rate() {};

		void sleep() {

		}
		//Óï·¨Ê¾Àý
		void example() {
			struct timespec req = { 0 };
			struct timespec rem = { 0 };
			time_t sec = 0;
			req.tv_sec = sec;
			req.tv_nsec = 8 * 1000000;

			chrono::high_resolution_clock::time_point t0, t;
			for (int i = 0; i < 100; i++) {
				t0 = chrono::high_resolution_clock::now();
				std::this_thread::sleep_for(std::chrono::milliseconds((int)((0.008 * m2mm))));
				t = chrono::high_resolution_clock::now();
				cout << chrono::duration_cast<std::chrono::duration<double>>(t - t0).count() << endl;
			}
		}
	};




};
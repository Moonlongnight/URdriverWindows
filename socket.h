#pragma once
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WINSOCK2.H>   
#include <stdio.h>
#include <string>
#include<iostream>
#include<sstream>
//���������ʹ�õĳ���      
#define SERVER_ADDRESS "192.168.17.129" //��������IP��ַ   PC
#define PORT           30003        //�������Ķ˿ں�
#define REVERSE_PORT   50001
#define MSGSIZE        1024         //�շ��������Ĵ�С      
#pragma comment(lib, "ws2_32.lib")

using namespace std;

SOCKET init_connect() {
	/*sClient*/
	SOCKET sClient;
	sClient = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);//ipv4  TCP 

	/*server info*/
	SOCKADDR_IN server;//store ip and port of server																										    
	memset(&server, 0, sizeof(SOCKADDR_IN)); //set server as 0 

	server.sin_family = PF_INET;
	server.sin_port = htons(PORT);
	server.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);

	/*connect*/
	cout << "connecting..." << endl;
	connect(sClient, (struct sockaddr*) & server, sizeof(SOCKADDR_IN));
	cout << "connected" << endl;
	return sClient;
}

SOCKET init_listen() {
	/*sListener*/
	SOCKET sListener;
	sListener = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);//ipv4  TCP 
	if (sListener == -1) {
		std::cout << "sListener assign error" << std::endl;
		return -1;
	}
	/*bind listerner with server info*/
	SOCKADDR_IN server;//store ip and port of server																										    
	memset(&server, 0, sizeof(SOCKADDR_IN)); //set server as 0 

	server.sin_family = PF_INET;
	server.sin_port = htons(REVERSE_PORT);
	/*INADDR_ANY��
	PORT=30003 ����1.IP
	PORT=30003 ����2.IP
	PORT=30003 ����2.IP
	����Ϣ���ᴫ����server����*/
	server.sin_addr.s_addr = htonl(INADDR_ANY);
	//server.sin_addr.s_addr = htonl((ULONG)0xc0a80103);
	//server.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);

	if (bind(sListener, (sockaddr*)&server, sizeof(server)) == -1) {
		std::cout << "bind error" << std::endl;
		return -1;
	}
	/*start listen*/
	if (listen(sListener, 5) == -1) {
		std::cout << "listen error" << std::endl;
		return -1;
	}

	printf("listener successfully bind to��%s \r\n", inet_ntoa(server.sin_addr));

	return sListener;
}
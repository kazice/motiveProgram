// client.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include<winsock2.h>
#include<stdio.h>
#pragma comment(lib,"ws2_32.lib")
#include <windows.h>
#include <conio.h>
void main()
{
	int temp32 = 0;
	char message[20]={0};
	WSADATA wsaData;
	SOCKET sockClient;//客户端Socket
	SOCKADDR_IN addrServer;//服务端地址
	WSAStartup(MAKEWORD(2,2),&wsaData);
	//新建客户端socket
	sockClient=socket(AF_INET,SOCK_STREAM,0);
	//定义要连接的服务端地址
	addrServer.sin_addr.S_un.S_addr=inet_addr("192.168.137.35");//目标IP(127.0.0.1是回送地址)
	addrServer.sin_family=AF_INET;
	addrServer.sin_port=htons(6000);//连接端口6000
	//连接到服务端
	connect(sockClient,(SOCKADDR*)&addrServer,sizeof(SOCKADDR));
	//发送数据
	while(1)
	{
		temp32 = getch();
		if(temp32 == 'q')break;
		printf("%d",temp32);
		sprintf(message, "%d", temp32);
		send(sockClient,message,strlen(message)+1,0);
	}
	//关闭socket
	closesocket(sockClient);
	WSACleanup();
	printf("关闭客户端");
}


// server.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include<winsock2.h>
#include<stdio.h>
#pragma comment(lib,"ws2_32.lib")

void main()
{
	int temp32;
	WSADATA wsaData;
	SOCKET sockServer;
	SOCKADDR_IN addrServer;
	SOCKET sockClient;
	SOCKADDR_IN addrClient;
	WSAStartup(MAKEWORD(2,2),&wsaData);
	sockServer=socket(AF_INET,SOCK_STREAM,0);
	addrServer.sin_addr.S_un.S_addr=inet_addr("192.168.137.1");//INADDR_ANY表示任何IP
	addrServer.sin_family=AF_INET;
	addrServer.sin_port=htons(6000);//绑定端口6000
	bind(sockServer,(SOCKADDR*)&addrServer,sizeof(SOCKADDR));
 
	//Listen监听端
	listen(sockServer,5);//5为等待连接数目
	printf("服务器已启动:\n监听中...\n");
	int len=sizeof(SOCKADDR);
	char sendBuf[100];//发送至客户端的字符串
	char recvBuf[100];//接受客户端返回的字符串
	//会阻塞进程，直到有客户端连接上来为止
	sockClient=accept(sockServer,(SOCKADDR*)&addrClient,&len);
	printf("客户端已连接\n");
	while(1)
	{
		//接收并打印客户端数据
		int Recv = recv(sockClient,recvBuf,100,0);
		printf("%d\n",Recv);
		if(Recv > 0)
		{
			temp32 = atoi(recvBuf);
			if(temp32 == 'a')
			{
				printf("OK");
			}
			printf("%s\n",recvBuf);
		}
		if(Recv == 0)
		{
			printf("客户端关闭\n");
			printf("等待客户端重新连接\n");
			sockClient=accept(sockServer,(SOCKADDR*)&addrClient,&len);
			int Recv1 = recv(sockClient,recvBuf,100,0);
			if(Recv1 > 0)
			{
				printf("客户端已重连\n");
				printf("%s\n",recvBuf);
			}
		}
	}
 
	//关闭socket
	closesocket(sockClient);
	WSACleanup();
}

// server.cpp : �������̨Ӧ�ó������ڵ㡣
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
	addrServer.sin_addr.S_un.S_addr=inet_addr("192.168.137.1");//INADDR_ANY��ʾ�κ�IP
	addrServer.sin_family=AF_INET;
	addrServer.sin_port=htons(6000);//�󶨶˿�6000
	bind(sockServer,(SOCKADDR*)&addrServer,sizeof(SOCKADDR));
 
	//Listen������
	listen(sockServer,5);//5Ϊ�ȴ�������Ŀ
	printf("������������:\n������...\n");
	int len=sizeof(SOCKADDR);
	char sendBuf[100];//�������ͻ��˵��ַ���
	char recvBuf[100];//���ܿͻ��˷��ص��ַ���
	//���������̣�ֱ���пͻ�����������Ϊֹ
	sockClient=accept(sockServer,(SOCKADDR*)&addrClient,&len);
	printf("�ͻ���������\n");
	while(1)
	{
		//���ղ���ӡ�ͻ�������
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
			printf("�ͻ��˹ر�\n");
			printf("�ȴ��ͻ�����������\n");
			sockClient=accept(sockServer,(SOCKADDR*)&addrClient,&len);
			int Recv1 = recv(sockClient,recvBuf,100,0);
			if(Recv1 > 0)
			{
				printf("�ͻ���������\n");
				printf("%s\n",recvBuf);
			}
		}
	}
 
	//�ر�socket
	closesocket(sockClient);
	WSACleanup();
}

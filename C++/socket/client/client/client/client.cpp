// client.cpp : �������̨Ӧ�ó������ڵ㡣
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
	SOCKET sockClient;//�ͻ���Socket
	SOCKADDR_IN addrServer;//����˵�ַ
	WSAStartup(MAKEWORD(2,2),&wsaData);
	//�½��ͻ���socket
	sockClient=socket(AF_INET,SOCK_STREAM,0);
	//����Ҫ���ӵķ���˵�ַ
	addrServer.sin_addr.S_un.S_addr=inet_addr("192.168.137.35");//Ŀ��IP(127.0.0.1�ǻ��͵�ַ)
	addrServer.sin_family=AF_INET;
	addrServer.sin_port=htons(6000);//���Ӷ˿�6000
	//���ӵ������
	connect(sockClient,(SOCKADDR*)&addrServer,sizeof(SOCKADDR));
	//��������
	while(1)
	{
		temp32 = getch();
		if(temp32 == 'q')break;
		printf("%d",temp32);
		sprintf(message, "%d", temp32);
		send(sockClient,message,strlen(message)+1,0);
	}
	//�ر�socket
	closesocket(sockClient);
	WSACleanup();
	printf("�رտͻ���");
}


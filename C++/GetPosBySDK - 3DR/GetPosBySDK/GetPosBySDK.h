#pragma once

#include <fstream>


#define FILTER_DATA_MAX 128
#define N_FILTER_X 2
#define N_FILTER_Y 2
#define N_FILTER_Z 10
#define OFSET_CENTER 0 //0.07569f


#define PENDULUM_OFFSET_X 0
#define PENDULUM_OFFSET_Y 0
#define PENDULUM_OFFSET_Z 0

#define PENDULUM_L 1.000f
#define pi 3.1415926f

typedef struct
{
	bool First;
	int N;
	int Num;
	float Output;
	float Sum;
	float Velt[FILTER_DATA_MAX];
}VELT_FILTER;

void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages
void resetClient();
int CreateClient(int iConnectionType);
void Display_Main_Menu(void);
void SendVeltZero2UAV(void);											//Send the Velt 0 to UAV
void IsPosloopStart(void);
DWORD WINAPI NetworkThread(LPVOID pM);
float MovingAvgFilter(VELT_FILTER *pVeltFilter,float VeltTemp);

extern unsigned int MyServersDataPort;
extern unsigned int MyServersCommandPort;


extern char szMyIPAddress[128];
extern char szServerIPAddress[128];
extern int analogSamplesPerMocapFrame;


//init of socket of Manifold SDK
bool CreateServerforSDK(void);
extern int temp32;
extern WSADATA wsaData;
extern SOCKET sockServer;
extern SOCKADDR_IN addrServer;
extern SOCKET sockClient;
extern SOCKADDR_IN addrClient;


union Send
{
	char chsendBuf[256];
	float flsendBuf[64];
};

typedef struct
{
	float timestart[256];
	float timeend[256];
	float timedelay[256];
}NET_TIME;

typedef struct
{
	unsigned char Head;
	unsigned char Len;
	unsigned char Seq;
	unsigned char SysID;
	unsigned char CompID;
	unsigned char MsgID;
	unsigned char CheckA;
	unsigned char CheckB;
	Send Data;
	unsigned char End;
}NETLINK_DATA;

extern NETLINK_DATA NetLinkData;
extern NETLINK_DATA NetLinkCmd;

extern char chSendBuf[265];
extern char chSendCmdBuf[265];

union SendVelt
{
	char chsendVeltBuf[20];
	float flsendVeltBuf[5];
};
//extern SendVelt SendVeltData;

extern char recvBuf[100];

//init of PID
extern REAL_POS_UAV RealPosUAV;
extern REAL_POS_UAV RealPosUAVPre;
extern DES_POS_UAV DesPosUAV;
extern DES_VELT_UAV DesVeltUAV;

//init of GetTime
extern LARGE_INTEGER Freq;
extern LARGE_INTEGER BeginTime;
extern LARGE_INTEGER EndTime;

//init of GetSpeed
typedef struct
{
	float flvx; 
	float flvy; 
	float flvz;   
}REAL_VELT_UAV;
extern REAL_VELT_UAV RealVeltUAV;
extern std::ofstream outVelt;
extern std::ofstream outPos;
extern std::ofstream outAttitude;

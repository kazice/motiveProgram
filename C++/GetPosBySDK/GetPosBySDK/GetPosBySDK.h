#pragma once

#include <fstream>


#define PENDULUM_L 1.0
#define pi 3.1415926
#define RollLimit 3
#define PitchLimit 3

#define XlimitMin -0.3
#define XlimitMax  0.3
#define YlimitMin -0.3
#define YlimitMax 0.3
#define ZlimitMin -0.2

void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages
void resetClient();
int CreateClient(int iConnectionType);
void Display_Main_Menu(void);
void SendVeltZero2UAV(void);											//Send the Velt 0 to UAV
void IsPosloopStart(void);

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

//init of socket of MATLAB
bool CreateServerforMATLAB(void);
extern WSADATA wsaDataM;
extern SOCKET sockServerM;
extern SOCKADDR_IN addrServerM;
extern SOCKET sockClientM;
extern SOCKADDR_IN addrClientM;

union Send
{
	char chsendBuf[32];
	float flsendBuf[7];
};
extern Send SendData;

union SendVelt
{
	char chsendVeltBuf[20];
	float flsendVeltBuf[5];
};
//extern SendVelt SendVeltData;

union SendCmd
{
	char chsendCmdBuf[32];
	float flsendCmdBuf[7];
};
extern SendCmd SendCmdData;

extern char recvBuf[100];

//init of PID
extern PID_CRL PidCrl_z;
extern PID_CRL PidCrl_x;
extern REAL_POS_UAV RealPosUAV;
extern REAL_POS_UAV RealPosUAVPre;
extern DES_POS_UAV DesPosUAV;
extern DES_VELT_UAV DesVeltUAV;
extern EN_RUN_STATE enRunStateX;
extern EN_RUN_STATE enRunStateY;
extern EN_RUN_STATE enRunStateZ;

extern EN_CONTROL_STATE enControlStateX;
extern EN_CONTROL_STATE enControlStateY;
extern EN_CONTROL_STATE enControlStateZ;

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

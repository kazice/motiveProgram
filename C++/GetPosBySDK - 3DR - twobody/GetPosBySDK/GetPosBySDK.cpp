// GetPosBySDK.cpp : Defines the entry point for the console application.
//=============================================================================
// Copyright © 2014 NaturalPoint, Inc. All Rights Reserved.
// 
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================
#include "StdAfx.h"
#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>
#include <windows.h>
#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "PIDController.h"
#include "Pendulum.h"
#include "GetPosBySDK.h"
#include <math.h>
#include <time.h>
#include "CMD.h"


#pragma warning( disable : 4996 )
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server

#define NETTEST 0
#define DEBUG 0

NatNetClient* theClient;
int iConnectionType = ConnectionType_Multicast;
//int iConnectionType = ConnectionType_Unicast;

unsigned int MyServersDataPort = 3130;
unsigned int MyServersCommandPort = 3131;


char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";
int analogSamplesPerMocapFrame = 0;


//init of socket of Manifold SDK
int temp32;
WSADATA wsaData;
SOCKET sockServer;
SOCKADDR_IN addrServer;
SOCKET sockClient;
SOCKADDR_IN addrClient;


NETLINK_DATA NetLinkData;
char chSendBuf[265];
NETLINK_DATA NetLinkCmd;
char chSendCmdBuf[265];
NETLINK_DATA NetLinkDelay;
char chSendDelayBuf[265];
float delaytime = 0;
bool recvflag;

char recvBuf[100];

NET_TIME net_delay = {0};

//init of PID
REAL_POS_UAV RealPosUAV = {0};
REAL_POS_UAV RealPosUAVPre = {0};

REAL_POS_UAV RealPosCenter = {0}; //摆杆支点位置

DES_POS_UAV DesPosUAV;
DES_VELT_UAV DesVeltUAV;

ATT_UAV AttUAV = {0};
ATT_UAV AttUAV_OPTI = {0};

bool bPendulumValid;

//init of GetTime
LARGE_INTEGER Freq;
LARGE_INTEGER BeginTime;
LARGE_INTEGER EndTime;

//init of GetTime
LARGE_INTEGER DelayFreq;
LARGE_INTEGER DelayTime;

//init of GetSpeed
REAL_VELT_UAV RealVeltUAV;
REAL_VELT_UAV FilterVeltUAV;
std::ofstream outVelt("Velt.txt");
std::ofstream outPos("Pos.txt");
std::ofstream outAttitude("Attitude.txt");

/****************velt_filter*********************/
VELT_FILTER VeltFilterX = {1,N_FILTER_X,0,0,0,{0}};
VELT_FILTER VeltFilterY = {1,N_FILTER_Y,0,0,0,{0}};
VELT_FILTER VeltFilterZ = {1,N_FILTER_Z,0,0,0,{0}};


/*************netdelay***************/

typedef enum
{
	NET_LISTENING,
	NET_GETHEAD,
	NET_GETPACKSTART,
	NET_GETDATA,
	NET_GETEND,
	NET_ERROR,
	NET_RETRY
}EN_NETLINK_STATE;

EN_NETLINK_STATE enNetLinkState;
NETLINK_DATA net_link_data;
char RecvBuf[256];


/*****************************************
*main
*
*****************************************/

int _tmain(int argc, _TCHAR* argv[])
{
    int iResult;

	//Head and End of Cmd Message
	NetLinkData.Seq = 0;
	NetLinkData.SysID = 0;
	NetLinkData.CompID = 0;

	NetLinkCmd.Seq = 0;
	NetLinkCmd.SysID = 0;
	NetLinkCmd.CompID = 0;

	if(!DEBUG)
	CreateServerforSDK();

	enNetLinkState = NET_LISTENING;
	QueryPerformanceFrequency(&DelayFreq); 
	HANDLE handle = CreateThread(NULL, 0, NetworkThread, NULL, 0, NULL);

    strcpy(szServerIPAddress, "192.168.2.251");		// not specified - assume server is local machine
	printf("Connecting to server at %s...\n", szServerIPAddress);
	strcpy(szMyIPAddress,"192.168.2.251");
	printf("Connecting from %s...\n", szMyIPAddress);

    // Create NatNet Client
    iResult = CreateClient(iConnectionType);
    if(iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }


	// send/receive test request
	printf("[SampleClient] Sending Test Request\n");
	void* response;
	int nBytes;
	iResult = theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	//Retrieve Data Descriptions from server
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = theClient->GetDataDescriptions(&pDataDefs);
	if(!pDataDefs)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.");
	}
	else
	{
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("MarkerSet Name : %s\n", pMS->szName);
                for(int i=0; i < pMS->nMarkers; i++)
                    printf("%s\n", pMS->szMarkerNames[i]);

            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("RigidBody Name : %s\n", pRB->szName);
                printf("RigidBody ID : %d\n", pRB->ID);
                printf("RigidBody Parent ID : %d\n", pRB->parentID);
                printf("Parent Offset : %3.6f,%3.6f,%3.6f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
            {
                // Skeleton
                sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                printf("Skeleton Name : %s\n", pSK->szName);
                printf("Skeleton ID : %d\n", pSK->skeletonID);
                printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
                for(int j=0; j < pSK->nRigidBodies; j++)
                {
                    sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
                    printf("  RigidBody Name : %s\n", pRB->szName);
                    printf("  RigidBody ID : %d\n", pRB->ID);
                    printf("  RigidBody Parent ID : %d\n", pRB->parentID);
                    printf("  Parent Offset : %3.6f,%3.6f,%3.6f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
                }
            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
            {
                // Force Plate
                sForcePlateDescription* pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                printf("Force Plate ID : %d\n", pFP->ID);
                printf("Force Plate Serial : %s\n", pFP->strSerialNo);
                printf("Force Plate Width : %3.6f\n", pFP->fWidth);
                printf("Force Plate Length : %3.6f\n", pFP->fLength);
                printf("Force Plate Electrical Center Offset (%3.6f, %3.6f, %3.6f)\n", pFP->fOriginX,pFP->fOriginY, pFP->fOriginZ);
                for(int iCorner=0; iCorner<4; iCorner++)
                    printf("Force Plate Corner %d : (%3.6f, %3.6f, %3.6f)\n", iCorner, pFP->fCorners[iCorner][0],pFP->fCorners[iCorner][1],pFP->fCorners[iCorner][2]);
                printf("Force Plate Type : %d\n", pFP->iPlateType);
                printf("Force Plate Data Type : %d\n", pFP->iChannelDataType);
                printf("Force Plate Channel Count : %d\n", pFP->nChannels);
                for(int iChannel=0; iChannel<pFP->nChannels; iChannel++)
                    printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
            }
            else
            {
                printf("Unknown data type.");
                // Unknown
            }
        }      
	}


	// Ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");

	Display_Main_Menu();
	while(c =_getch())
	{
		CMD_deal(c);		
		Display_Main_Menu();
		if(bExit)
			break;
	}

	// Done - clean up.
	theClient->Uninitialize();
	return ErrorCode_OK;
}

bool CreateServerforSDK(void)
{
	WSAStartup(MAKEWORD(2,2),&wsaData);
	sockServer=socket(AF_INET,SOCK_STREAM,0);
	addrServer.sin_addr.S_un.S_addr=inet_addr("192.168.2.251");
	addrServer.sin_family=AF_INET;
	addrServer.sin_port=htons(6000);
	bind(sockServer,(SOCKADDR*)&addrServer,sizeof(SOCKADDR));
 
	listen(sockServer,5);
	printf("Server had start:\nListening...6000\n");
	int len=sizeof(SOCKADDR);
	
	sockClient=accept(sockServer,(SOCKADDR*)&addrClient,&len);
	printf("Client of Manifold had connect\n");

	return 1;
}

// Establish a NatNet Client connection
int CreateClient(int iConnectionType)
{
    // release previous server
    if(theClient)
    {
        theClient->Uninitialize();
        delete theClient;
    }

    // create NatNet client
    theClient = new NatNetClient(iConnectionType);



    // set the callback handlers
    theClient->SetVerbosityLevel(Verbosity_Warning);
    theClient->SetMessageCallback(MessageHandler);
    theClient->SetDataCallback( DataHandler, theClient );	// this function will receive data from the server
    // [optional] use old multicast group
    //theClient->SetMulticastAddress("224.0.0.1");

    // print version info
    unsigned char ver[4];
    theClient->NatNetVersion(ver);
    printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

    // Init Client and connect to NatNet server
    // to use NatNet default port assignments
    int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress);
    // to use a different port for commands and/or data:
    //int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // get # of analog samples per mocap frame of data
        void* pResult;
        int ret = 0;
        int nBytes = 0;
        ret = theClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d", analogSamplesPerMocapFrame);
        }

        // print server info
        sServerDescription ServerDescription;
        memset(&ServerDescription, 0, sizeof(ServerDescription));
        theClient->GetServerDescription(&ServerDescription);
        if(!ServerDescription.HostPresent)
        {
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
            ServerDescription.HostAppVersion[1],ServerDescription.HostAppVersion[2],ServerDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
            ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", szMyIPAddress);
        printf("Server IP:%s\n", szServerIPAddress);
        printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
    }

    return ErrorCode_OK;

}



// DataHandler receives data from the server
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	QueryPerformanceCounter(&EndTime);
	QueryPerformanceFrequency(&Freq);
	double DataTimeInt = (double)(EndTime.QuadPart -BeginTime.QuadPart)*1000/Freq.QuadPart;
	//printf("Time is:%lf\n",DataTimeInt);
	
	NatNetClient* pClient = (NatNetClient*) pUserData;
	int i=0;

    //printf("FrameID : %d\n", data->iFrame);
    //printf("Timestamp :  %3.2lf\n", data->fTimestamp);
    //printf("Latency :  %3.2lf\n", data->fLatency);

    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    //if(bIsRecording)
    //    printf("RECORDING\n");
    //if(bTrackedModelsChanged)
    //    printf("Models Changed.\n");
	
        
    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	int hour, minute, second, frame, subframe;
	bool bValid = pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
	// decode to friendly string
	char szTimecode[128] = "";
	pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
	//printf("Timecode : %s\n", szTimecode);

	// Rigid Bodies
	/*printf("\nRigid Bodies [Count=%d]\n", data->nRigidBodies);
	for(i=0; i < data->nRigidBodies; i++)
	{
		printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, data->RigidBodies[i].params & 0x01);
		printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		printf("\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\n",
			data->RigidBodies[i].x,
			data->RigidBodies[i].y,
			data->RigidBodies[i].z,
			data->RigidBodies[i].qx,
			data->RigidBodies[i].qy,
			data->RigidBodies[i].qz,
			data->RigidBodies[i].qw);

		printf("\tRigid body markers [Count=%d]\n", data->RigidBodies[i].nMarkers);
		for(int iMarker=0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)
		{
			printf("\t\t");
			if(data->RigidBodies[i].MarkerIDs)
				printf("MarkerID:%d", data->RigidBodies[i].MarkerIDs[iMarker]);
			if(data->RigidBodies[i].MarkerSizes)
				printf("\tMarkerSize:%3.6f", data->RigidBodies[i].MarkerSizes[iMarker]);
			if(data->RigidBodies[i].Markers)
				printf("\tMarkerPos:%3.6f,%3.6f,%3.6f\n" ,
					data->RigidBodies[i].Markers[iMarker][0],
					data->RigidBodies[i].Markers[iMarker][1],
					data->RigidBodies[i].Markers[iMarker][2]);
		}
	}*/

	// params
	// 0x01 : bool, rigid body was successfully tracked in this frame
	bool bTrackingValid = data->RigidBodies[0].params & 0x01;
	//无人机位置
	RealPosUAV.flx = data->RigidBodies[0].x;
	RealPosUAV.fly = data->RigidBodies[0].z;
	RealPosUAV.flz = data->RigidBodies[0].y;
	//printf("mav_x\tmav_y\tmav_z\n");
	//printf("%f\t%f\t%f\n\n",RealPosUAV.flx,RealPosUAV.fly,RealPosUAV.flz);

	//无人机姿态
	AttUAV.roll = atan2(2*(data->RigidBodies[0].qw*data->RigidBodies[0].qx+data->RigidBodies[0].qy*data->RigidBodies[0].qz),1-2*(data->RigidBodies[0].qx*data->RigidBodies[0].qx+data->RigidBodies[0].qy*data->RigidBodies[0].qy));
	AttUAV.yaw = -asin(2*(data->RigidBodies[0].qw*data->RigidBodies[0].qy-data->RigidBodies[0].qz*data->RigidBodies[0].qx));
	AttUAV.pitch = atan2(2*(data->RigidBodies[0].qw*data->RigidBodies[0].qz+data->RigidBodies[0].qx*data->RigidBodies[0].qy),1-2*(data->RigidBodies[0].qy*data->RigidBodies[0].qy+data->RigidBodies[0].qz*data->RigidBodies[0].qz));	
	//printf("pitch\tyaw\troll\n");
	//printf("%f\t%f\t%f\n\n",AttUAV.pitch/3.1415926*180,AttUAV.yaw/3.1415926*180,AttUAV.roll/3.1415926*180);

	//视觉系统坐标系下支点位置
	RealPosCenter.flx = data->RigidBodies[1].x;
	RealPosCenter.fly = data->RigidBodies[1].y;
	RealPosCenter.flz = data->RigidBodies[1].z;
	//printf("center_x\tcenter_y\tcenter_z\n");
	//printf("%f\t%f\t%f\n\n",RealPosCenter.flx,RealPosCenter.fly,RealPosCenter.flz);
	//printf("%f\t%f\t%f\n",data->OtherMarkers[0][0],data->OtherMarkers[0][1],data->OtherMarkers[0][2]);

	// Other Markers
	//printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
	//判断离散点是否有摆杆
	for(int ii=0; ii < data->nOtherMarkers; ii++)
	{
		//printf("Other Marker %d : %3.6f\t%3.6f\t%3.6f\n",
		//	ii,
		//	data->OtherMarkers[ii][0],
		//	data->OtherMarkers[ii][1],
		//	data->OtherMarkers[ii][2]);
		//判断是否识别到摆杆
		if(data->RigidBodies[1].params & 0x01) //支点位置是否识别
		{
			if(data->OtherMarkers[ii][1]-RealPosCenter.fly > 0.8 && data->OtherMarkers[ii][1]-RealPosCenter.fly < 1.3) //xyz三个方向位置是否在距离飞机指定范围内
			{
				if(data->OtherMarkers[ii][0] - RealPosCenter.flx > -0.18 && data->OtherMarkers[ii][0] - RealPosCenter.flx < 0.18)
				{
					if(data->OtherMarkers[ii][2] - RealPosCenter.flz > -0.18 && data->OtherMarkers[ii][2] - RealPosCenter.flz < 0.18)
					{
						PendulumState.Pos.flx = (data->OtherMarkers[ii][0] + PENDULUM_OFFSET_X - RealPosCenter.flx)/2;
						PendulumState.Pos.flz = (data->OtherMarkers[ii][1] + PENDULUM_OFFSET_Z - RealPosCenter.fly)/2;
						PendulumState.Pos.fly = (data->OtherMarkers[ii][2] + PENDULUM_OFFSET_Y - RealPosCenter.flz)/2;
						//printf("Pendulum: %3.6f\t%3.6f\t%3.6f\n",
						//	PendulumState.Pos.flx,
						//	PendulumState.Pos.fly,
						//	PendulumState.Pos.flz);
						bPendulumValid = 1;  //每一次发送数据后置零，用于判断是否追踪到摆杆
					}
				}
			}
		}
		else
		{
			//printf("lost marker\n");
		}
	}

	//计算摆杆姿态
	PendulumState.Attitude.flx = asin(PendulumState.Pos.flx / PENDULUM_L);
	PendulumState.Attitude.fly = asin(PendulumState.Pos.fly / PENDULUM_L);
	//printf("Attitude of Pendulum[x y]:\n%3.6f\t%3.6f\n",PendulumState.Attitude.flx * 180 / 3.1416,PendulumState.Attitude.fly * 180 / 3.1416);

	//是否显示四旋翼位置
	if(bDataDisplay.Pos & !bMenuFlag)
	{
		printf("[%3.6f\t%3.6f\t%3.6f]\n",
			RealPosUAV.flx,
			RealPosUAV.fly,
			RealPosUAV.flz);
	}

	//追踪到四旋翼之后计算四旋翼速度及滤波
	if(bTrackingValid == 1)
	{
		RealVeltUAV.flvx = (RealPosUAV.flx - RealPosUAVPre.flx) / (float)DataTimeInt * 1000;
		RealVeltUAV.flvy = (RealPosUAV.fly - RealPosUAVPre.fly) / (float)DataTimeInt * 1000;
		RealVeltUAV.flvz = (RealPosUAV.flz - RealPosUAVPre.flz) / (float)DataTimeInt * 1000;

		RealPosUAVPre.flx = RealPosUAV.flx;
		RealPosUAVPre.fly = RealPosUAV.fly;
		RealPosUAVPre.flz = RealPosUAV.flz;

		FilterVeltUAV.flvx = MovingAvgFilter(&VeltFilterX,RealVeltUAV.flvx);
		FilterVeltUAV.flvy = MovingAvgFilter(&VeltFilterY,RealVeltUAV.flvy);
		FilterVeltUAV.flvz = MovingAvgFilter(&VeltFilterZ,RealVeltUAV.flvz);
	}

	//是否显示四旋翼速度
	if(bDataDisplay.Velt & !bMenuFlag)
	{
		printf("[%3.6f\t%3.6f\t%3.6f]\n",
			RealVeltUAV.flvx,
			RealVeltUAV.flvy,
			RealVeltUAV.flvz);
	}

	//发送反馈数据——速度反馈/位置反馈/摆杆反馈/网络延迟
	QueryPerformanceCounter(&DelayTime);
	NetLinkData.MsgID = 0x05; 
	NetLinkData.Len = 78; 
	NetLinkData.Data.chsendBuf[76] = bTrackingValid;
	NetLinkData.Data.chsendBuf[77] = bPendulumValid;

	NetLinkData.Data.flsendBuf[0] = RealPosUAV.flx;
	NetLinkData.Data.flsendBuf[1] = RealPosUAV.fly;
	NetLinkData.Data.flsendBuf[2] = RealPosUAV.flz;

	NetLinkData.Data.flsendBuf[3] = AttUAV.roll;
	NetLinkData.Data.flsendBuf[4] = AttUAV.yaw;
	NetLinkData.Data.flsendBuf[5] = AttUAV.pitch;
		
	NetLinkData.Data.flsendBuf[6] = RealVeltUAV.flvx;
	NetLinkData.Data.flsendBuf[7] = RealVeltUAV.flvy;
	NetLinkData.Data.flsendBuf[8] = RealVeltUAV.flvz;

	NetLinkData.Data.flsendBuf[9] = PendulumState.Pos.flx;
	NetLinkData.Data.flsendBuf[10] = PendulumState.Pos.fly;
	NetLinkData.Data.flsendBuf[11] = PendulumState.Pos.flz;

	NetLinkData.Data.flsendBuf[12] = PendulumState.Attitude.flx;
	NetLinkData.Data.flsendBuf[13] = PendulumState.Attitude.fly;

	if(NetLinkData.Seq == 0)
	{
		NetLinkData.Data.flsendBuf[14] = net_delay.timedelay[251];
		net_delay.timedelay[251] = 60;
	}
	else if(NetLinkData.Seq == 1)
	{
		NetLinkData.Data.flsendBuf[14] = net_delay.timedelay[252];
		net_delay.timedelay[252] = 60;
	}
	else if(NetLinkData.Seq == 2)
	{
		NetLinkData.Data.flsendBuf[14] = net_delay.timedelay[253];
		net_delay.timedelay[253] = 60;
	}
	else if(NetLinkData.Seq == 3)
	{
		NetLinkData.Data.flsendBuf[14] = net_delay.timedelay[254];
		net_delay.timedelay[254] = 60;
	}
	else if(NetLinkData.Seq == 4)
	{
		NetLinkData.Data.flsendBuf[14] = net_delay.timedelay[255];
		net_delay.timedelay[255] = 60;
	}
	else
	{
		NetLinkData.Data.flsendBuf[14] = net_delay.timedelay[NetLinkData.Seq-5];
	}
	NetLinkData.Data.flsendBuf[15] = (float)DelayTime.QuadPart*1000/DelayFreq.QuadPart;
	net_delay.timestart[NetLinkData.Seq] = NetLinkData.Data.flsendBuf[15];
	net_delay.timedelay[NetLinkData.Seq-5] = 60;
	//printf("delay last is: %f\n",net_delay.timedelay[NetLinkData.Seq-1]);

	NetLinkData.Data.flsendBuf[16] = FilterVeltUAV.flvx;
	NetLinkData.Data.flsendBuf[17] = FilterVeltUAV.flvy;
	NetLinkData.Data.flsendBuf[18] = FilterVeltUAV.flvz;

	chSendBuf[0] = 0xFE;
	chSendBuf[1] = NetLinkData.Len;
	chSendBuf[2] = NetLinkData.Seq;
	chSendBuf[3] = NetLinkData.SysID;
	chSendBuf[4] = NetLinkData.CompID;
	chSendBuf[5] = NetLinkData.MsgID;
	for(int data_i=0;data_i<NetLinkData.Len;data_i++)
	{
		chSendBuf[data_i+6] = NetLinkData.Data.chsendBuf[data_i];
	}
	chSendBuf[6+NetLinkData.Len] = NetLinkData.CheckA;
	chSendBuf[7+NetLinkData.Len] = NetLinkData.CheckB;
	chSendBuf[8+NetLinkData.Len] = 0xAA;

	if(!NETTEST)send(sockClient,chSendBuf,NetLinkData.Len+9,0);

	NetLinkData.Seq++;
	bPendulumValid = 0;

	//保存数据
	//outPos<<data->iFrame<<'\t'<<RealPosUAV.flx<<'\t'<<RealPosUAV.fly<<'\t'<<RealPosUAV.flz<<std::endl;
	//outAttitude<<data->iFrame<<'\t'<<RealPosUAV.flx<<'\t'<<RealPosUAV.fly<<'\t'<<RealPosUAV.flz<<std::endl;

	//计算时间
	QueryPerformanceCounter(&BeginTime);
		
}

//

// MessageHandler receives NatNet error/debug messages
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}


void resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = theClient->Uninitialize();
	if(iSuccess != 0)
		printf("error un-initting Client\n");

	iSuccess = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	if(iSuccess != 0)
		printf("error re-initting Client\n");


}


void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [a] SDK Version Query         | [i] Input DesPos of Z           |\n");
	printf("| [b] Request Control(Unlock)   | [j] Input DesPos of X           |\n");
	printf("| [c] Release Control(Lock)     | [l] Input DesPos of Y           |\n");	
	printf("| [d] Takeoff                   | [s] Show/Hide Debug Data        |\n");	
	printf("| [e] Landing                   | [k] Show/Hide Main menu         |\n");
	printf("| [x] Input DesVelt of X        | [y] Input DesVelt of Y          |\n");
	printf("| [f] change PID of Pos Velt    | [g] change PID of Pendulum      |\n");
	printf("| [q] Quit the Server           |\n");
	printf("+-----------------------------------------------------------------+\n");
    printf("input a/b/c etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}

/********************************/

DWORD WINAPI NetworkThread(LPVOID pM)
{
	printf("start network delay test!\n");
	while(!NETTEST)
	{
		if(enNetLinkState == NET_LISTENING)
		{
			int Recv = recv(sockClient,RecvBuf,1,0);
			if(Recv > 0)
			{
	        	net_link_data.Head = RecvBuf[0];
	        	if(net_link_data.Head == 0xFE)
	        	{
	        		enNetLinkState = NET_GETHEAD;
	        	}
			}
			else if(Recv == 0)
			{
	        	enNetLinkState = NET_ERROR;
			}
		}
		else if(enNetLinkState == NET_GETHEAD)
		{
			int Recv = recv(sockClient,RecvBuf,5,0);
			if(Recv > 0)
			{
	        	net_link_data.Len = RecvBuf[0];
	        	net_link_data.Seq = RecvBuf[1];
	        	net_link_data.SysID = RecvBuf[2];
	        	net_link_data.CompID = RecvBuf[3];
	        	net_link_data.MsgID = RecvBuf[4];
	        	enNetLinkState = NET_GETPACKSTART;
			}
			else if(Recv == 0)
			{
	        	enNetLinkState = NET_ERROR;
			}
		}
		else if(enNetLinkState == NET_GETPACKSTART)
		{
			int Recv = recv(sockClient,RecvBuf,net_link_data.Len,0);
			if(Recv > 0)
			{
	        	for(int data_i=0;data_i<net_link_data.Len;data_i++)
	        	{
	        		net_link_data.Data.chsendBuf[data_i] = RecvBuf[data_i];
	        	}
	        	enNetLinkState = NET_GETDATA;
			}
			else if(Recv == 0)
			{
	        	enNetLinkState = NET_ERROR;
			}
		}
		else if(enNetLinkState == NET_GETDATA)
		{
			int Recv = recv(sockClient,RecvBuf,3,0);
			if(Recv > 0)
			{
	        	net_link_data.CheckA = RecvBuf[0];
	        	net_link_data.CheckB = RecvBuf[1];
	        	net_link_data.End = RecvBuf[2];
	        	if(net_link_data.End == 0xAA)
	        	{
	        		enNetLinkState = NET_GETEND;
	        	}
	        	else
	        	{
	        		enNetLinkState = NET_LISTENING;
	        	}
			}
			else if(Recv == 0)
			{
	        	enNetLinkState = NET_ERROR;
			}
		}
		else if(enNetLinkState == NET_GETEND)
		{
			if(net_link_data.MsgID == 0x07) //delay
			{
				QueryPerformanceCounter(&DelayTime);
				net_delay.timeend[net_link_data.Seq] = (float)DelayTime.QuadPart*1000/DelayFreq.QuadPart;
				net_delay.timedelay[net_link_data.Seq] = (net_delay.timeend[net_link_data.Seq] - net_delay.timestart[net_link_data.Seq])/2;

				//printf("start time get is:%f\n", net_link_data.Data.flsendBuf[0]);
				//printf("end time now is:%f\n", net_delay.timeend[net_link_data.Seq]);
				//printf("start time now is:%f\n", net_delay.timestart[net_link_data.Seq]);
				//printf("delay is:%f\n",net_delay.timedelay[net_link_data.Seq]);
			}
			else if(net_link_data.MsgID == 0x08)
			{
				printf("PID of Pos and Velt change done\n");
			}
			else if(net_link_data.MsgID == 0x09)
			{
				printf("PID of Pendulum change done\n");
			}
			enNetLinkState = NET_LISTENING;
		}
		else if(enNetLinkState == NET_ERROR)
		{
			printf("network error,check it!\n");
		}
	}
	while(NETTEST)
	{
		NetLinkDelay.MsgID = 0x06; 
		NetLinkDelay.Len = 50; 
		Sleep(10);
		QueryPerformanceCounter(&DelayTime);
		NetLinkDelay.Data.flsendBuf[0] = (float)DelayTime.QuadPart*1000/DelayFreq.QuadPart;
		//printf("start time is:%f\n",NetLinkDelay.Data.flsendBuf[0]);
		NetLinkDelay.Data.flsendBuf[1] = delaytime;

		chSendDelayBuf[0] = 0xFE;
		chSendDelayBuf[1] = NetLinkDelay.Len;
		chSendDelayBuf[2] = NetLinkDelay.Seq;
		chSendDelayBuf[3] = NetLinkDelay.SysID;
		chSendDelayBuf[4] = NetLinkDelay.CompID;
		chSendDelayBuf[5] = NetLinkDelay.MsgID;
		for(int data_i=0;data_i<NetLinkDelay.Len;data_i++)
		{
			chSendDelayBuf[data_i+6] = NetLinkDelay.Data.chsendBuf[data_i];
		}
		chSendDelayBuf[6+NetLinkDelay.Len] = NetLinkDelay.CheckA;
		chSendDelayBuf[7+NetLinkDelay.Len] = NetLinkDelay.CheckB;
		chSendDelayBuf[8+NetLinkDelay.Len] = 0xAA;
		
		send(sockClient,chSendDelayBuf,NetLinkDelay.Len+9,0);
		recvflag = 1;
		while(recvflag)
		{
			if(enNetLinkState == NET_LISTENING)
			{
				int Recv = recv(sockClient,RecvBuf,1,0);
				if(Recv > 0)
				{
	        		net_link_data.Head = RecvBuf[0];
	        		if(net_link_data.Head == 0xFE)
	        		{
	        			enNetLinkState = NET_GETHEAD;
	        		}
				}
				else if(Recv == 0)
				{
	        		enNetLinkState = NET_ERROR;
				}
			}
			else if(enNetLinkState == NET_GETHEAD)
			{
				int Recv = recv(sockClient,RecvBuf,5,0);
				if(Recv > 0)
				{
	        		net_link_data.Len = RecvBuf[0];
	        		net_link_data.Seq = RecvBuf[1];
	        		net_link_data.SysID = RecvBuf[2];
	        		net_link_data.CompID = RecvBuf[3];
	        		net_link_data.MsgID = RecvBuf[4];
	        		enNetLinkState = NET_GETPACKSTART;
				}
				else if(Recv == 0)
				{
	        		enNetLinkState = NET_ERROR;
				}
			}
			else if(enNetLinkState == NET_GETPACKSTART)
			{
				int Recv = recv(sockClient,RecvBuf,net_link_data.Len,0);
				if(Recv > 0)
				{
	        		for(int data_i=0;data_i<net_link_data.Len;data_i++)
	        		{
	        			net_link_data.Data.chsendBuf[data_i] = RecvBuf[data_i];
	        		}
	        		enNetLinkState = NET_GETDATA;
				}
				else if(Recv == 0)
				{
	        		enNetLinkState = NET_ERROR;
				}
			}
			else if(enNetLinkState == NET_GETDATA)
			{
				int Recv = recv(sockClient,RecvBuf,3,0);
				if(Recv > 0)
				{
	        		net_link_data.CheckA = RecvBuf[0];
	        		net_link_data.CheckB = RecvBuf[1];
	        		net_link_data.End = RecvBuf[2];
	        		if(net_link_data.End == 0xAA)
	        		{
	        			enNetLinkState = NET_GETEND;
	        		}
	        		else
	        		{
	        			enNetLinkState = NET_LISTENING;
	        		}
				}
				else if(Recv == 0)
				{
	        		enNetLinkState = NET_ERROR;
				}
			}
			else if(enNetLinkState == NET_GETEND)
			{
				if(net_link_data.MsgID == 0x06) //delay test
				{
					QueryPerformanceCounter(&DelayTime);
					//printf("start time get is:%f\n", net_link_data.Data.flsendBuf[0]);
					delaytime = ((float)DelayTime.QuadPart*1000/DelayFreq.QuadPart - net_link_data.Data.flsendBuf[0])/2;
					//printf("end time is:%f\n", (float)DelayTime.QuadPart*1000/DelayFreq.QuadPart);
					printf("delay is:%f\n",delaytime);
				}
				enNetLinkState = NET_LISTENING;
				recvflag = 0;
			}
			else if(enNetLinkState == NET_ERROR)
			{
				printf("network error,check it!\n");
			}
		}
	}
}


float MovingAvgFilter(VELT_FILTER *pVeltFilter,float VeltTemp)
{
	pVeltFilter->Sum = 0;
	pVeltFilter->Velt[pVeltFilter->Num] = VeltTemp;
	pVeltFilter->Num++;
	if(pVeltFilter->First == 1)
	{
		if(pVeltFilter->Num == pVeltFilter->N)
		{
			pVeltFilter->Num = 0;
			pVeltFilter->First = 0;
		}
		return VeltTemp;
	}
	else
	{
		if(pVeltFilter->Num == pVeltFilter->N)
		{
			pVeltFilter->Num = 0;
		}
	}
	for(int i=0;i<pVeltFilter->N;i++)
	{
		pVeltFilter->Sum = pVeltFilter->Sum + pVeltFilter->Velt[i];
	}

	pVeltFilter->Output = pVeltFilter->Sum / pVeltFilter->N;
	//printf("%f\n",pVeltFilter->Output);
	return pVeltFilter->Output;
}


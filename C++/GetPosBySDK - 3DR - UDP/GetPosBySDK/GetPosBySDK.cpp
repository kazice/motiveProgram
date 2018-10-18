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
int addrServer_len;
SOCKET sockClient;
SOCKADDR_IN addrClient;
int addrClient_len;


NETLINK_DATA NetLinkData;
char chSendBuf[265];
NETLINK_DATA NetLinkCmd;
char chSendCmdBuf[265];
NETLINK_DATA NetLinkDelay;
char chSendDelayBuf[265];
float delaytime = 0;
bool recvflag;

NET_TIME net_delay = {0};

//init of PID
REAL_POS_UAV RealPosUAV = {0};
REAL_POS_UAV RealPosUAVPre = {0};

DES_POS_UAV DesPosUAV;
DES_VELT_UAV DesVeltUAV;

ATT_UAV AttUAV = {0};

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
std::ofstream outVelt("Velt.txt");
std::ofstream outPos("Pos.txt");
std::ofstream outAttitude("Attitude.txt");


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
char RecvBuf[265];

/********************************/

DWORD WINAPI NetworkThread(LPVOID pM)
{
	printf("start network delay test!\n");
	while(!NETTEST)
	{
		if(enNetLinkState == NET_LISTENING)
		{
			int Recv = recvfrom(sockServer,RecvBuf,128,0,(sockaddr *)&addrServer,&addrServer_len);
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
		if(enNetLinkState == NET_GETHEAD)
		{
	        net_link_data.Len = RecvBuf[1];
	        net_link_data.Seq = RecvBuf[2];
	        net_link_data.SysID = RecvBuf[3];
	        net_link_data.CompID = RecvBuf[4];
	        net_link_data.MsgID = RecvBuf[5];
	        enNetLinkState = NET_GETPACKSTART;
		}
		if(enNetLinkState == NET_GETPACKSTART)
		{
	        for(int data_i=0;data_i<net_link_data.Len;data_i++)
	        {
	        	net_link_data.Data.chsendBuf[data_i] = RecvBuf[data_i+6];
	        }
	        enNetLinkState = NET_GETDATA;
		}
		if(enNetLinkState == NET_GETDATA)
		{
	        net_link_data.CheckA = RecvBuf[net_link_data.Len+6];
	        net_link_data.CheckB = RecvBuf[net_link_data.Len+7];
	        net_link_data.End = RecvBuf[net_link_data.Len+8];
	        if(net_link_data.End == 0xAA)
	        {
	        	enNetLinkState = NET_GETEND;
	        }
	        else
	        {
	        	enNetLinkState = NET_LISTENING;
	        }
		}
		if(enNetLinkState == NET_GETEND)
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
		
		sendto(sockClient,chSendDelayBuf,128,0,(sockaddr *)&addrClient,sizeof(addrClient));
		recvflag = 1;
		while(recvflag)
		{
			if(enNetLinkState == NET_LISTENING)
			{
				int Recv = recvfrom(sockServer,RecvBuf,128,0,(sockaddr *)&addrServer,&addrServer_len);
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
			if(enNetLinkState == NET_GETHEAD)
			{
	        	net_link_data.Len = RecvBuf[1];
	        	net_link_data.Seq = RecvBuf[2];
	        	net_link_data.SysID = RecvBuf[3];
	        	net_link_data.CompID = RecvBuf[4];
	        	net_link_data.MsgID = RecvBuf[5];
	        	enNetLinkState = NET_GETPACKSTART;
			}
			if(enNetLinkState == NET_GETPACKSTART)
			{
	        	for(int data_i=0;data_i<net_link_data.Len;data_i++)
	        	{
	        		net_link_data.Data.chsendBuf[data_i] = RecvBuf[data_i+6];
	        	}
	        	enNetLinkState = NET_GETDATA;
			}
			if(enNetLinkState == NET_GETDATA)
			{
	        	net_link_data.CheckA = RecvBuf[net_link_data.Len+6];
	        	net_link_data.CheckB = RecvBuf[net_link_data.Len+7];
	        	net_link_data.End = RecvBuf[net_link_data.Len+8];
	        	if(net_link_data.End == 0xAA)
	        	{
	        		enNetLinkState = NET_GETEND;
	        	}
	        	else
	        	{
	        		enNetLinkState = NET_LISTENING;
	        	}
			}
			if(enNetLinkState == NET_GETEND)
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

	CreateServerforSDK();
	enNetLinkState = NET_LISTENING;
	QueryPerformanceFrequency(&DelayFreq); 
	HANDLE handle = CreateThread(NULL, 0, NetworkThread, NULL, 0, NULL);

    strcpy(szServerIPAddress, "192.168.2.254");		// not specified - assume server is local machine
	printf("Connecting to server at %s...\n", szServerIPAddress);
	strcpy(szMyIPAddress,"192.168.2.254");
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
	sockServer=socket(AF_INET,SOCK_DGRAM,0);
	addrServer.sin_addr.S_un.S_addr=inet_addr("192.168.2.254");
	addrServer.sin_family=AF_INET;
	addrServer.sin_port=htons(6000);
	bind(sockServer,(SOCKADDR*)&addrServer,sizeof(SOCKADDR));
	addrServer_len = sizeof(addrServer);
 
	sockClient=socket(AF_INET,SOCK_DGRAM,0);
	addrClient.sin_addr.S_un.S_addr=inet_addr("192.168.2.253");
	addrClient.sin_family=AF_INET;
	addrClient.sin_port=htons(6001);
	addrClient_len = sizeof(addrClient);

	printf("Client of Manifold ready to connect\n");

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
	
	NatNetClient* pClient = (NatNetClient*) pUserData;
	int i=0;

	QueryPerformanceCounter(&EndTime);
	QueryPerformanceFrequency(&Freq);
	double DataTimeInt = (double)(EndTime.QuadPart -BeginTime.QuadPart)*1000/Freq.QuadPart;
	//printf("Time is:%lf\n",DataTimeInt);

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
	/*printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);*/
	for(i=0; i < data->nRigidBodies; i++)
	{
		//printf("%d\n",data->nRigidBodies);
		//			printf("[%3.6f\t%3.6f\t%3.6f]\n",
		//		RealPosUAV.flx,
		//		RealPosUAV.fly,
		//		RealPosUAV.flz);
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame

		bool bTrackingValid = data->RigidBodies[i].params & 0x01;

		RealPosUAV.flx = data->RigidBodies[i].x;
		RealPosUAV.fly = data->RigidBodies[i].z;
		RealPosUAV.flz = data->RigidBodies[i].y;

		// Other Markers
		//printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
		for(int ii=0; ii < data->nOtherMarkers; ii++)
		{
			//printf("Other Marker %d : %3.6f\t%3.6f\t%3.6f\n",
			//	i,
			//	data->OtherMarkers[i][0],
			//	data->OtherMarkers[i][1],
			//	data->OtherMarkers[i][2]);
			//is get Pendulum
			if(data->OtherMarkers[ii][1]-RealPosUAV.flz > 1.0 && data->OtherMarkers[ii][1]-RealPosUAV.flz < 1.3)
			{
				if(data->OtherMarkers[ii][0] - RealPosUAV.flx > -0.18 && data->OtherMarkers[ii][0] - RealPosUAV.flx < 0.18)
				{
					if(data->OtherMarkers[ii][2] - RealPosUAV.fly > -0.18 && data->OtherMarkers[ii][2] - RealPosUAV.fly < 0.18)
					{
						PendulumState.Pos.flx = data->OtherMarkers[ii][0] + PENDULUM_OFFSET_X;
						PendulumState.Pos.flz = data->OtherMarkers[ii][1] + PENDULUM_OFFSET_Z;
						PendulumState.Pos.fly = data->OtherMarkers[ii][2] + PENDULUM_OFFSET_Y;
						//printf("Pendulum: %3.6f\t%3.6f\t%3.6f\n",
						//	PendulumState.Pos.flx,
						//	PendulumState.Pos.fly,
						//	PendulumState.Pos.flz);
						bPendulumValid = 1;
					}
				}
			}
		}

		AttUAV.roll = atan2(2*(data->RigidBodies[i].qw*data->RigidBodies[i].qx+data->RigidBodies[i].qy*data->RigidBodies[i].qz),1-2*(data->RigidBodies[i].qx*data->RigidBodies[i].qx+data->RigidBodies[i].qy*data->RigidBodies[i].qy));
		AttUAV.yaw = -asin(2*(data->RigidBodies[i].qw*data->RigidBodies[i].qy-data->RigidBodies[i].qz*data->RigidBodies[i].qx));
		AttUAV.pitch = atan2(2*(data->RigidBodies[i].qw*data->RigidBodies[i].qz+data->RigidBodies[i].qx*data->RigidBodies[i].qy),1-2*(data->RigidBodies[i].qy*data->RigidBodies[i].qy+data->RigidBodies[i].qz*data->RigidBodies[i].qz));

		//printf("pitch\tyaw\troll\n");
		//printf("%f\t%f\t%f\n",AttUAV.pitch/3.1415926*180,AttUAV.yaw/3.1415926*180,AttUAV.roll/3.1415926*180);

		PendulumState.Attitude.flx = asin((PendulumState.Pos.flx - RealPosUAV.flx) / PENDULUM_L);
		PendulumState.Attitude.fly = asin((PendulumState.Pos.fly - RealPosUAV.fly) / PENDULUM_L);

		//printf("Attitude of Pendulum[x y]:\n%3.6f\t%3.6f\n",PendulumState.Attitude.flx * 180 / 3.1416,PendulumState.Attitude.fly * 180 / 3.1416);

		if(bDataDisplay.Pos & !bMenuFlag)
		{
			printf("[%3.6f\t%3.6f\t%3.6f]\n",
				RealPosUAV.flx,
				RealPosUAV.fly,
				RealPosUAV.flz);
		}
		if(bTrackingValid == 1)
		{
			RealVeltUAV.flvx = (RealPosUAV.flx - RealPosUAVPre.flx) / (float)DataTimeInt * 1000;
			RealVeltUAV.flvy = (RealPosUAV.fly - RealPosUAVPre.fly) / (float)DataTimeInt * 1000;
			RealVeltUAV.flvz = (RealPosUAV.flz - RealPosUAVPre.flz) / (float)DataTimeInt * 1000;

			RealPosUAVPre.flx = RealPosUAV.flx;
			RealPosUAVPre.fly = RealPosUAV.fly;
			RealPosUAVPre.flz = RealPosUAV.flz;

		}

		if(bDataDisplay.Velt & !bMenuFlag)
		{
			printf("[%3.6f\t%3.6f\t%3.6f]\n",
				RealVeltUAV.flvx,
				RealVeltUAV.flvy,
				RealVeltUAV.flvz);
		}


		QueryPerformanceCounter(&DelayTime);
		NetLinkData.MsgID = 0x05; 
		NetLinkData.Len = 66; 
		NetLinkData.Data.chsendBuf[64] = bTrackingValid;
		NetLinkData.Data.chsendBuf[65] = bPendulumValid;

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
			NetLinkData.Data.flsendBuf[14] = net_delay.timedelay[254];
			net_delay.timedelay[254] = 30;
		}
		else if(NetLinkData.Seq == 1)
		{
			NetLinkData.Data.flsendBuf[14] = net_delay.timedelay[255];
			net_delay.timedelay[254] = 30;
		}
		else
		{
			NetLinkData.Data.flsendBuf[14] = net_delay.timedelay[NetLinkData.Seq-2];
		}
		NetLinkData.Data.flsendBuf[15] = (float)DelayTime.QuadPart*1000/DelayFreq.QuadPart;
		net_delay.timestart[NetLinkData.Seq] = NetLinkData.Data.flsendBuf[15];
		net_delay.timedelay[NetLinkData.Seq-2] = 30;
		//printf("delay last is: %f\n",net_delay.timedelay[NetLinkData.Seq-1]);

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

		if(!NETTEST)sendto(sockClient,chSendBuf,128,0,(sockaddr *)&addrClient,sizeof(addrClient));;

		NetLinkData.Seq++;
		bPendulumValid = 0;

		//outPos<<data->iFrame<<'\t'<<RealPosUAV.flx<<'\t'<<RealPosUAV.fly<<'\t'<<RealPosUAV.flz<<std::endl;
		//outAttitude<<data->iFrame<<'\t'<<RealPosUAV.flx<<'\t'<<RealPosUAV.fly<<'\t'<<RealPosUAV.flz<<std::endl;
		QueryPerformanceCounter(&BeginTime);
		

		//printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
		//printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		//printf("\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\n",
		//	data->RigidBodies[i].x,
		//	data->RigidBodies[i].y,
		//	data->RigidBodies[i].z,
		//	data->RigidBodies[i].qx,
		//	data->RigidBodies[i].qy,
		//	data->RigidBodies[i].qz,
		//	data->RigidBodies[i].qw);

		//printf("\tRigid body markers [Count=%d]\n", data->RigidBodies[i].nMarkers);
		//for(int iMarker=0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)
		//{
  //          printf("\t\t");
  //          if(data->RigidBodies[i].MarkerIDs)
  //              printf("MarkerID:%d", data->RigidBodies[i].MarkerIDs[iMarker]);
  //          if(data->RigidBodies[i].MarkerSizes)
  //              printf("\tMarkerSize:%3.6f", data->RigidBodies[i].MarkerSizes[iMarker]);
  //          if(data->RigidBodies[i].Markers)
  //              printf("\tMarkerPos:%3.6f,%3.6f,%3.6f\n" ,
  //                  data->RigidBodies[i].Markers[iMarker][0],
  //                  data->RigidBodies[i].Markers[iMarker][1],
  //                  data->RigidBodies[i].Markers[iMarker][2]);
  //      }
	}

}

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



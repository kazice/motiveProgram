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

//init of socket of MATLAB
WSADATA wsaDataM;
SOCKET sockServerM;
SOCKADDR_IN addrServerM;
SOCKET sockClientM;
SOCKADDR_IN addrClientM;

Send SendData;
//SendVelt SendVeltData;
SendCmd SendCmdData;
SendVelt SendVeltData2MATLAB;
char recvBuf[100];

//init of PID
PID_CRL PidCrl_z = {0};
PID_CRL PidCrl_y = {0};
PID_CRL PidCrl_x = {0};
REAL_POS_UAV RealPosUAV = {0};
REAL_POS_UAV RealPosUAVPre = {0};

DES_POS_UAV InitDesPos ={-0.3,0,0.8};

DES_POS_UAV DesPosUAV;
DES_VELT_UAV DesVeltUAV;
EN_RUN_STATE enRunStateX;
EN_RUN_STATE enRunStateY;
EN_RUN_STATE enRunStateZ;

EN_CONTROL_STATE enControlStateX;
EN_CONTROL_STATE enControlStateY;
EN_CONTROL_STATE enControlStateZ;


//init of GetTime
LARGE_INTEGER Freq;
LARGE_INTEGER BeginTime;
LARGE_INTEGER EndTime;

//init of GetSpeed
REAL_VELT_UAV RealVeltUAV;
std::ofstream outVelt("Velt.txt");
std::ofstream outPos("Pos.txt");
std::ofstream outAttitude("Attitude.txt");


int _tmain(int argc, _TCHAR* argv[])
{
    int iResult;


	PidCrl_z.flP = 1.0f;
	PidCrl_z.flI = 0;
	PidCrl_z.flD = 0;

	PidCrl_y.flP = 3.5f;
	PidCrl_y.flI = 0.001f;
	PidCrl_y.flD = 0;

	PidCrl_x.flP = 3.5f;
	PidCrl_x.flI = 0.001f;
	PidCrl_x.flD = 0;

	PidPendulum_x.flP = 0.5f;
	PidPendulum_x.flI = 0.01f;
	PidPendulum_x.flD = 0;


	PidPendulum_y.flP = 0.5f;
	PidPendulum_y.flI = 0.01f;
	PidPendulum_y.flD = 0;
     
	//Head and End of Cmd Message
	SendCmdData.chsendCmdBuf[0] = 0x55;
	SendCmdData.chsendCmdBuf[19] = 0xAA;

	CreateServerforSDK();
	//CreateServerforMATLAB();
    strcpy(szServerIPAddress, "192.168.1.254");		// not specified - assume server is local machine
	printf("Connecting to server at %s...\n", szServerIPAddress);
	strcpy(szMyIPAddress,"192.168.1.254");
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
	addrServer.sin_addr.S_un.S_addr=inet_addr("192.168.1.254");
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

bool CreateServerforMATLAB(void)
{
	WSAStartup(MAKEWORD(2,2),&wsaDataM);
	sockServerM=socket(AF_INET,SOCK_STREAM,0);
	addrServerM.sin_addr.S_un.S_addr=inet_addr("192.168.1.254");
	addrServerM.sin_family=AF_INET;
	addrServerM.sin_port=htons(5999);
	bind(sockServerM,(SOCKADDR*)&addrServerM,sizeof(SOCKADDR));
 
	listen(sockServerM,5);
	printf("Server had start:\nListening...5999\n");
	int len=sizeof(SOCKADDR);
	
	sockClientM=accept(sockServerM,(SOCKADDR*)&addrClientM,&len);
	printf("Client of MATLAB had connect\n");

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

	//// Other Markers
	//printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
	for(i=0; i < data->nOtherMarkers; i++)
	{
		//printf("Other Marker %d : %3.6f\t%3.6f\t%3.6f\n",
		//	i,
		//	data->OtherMarkers[i][0],
		//	data->OtherMarkers[i][1],
		//	data->OtherMarkers[i][2]);

		PendulumState.Pos.flx = data->OtherMarkers[i][0];
		PendulumState.Pos.flz = data->OtherMarkers[i][1];
		PendulumState.Pos.fly = data->OtherMarkers[i][2];
	}

	
	// Rigid Bodies
	/*printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);*/
	for(i=0; i < data->nRigidBodies; i++)
	{
		//printf("%d\n",data->nRigidBodies);
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
		//SendData.chsendBuf[28] = 'a';
		bool bTrackingValid = data->RigidBodies[i].params & 0x01;
		//SendData.flsendBuf[0]=data->RigidBodies[i].x;
		//SendData.flsendBuf[1]=data->RigidBodies[i].y;
		//SendData.flsendBuf[2]=data->RigidBodies[i].z;
		//SendData.flsendBuf[3]=data->RigidBodies[i].qx;
		//SendData.flsendBuf[4]=data->RigidBodies[i].qy;
		//SendData.flsendBuf[5]=data->RigidBodies[i].qz;
		//SendData.flsendBuf[6]=data->RigidBodies[i].qw;

		RealPosUAV.flx = data->RigidBodies[i].x;
		RealPosUAV.fly = data->RigidBodies[i].z;
		RealPosUAV.flz = data->RigidBodies[i].y;

		

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

			outVelt<<data->iFrame<<'\t'<<RealVeltUAV.flvx<<'\t'<<RealVeltUAV.flvy<<'\t'<<RealVeltUAV.flvz<<std::endl;
		}

		if(bDataDisplay.Velt & !bMenuFlag)
		{
			printf("[%3.6f\t%3.6f\t%3.6f]\n",
				RealVeltUAV.flvx,
				RealVeltUAV.flvy,
				RealVeltUAV.flvz);
		}

		SendVeltData2MATLAB.flsendVeltBuf[1] = _ltos(RealVeltUAV.flvx);
		SendVeltData2MATLAB.flsendVeltBuf[2] = _ltos(RealVeltUAV.flvy);
		SendVeltData2MATLAB.flsendVeltBuf[3] = _ltos(RealVeltUAV.flvz);

		/*************************************************
		*Controller of Pendulum
		*
		***************************************************/
		if(bTrackingValid == 1 && data->nOtherMarkers == 1)
		{
			IsPendStart();
			if(enPendulumRunState == P_FIRST)
			{
				SendCmdData.chsendCmdBuf[1] = 0x02; 
				SendCmdData.chsendCmdBuf[2] = 'f';
				send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				enPendulumRunState = P_RUN;
			}
			if(enPendulumRunState == P_RUN)
			{
				printf("OK\n");

				PidPendulum_x.flE = PendulumState.Attitude.flx * 180 / pi;
				CalAbsPendPID(&PidPendulum_x);
				SendCmdData.flsendCmdBuf[2] = -PidPendulum_x.flU;
				PidPendulum_y.flE = PendulumState.Attitude.fly * 180 / pi;
				CalAbsPendPID(&PidPendulum_y);

				SendCmdData.flsendCmdBuf[1] = PidPendulum_y.flU;
				SendCmdData.chsendCmdBuf[1] = 0x03; 
				send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
			}
			IsPendFail();
		}
		else
		{

		}


		/****************Auto Start Posloop*******************/
		if(bTrackingValid == 1)
		{
			if(enRunStateX == RUN || enRunStateY == RUN || enRunStateZ == RUN
			|| enRunStateX == FIRST || enRunStateY == FIRST || enRunStateZ == FIRST)
			{
				
			}
			else
			{
				IsPosloopStart();
			}
		}


		//send(sockClientM,SendVeltData2MATLAB.chsendVeltBuf,sizeof(SendCmdData.chsendCmdBuf),0);

		/************PosLoop of X-axis****************/
		//Pos-Attitude mode
		if(enControlStateX = POS)
		{
			if(enRunStateX == FIRST)
			{
				enRunStateX = RUN;
			}
			if(enRunStateX == RUN)
			{
				if(bTrackingValid == 1)
				{
					PidCrl_x.flE = DesPosUAV.flx - RealPosUAV.flx;
					CalAbsPID(&PidCrl_x);
					SendCmdData.flsendCmdBuf[2] = ClipFloat(-PidCrl_x.flU,-PitchLimit,PitchLimit);
					if(bDataDisplay.PosX & !bMenuFlag)
					{
						printf("DesPos of UAV is:%f\n",DesPosUAV.flx);
						printf("RealPos of UAV is:%f; RealVelt of UAV is:%f\n",RealPosUAV.flx,RealVeltUAV.flvx);
					}
					if(bDataDisplay.OutputAttitudePitch & !bMenuFlag)
					{
						printf("OutPut of Pitch is:%f\n",SendCmdData.flsendCmdBuf[2]);
					}
					SendCmdData.chsendCmdBuf[1] = 0x03; 
					send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}
				else
				{
					enRunStateX = DONE;
					printf("Lost the UAV!!!\n");
					SendCmdData.flsendCmdBuf[1] = 0;
					SendCmdData.chsendCmdBuf[1] = 0x01; 
					send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}
			}
			if(enRunStateX == DONE)
			{
				SendVeltZero2UAV();
			}
		}
		//Velt mode
		else if(enControlStateX = VELT)
		{
			if(enRunStateX == FIRST)
			{
				enRunStateX = RUN;
			}
			if(enRunStateX == RUN)
			{
				if(bTrackingValid == 1)
				{
					if(bDataDisplay.PosX & !bMenuFlag)
					{
						printf("DesPos of UAV is:%f\n",DesPosUAV.flx);
						printf("RealPos of UAV is:%f; RealVelt of UAV is:%f\n",RealPosUAV.flx,RealVeltUAV.flvx);
					}
					if(bDataDisplay.OutputVeltX & !bMenuFlag)
					{
						printf("OutPut of Velt is:%f\n",SendCmdData.flsendCmdBuf[1]);
					}
					SendCmdData.flsendCmdBuf[1] = DesVeltUAV.flx;
					SendCmdData.chsendCmdBuf[1] = 0x01; 
					send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}
				else
				{
					enRunStateX = DONE;
					printf("Lost the UAV!!!\n");
					SendCmdData.flsendCmdBuf[1] = 0;
					SendCmdData.chsendCmdBuf[1] = 0x01; 
					send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}
			}
			if(enRunStateX == DONE)
			{
				SendVeltZero2UAV();
			}
		}

		/************PosLoop of Y-axis****************/
		//Pos-Attitude mode
		if(enControlStateY = POS)
		{
			if(enRunStateY == FIRST)
			{
				enRunStateY = RUN;
			}
			if(enRunStateY == RUN)
			{
				if(bTrackingValid == 1)
				{
					PidCrl_y.flE = DesPosUAV.fly - RealPosUAV.fly;
					CalAbsPID(&PidCrl_y);
					SendCmdData.flsendCmdBuf[1] = ClipFloat(PidCrl_y.flU,-RollLimit,RollLimit);
					if(bDataDisplay.PosY & !bMenuFlag)
					{
						printf("DesPos of UAV is:%f\n",DesPosUAV.fly);
						printf("RealPos of UAV is:%f; RealVelt of UAV is:%f\n",RealPosUAV.fly,RealVeltUAV.flvy);
					}
					if(bDataDisplay.OutputAttitudeRoll & !bMenuFlag)
					{
						printf("OutPut of Roll is:%f\n",SendCmdData.flsendCmdBuf[1]);
					}
					SendCmdData.chsendCmdBuf[1] = 0x03; 
					send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}
				else
				{
					enRunStateY = DONE;
					printf("Lost the UAV!!!\n");
					SendCmdData.flsendCmdBuf[2] = 0;
					SendCmdData.chsendCmdBuf[1] = 0x01; 
					send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}
			}
			if(enRunStateY == DONE)
			{
				SendVeltZero2UAV();
			}
		}
		//Velt mode
		else if(enControlStateY = VELT)
		{
			if(enRunStateY == FIRST)
			{
				enRunStateY = RUN;
			}
			if(enRunStateY == RUN)
			{
				if(bTrackingValid == 1)
				{
					if(bDataDisplay.PosY & !bMenuFlag)
					{
						printf("DesPos of UAV is:%f\n",DesPosUAV.fly);
						printf("RealPos of UAV is:%f; RealVelt of UAV is:%f\n",RealPosUAV.fly,RealVeltUAV.flvy);
					}
					if(bDataDisplay.OutputVeltY & !bMenuFlag)
					{
						printf("OutPut of Velt is:%f\n",SendCmdData.flsendCmdBuf[2]);
					}
					SendCmdData.flsendCmdBuf[2] = DesVeltUAV.fly;
					SendCmdData.chsendCmdBuf[1] = 0x01; 
					send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}
				else
				{
					enRunStateY = DONE;
					printf("Lost the UAV!!!\n");
					SendCmdData.flsendCmdBuf[2] = 0;
					SendCmdData.chsendCmdBuf[1] = 0x01; 
					send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}
			}
			if(enRunStateY == DONE)
			{
				SendVeltZero2UAV();
			}
		}

		/************PosLoop of Z-axis****************/
		//Attitude mode
		if(enControlStateZ = POS)
		{
			if(enRunStateZ == FIRST)
			{
				enRunStateZ = RUN;
			}
			if(enRunStateZ == RUN)
			{
				if(bTrackingValid == 1)
				{
					PidCrl_z.flE = DesPosUAV.flz - RealPosUAV.flz;
					CalAbsPID(&PidCrl_z);
					SendCmdData.flsendCmdBuf[3] = PidCrl_z.flU;
					if(bDataDisplay.PosZ & !bMenuFlag)
					{
						printf("DesPos of UAV is:%f\n",DesPosUAV.flz);
						printf("RealPos of UAV is:%f\n",RealPosUAV.flz);
					}
					if(bDataDisplay.OutputVeltZ & !bMenuFlag)
					{
						printf("OutPut of VeltZ is:%f\n",SendCmdData.flsendCmdBuf[3]);
					}
					SendCmdData.chsendCmdBuf[1] = 0x01; 
					send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}
				else
				{
						enRunStateZ = DONE;
						printf("Lost the UAV!!!\n");
						SendCmdData.flsendCmdBuf[3] = 0;
						SendCmdData.chsendCmdBuf[1] = 0x01; 
						send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
				}

			}
			if(enRunStateZ == DONE)
			{
				SendVeltZero2UAV();
			}
		}
		else if(enControlStateZ = VELT)
		{

		}

		outPos<<data->iFrame<<'\t'<<RealPosUAV.flx<<'\t'<<RealPosUAV.fly<<'\t'<<RealPosUAV.flz<<std::endl;
		//outAttitude<<data->iFrame<<'\t'<<RealPosUAV.flx<<'\t'<<RealPosUAV.fly<<'\t'<<RealPosUAV.flz<<std::endl;
		QueryPerformanceCounter(&BeginTime);
		

		//send(sockClient,SendData.chsendBuf,sizeof(SendData.chsendBuf),0);
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
	printf("| [b] Request Control           | [j] Input DesPos of X           |\n");
	printf("| [b]                           | [l] Input DesPos of Y           |\n");
	printf("| [c] Release Control           | [q] Quit the Server             |\n");	
	printf("| [d] Takeoff                   | [s] Show/Hide Debug Data        |\n");	
	printf("| [e] Landing                   | [k] Show/Hide Main menu         |\n");
	printf("| [x] Input DesVelt of X        | [y] Input DesVelt of Y          |\n");
	printf("| [z] resetUAV                  |                                 |\n");
	printf("| [f] Start Protect             | [g] Stop Protect                |\n");
	printf("+-----------------------------------------------------------------+\n");
    printf("input a/b/c etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}

void SendVeltZero2UAV(void)
{
	if(enRunStateX == DONE)
	{
		SendCmdData.flsendCmdBuf[1] = 0;
	}
	if(enRunStateY == DONE)
	{
		SendCmdData.flsendCmdBuf[2] = 0;
	}
	if(enRunStateZ == DONE)
	{
		SendCmdData.flsendCmdBuf[3] = 0;
	}
	SendCmdData.chsendCmdBuf[1] = 0x01; 
	send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);
}

void IsPosloopStart(void)
{
	if(RealPosUAV.flx > InitDesPos.flx + XlimitMin && RealPosUAV.flx < InitDesPos.flx + XlimitMax 
	&& RealPosUAV.fly > InitDesPos.fly + YlimitMin && RealPosUAV.fly < InitDesPos.fly + YlimitMax
	&& RealPosUAV.flz > InitDesPos.flz + ZlimitMin) 
	{

		SendCmdData.chsendCmdBuf[1] = 0x02; 
		SendCmdData.chsendCmdBuf[2] = 'b';
		send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);

		SendCmdData.chsendCmdBuf[1] = 0x02; 
		SendCmdData.chsendCmdBuf[2] = 'z';
		send(sockClient,SendCmdData.chsendCmdBuf,sizeof(SendCmdData.chsendCmdBuf),0);

		DesPosUAV.flx = InitDesPos.flx;
		enControlStateX = POS;
		enRunStateX = FIRST;

		DesPosUAV.fly = InitDesPos.fly;
		enControlStateY = POS;
		enRunStateY = FIRST;

		DesPosUAV.flz = InitDesPos.flz;
		enControlStateZ = POS;
		enRunStateZ = FIRST;
	}
}
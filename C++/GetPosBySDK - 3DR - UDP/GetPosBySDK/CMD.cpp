#include "StdAfx.h"
#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>
#include "PIDController.h"
#include "GetPosBySDK.h"
#include "CMD.h"

#pragma warning( disable : 4996 )
int c;

#define PID_LEN_POS_VELT 29
#define PID_LEN_PENDULUM 6
bool bExit = false;
int cmd_dis;
bool bMenuFlag = false;
DATA_DIS bDataDisplay = {false};
FILE*fp=NULL;
float fp_read_temp;




void doSendCmd(void)
{
	NetLinkCmd.Len = 1;
	NetLinkCmd.MsgID = 0x02;

	chSendCmdBuf[0] = 0xFE;
	chSendCmdBuf[1] = NetLinkCmd.Len;
	chSendCmdBuf[2] = NetLinkCmd.Seq;
	chSendCmdBuf[3] = NetLinkCmd.SysID;
	chSendCmdBuf[4] = NetLinkCmd.CompID;
	chSendCmdBuf[5] = NetLinkCmd.MsgID;
	for(int data_i=0;data_i<NetLinkCmd.Len;data_i++)
	{
		chSendCmdBuf[data_i+6] = NetLinkCmd.Data.chsendBuf[data_i];
	}
	chSendCmdBuf[6+NetLinkCmd.Len] = NetLinkCmd.CheckA;
	chSendCmdBuf[7+NetLinkCmd.Len] = NetLinkCmd.CheckB;
	chSendCmdBuf[8+NetLinkCmd.Len] = 0xAA;

	sendto(sockClient,chSendCmdBuf,128,0,(sockaddr *)&addrClient,sizeof(addrClient));
}

void doSendPos(void)
{
	NetLinkCmd.Len = 12;
	NetLinkCmd.MsgID = 0x04;

	chSendCmdBuf[0] = 0xFE;
	chSendCmdBuf[1] = NetLinkCmd.Len;
	chSendCmdBuf[2] = NetLinkCmd.Seq;
	chSendCmdBuf[3] = NetLinkCmd.SysID;
	chSendCmdBuf[4] = NetLinkCmd.CompID;
	chSendCmdBuf[5] = NetLinkCmd.MsgID;
	for(int data_i=0;data_i<NetLinkCmd.Len;data_i++)
	{
		chSendCmdBuf[data_i+6] = NetLinkCmd.Data.chsendBuf[data_i];
	}
	chSendCmdBuf[6+NetLinkCmd.Len] = NetLinkCmd.CheckA;
	chSendCmdBuf[7+NetLinkCmd.Len] = NetLinkCmd.CheckB;
	chSendCmdBuf[8+NetLinkCmd.Len] = 0xAA;

	sendto(sockClient,chSendCmdBuf,128,0,(sockaddr *)&addrClient,sizeof(addrClient));
}

void CMD_deal(int cmd)
{	
	printf("The input CMD is:%c\n",cmd);
	switch(cmd)
	{
		case 'a':
			NetLinkCmd.Data.chsendBuf[0] = c;
			doSendCmd();
			break;
		case 'b':
			NetLinkCmd.Data.chsendBuf[0] = c;
			doSendCmd();
			break;
		case 'c':
			NetLinkCmd.Data.chsendBuf[0] = c;
			doSendCmd();
			break;
		case 'd':
			NetLinkCmd.Data.chsendBuf[0] = c;
			doSendCmd();
			break;
		case 'e':
			NetLinkCmd.Data.chsendBuf[0] = c;
			doSendCmd();
			break;
		case 'f':
			fp=fopen("C:\\Users\\user\\Desktop\\Yan Wei\\C++\\GetPosBySDK - 3DR\\GetPosBySDK\\pid_pos_velt.txt","r+");
			if(NULL==fp)
			{
				printf("No pid file\n");
			}
			else
			{
				printf("\n\nget pid data\n");
				printf("x y z roll yaw pitch vx vy vz landspeed takeoffspeed\nP\tI\tD\n");
				for(int i=0;i<PID_LEN_POS_VELT;i++)
				{
					fscanf(fp,"%f",&NetLinkCmd.Data.flsendBuf[i]);
					//NetLinkCmd.Data.flsendBuf[i] = (float)fp_read_temp;
					printf("%f\t",NetLinkCmd.Data.flsendBuf[i]);
					if(i%3 == 2)
					{
						printf("\n");
					}
				}
				printf("\n\n");
				fclose(fp);
				fp = NULL;
				NetLinkCmd.Len = 4*PID_LEN_POS_VELT;
				NetLinkCmd.MsgID = 0x08;

				chSendCmdBuf[0] = 0xFE;
				chSendCmdBuf[1] = NetLinkCmd.Len;
				chSendCmdBuf[2] = NetLinkCmd.Seq;
				chSendCmdBuf[3] = NetLinkCmd.SysID;
				chSendCmdBuf[4] = NetLinkCmd.CompID;
				chSendCmdBuf[5] = NetLinkCmd.MsgID;
				for(int data_i=0;data_i<NetLinkCmd.Len;data_i++)
				{
					chSendCmdBuf[data_i+6] = NetLinkCmd.Data.chsendBuf[data_i];
				}
				chSendCmdBuf[6+NetLinkCmd.Len] = NetLinkCmd.CheckA;
				chSendCmdBuf[7+NetLinkCmd.Len] = NetLinkCmd.CheckB;
				chSendCmdBuf[8+NetLinkCmd.Len] = 0xAA;

				sendto(sockClient,chSendCmdBuf,128,0,(sockaddr *)&addrClient,sizeof(addrClient));
			}
			
			break;
		case 'g':
			fp=fopen("C:\\Users\\user\\Desktop\\Yan Wei\\C++\\GetPosBySDK - 3DR\\GetPosBySDK\\pid_pendulum.txt","r+");
			if(NULL==fp)
			{
				printf("No pid file\n");
			}
			else
			{
				printf("\n\nget pid data\n");
				printf("x y\nP\tI\tD\n");
				for(int i=0;i<PID_LEN_PENDULUM;i++)
				{
					fscanf(fp,"%f",&NetLinkCmd.Data.flsendBuf[i]);
					//NetLinkCmd.Data.flsendBuf[i] = (float)fp_read_temp;
					printf("%f\t",NetLinkCmd.Data.flsendBuf[i]);
					if(i%3 == 2)
					{
						printf("\n");
					}
				}
				printf("\n\n");
				fclose(fp);
				fp = NULL;
				NetLinkCmd.Len = 4*PID_LEN_PENDULUM;
				NetLinkCmd.MsgID = 0x09;

				chSendCmdBuf[0] = 0xFE;
				chSendCmdBuf[1] = NetLinkCmd.Len;
				chSendCmdBuf[2] = NetLinkCmd.Seq;
				chSendCmdBuf[3] = NetLinkCmd.SysID;
				chSendCmdBuf[4] = NetLinkCmd.CompID;
				chSendCmdBuf[5] = NetLinkCmd.MsgID;
				for(int data_i=0;data_i<NetLinkCmd.Len;data_i++)
				{
					chSendCmdBuf[data_i+6] = NetLinkCmd.Data.chsendBuf[data_i];
				}
				chSendCmdBuf[6+NetLinkCmd.Len] = NetLinkCmd.CheckA;
				chSendCmdBuf[7+NetLinkCmd.Len] = NetLinkCmd.CheckB;
				chSendCmdBuf[8+NetLinkCmd.Len] = 0xAA;

				sendto(sockClient,chSendCmdBuf,128,0,(sockaddr *)&addrClient,sizeof(addrClient));
			}
			break;
		case 'i':
			printf("Input the DesPos of UAV:");
			scanf("%f",&DesPosUAV.flz);
			if(DesPosUAV.flz < 0.3f ||DesPosUAV.flz > 1.0f)
			{
				printf("The Input is error, Please input 'i' and try again!\n");
			}
			else
			{
				printf("The DesPos of Z is:%f\n",DesPosUAV.flz);
				NetLinkCmd.Data.flsendBuf[0] = 5555;
				NetLinkCmd.Data.flsendBuf[1] = 5555;
				NetLinkCmd.Data.flsendBuf[2] = DesPosUAV.flz;
				doSendPos();

			}
			break;
		case 'j':
			printf("Input the DesPos of UAV:");
			scanf("%f",&DesPosUAV.flx);
			if(DesPosUAV.flx < -1.2f ||DesPosUAV.flx > 0.2f)
			{
				printf("The Input is error, Please input 'j' and try again!\n");
			}
			else
			{
				printf("The DesPos of X is:%f\n",DesPosUAV.flx);
				NetLinkCmd.Data.flsendBuf[0] = DesPosUAV.flx;
				NetLinkCmd.Data.flsendBuf[1] = 5555;
				NetLinkCmd.Data.flsendBuf[2] = 5555;
				doSendPos();
			}
			break;
		case 'l':
			printf("Input the DesPos of UAV:");
			scanf("%f",&DesPosUAV.fly);
			if(DesPosUAV.fly < -0.7f ||DesPosUAV.fly > 0.7f)
			{
				printf("The Input is error, Please input 'l' and try again!\n");
			}
			else
			{
				printf("The DesPos of Y is:%f\n",DesPosUAV.fly);
				NetLinkCmd.Data.flsendBuf[0] = 5555;
				NetLinkCmd.Data.flsendBuf[1] = DesPosUAV.fly;
				NetLinkCmd.Data.flsendBuf[2] = 5555;
				doSendPos();
			}
			break;
		case 'k':
			bMenuFlag = !bMenuFlag;
			break;
		case 'q':
			bExit = true;
			closesocket(sockClient);
			outVelt.close();
			outPos.close();
			outAttitude.close();
			break;	
		case 's':
			Display_CMD_Deal();
			break;
		case 'r':
			resetClient();
			break;
		case 'x':
			printf("Input the DesVelt of UAV:");
			scanf("%f",&DesVeltUAV.flx);
			if(DesVeltUAV.flx < -2.0f ||DesVeltUAV.flx > 2.0f)
			{
				printf("The Input is error, Please input 'x' and try again!\n");
			}
			else
			{
				printf("The DesVelt of X is:%f\n",DesVeltUAV.flx);

			}
			break;
		case 'y':
			printf("Input the DesVelt of UAV:");
			scanf("%f",&DesVeltUAV.fly);
			if(DesVeltUAV.fly < -2.0f ||DesVeltUAV.fly > 2.0f)
			{
				printf("The Input is error, Please input 'y' and try again!\n");
			}
			else
			{
				printf("The DesVelt of Y is:%f\n",DesVeltUAV.fly);

			}
			break;
		//case 'p':
		//    sServerDescription ServerDescription;
		//    memset(&ServerDescription, 0, sizeof(ServerDescription));
		//    theClient->GetServerDescription(&ServerDescription);
		//    if(!ServerDescription.HostPresent)
		//    {
		//        printf("Unable to connect to server. Host not present. Exiting.");
		//        return 1;
		//    }
		//    break;	
		//case 'f':
		//    {
		//        sFrameOfMocapData* pData = theClient->GetLastFrameOfData();
		//        printf("Most Recent Frame: %d", pData->iFrame);
		//    }
		//    break;	
		//case 'm':	                        // change to multicast
		//    iConnectionType = ConnectionType_Multicast;
		//    iResult = CreateClient(iConnectionType);
		//    if(iResult == ErrorCode_OK)
		//        printf("Client connection type changed to Multicast.\n\n");
		//    else
		//        printf("Error changing client connection type to Multicast.\n\n");
		//    break;
		//case 'u':	                        // change to unicast
		//    iConnectionType = ConnectionType_Unicast;
		//    iResult = CreateClient(iConnectionType);
		//    if(iResult == ErrorCode_OK)
		//        printf("Client connection type changed to Unicast.\n\n");
		//    else
		//        printf("Error changing client connection type to Unicast.\n\n");
		//    break;
		//case 'y' :                          // connect
		//    iResult = CreateClient(iConnectionType);
		//    break;
		//case 'z' :                          // disconnect
		//    // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
		//    iResult = theClient->SendMessageAndWait("Disconnect", &response, &nBytes);
		//    if (iResult == ErrorCode_OK)
		//        printf("[SampleClient] Disconnected");
		//    break;
		default:
			break;
	}


}

void Display_CMD_Deal(void)
{
	bMenuFlag = true;
	Display_CMD_Menu();
	cmd_dis =_getch();
	printf("The input CMD is:%c\n",cmd_dis);
	switch(cmd_dis)
	{
		case 'a':
			bDataDisplay.PosX = !bDataDisplay.PosX;
			break;
		case 'b':
			bDataDisplay.PosY = !bDataDisplay.PosY;
			break;
		case 'c':
			bDataDisplay.PosZ = !bDataDisplay.PosZ;
			break;
		case 'd':
			bDataDisplay.VeltX = !bDataDisplay.VeltX;
			break;
		case 'e':
			bDataDisplay.VeltY = !bDataDisplay.VeltY;
			break;
		case 'f':
			bDataDisplay.VeltZ = !bDataDisplay.VeltZ;
			break;
		case 'g':
			bDataDisplay.Pos = !bDataDisplay.Pos;
			break;
		case 'h':
			bDataDisplay.Velt = !bDataDisplay.Velt;
			break;
		case 'i':
			bDataDisplay.OutputVeltX = !bDataDisplay.OutputVeltX;
			break;
		case 'j':
			bDataDisplay.OutputVeltY = !bDataDisplay.OutputVeltY;
			break;
		case 'k':
			bDataDisplay.OutputVeltZ = !bDataDisplay.OutputVeltZ;
			break;
		case 'l':
			bDataDisplay.OutputAttitudeRoll = !bDataDisplay.OutputAttitudeRoll;
			break;
		case 'm':
			bDataDisplay.OutputAttitudePitch = !bDataDisplay.OutputAttitudePitch;
			break;
		case 'n':
			bDataDisplay.OutputAttitudeYaw = !bDataDisplay.OutputAttitudeYaw;
			break;
		case 'q':
			break;
		default:
			break;
	}
	bMenuFlag = false;
}

void Display_CMD_Menu(void)
{
	printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [a] Pos of x(%d)          | [q] Quit without change     |\n",bDataDisplay.PosX);
	printf("| [b] Pos of y(%d)          | [g] Pos of UAV by Track(%d) |\n",bDataDisplay.PosY,bDataDisplay.Pos);	
	printf("| [c] Pos of z(%d)          | [h] Velt of UAV by Track(%d)|\n",bDataDisplay.PosZ,bDataDisplay.Velt);	
	printf("| [d] Velt of x(%d)         | [i] Output of VeltX(%d)     |\n",bDataDisplay.VeltX,bDataDisplay.OutputVeltX);	
	printf("| [e] Velt of y(%d)         | [j] Output of VeltY(%d)     |\n",bDataDisplay.VeltY,bDataDisplay.OutputVeltY);
	printf("| [f] Velt of z(%d)         | [k] Output of VeltX(%d)     |\n",bDataDisplay.VeltZ,bDataDisplay.OutputVeltZ);
	printf("| [l] Attitude of roll(%d)  |                             |\n",bDataDisplay.OutputAttitudeRoll);
	printf("| [m] Attitude of pitch(%d) |                             |\n",bDataDisplay.OutputAttitudePitch);
	printf("| [n] Attitude of yaw(%d)   |                             |\n",bDataDisplay.OutputAttitudeYaw);
	printf("+-----------------------------------------------------------------+\n");
    printf("input a/b/c etc..then press enter key\r\n");
    printf("The windows will show/hide the data you want to see(0-hide,1-show)\r\n");
    printf("----------------------------------------\r\n");
}

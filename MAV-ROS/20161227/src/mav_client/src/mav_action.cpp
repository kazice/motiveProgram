/*
 * mav_action.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: yw
 */
#include <mav_client/mav_action.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PID_DATA_LEN 29
short land_speed = 2;
short takeoff_speed = 2;

POS_MAV desPosMAV;
VELT_MAV desVeltMAV;
ATT_MAV desAttMAV;

POS_MAV realPosMAV;
VELT_MAV realVeltMAV;
ATT_MAV realAttMAV;

PID_CRL_MAV pidCrlMAV;
EN_RUN_STATE enTakeoffState = NEVER;
EN_CONTROL_STATE enControlStateX = POS;
EN_CONTROL_STATE enControlStateY = POS;
EN_CONTROL_STATE enControlStateZ = OPEN;

mavros_msgs::OverrideRCIn RC_controller_output;


void data_init(void)
{
	FILE *fp;
	float pid_data[PID_DATA_LEN];
	fp=fopen("/home/ubuntu/catkin_ws/src/mav_client/src/pid.txt","r+");
	if(!fp)
	{
		printf("data is not exist,check it\n");
		printf("use default data\n");
		pidCrlMAV.pidPosx.flP = 1000;
		pidCrlMAV.pidPosx.flI = 0.0;
		pidCrlMAV.pidPosx.flD = 0.0;

		pidCrlMAV.pidPosy.flP = 1000;
		pidCrlMAV.pidPosy.flI = 0.0;
		pidCrlMAV.pidPosy.flD = 0.0;

		pidCrlMAV.pidPosz.flP = 500;
		pidCrlMAV.pidPosz.flI = 2.0;
		pidCrlMAV.pidPosz.flD = 0.0;

		pidCrlMAV.pidRoll.flP = 300;
		pidCrlMAV.pidRoll.flI = 0.0;
		pidCrlMAV.pidRoll.flD = 0.0;

		pidCrlMAV.pidYaw.flP = 500;
		pidCrlMAV.pidYaw.flI = 0.0;
		pidCrlMAV.pidYaw.flD = 0.0;

		pidCrlMAV.pidPitch.flP = 0.1;
		pidCrlMAV.pidPitch.flI = 0.0;
		pidCrlMAV.pidPitch.flD = 0.0;

		pidCrlMAV.pidVeltx.flP = 300;
		pidCrlMAV.pidVeltx.flI = 0.0;
		pidCrlMAV.pidVeltx.flD = 0.0;

		pidCrlMAV.pidVelty.flP = 300;
		pidCrlMAV.pidVelty.flI = 0.0;
		pidCrlMAV.pidVelty.flD = 0.0;

		pidCrlMAV.pidVeltz.flP = 200;
		pidCrlMAV.pidVeltz.flI = 3.0;
		pidCrlMAV.pidVeltz.flD = 0.0;
	}
	else
	{
		printf("read PID data.......\n");
		for(int i=0;i<PID_DATA_LEN;i++)
		{
			 fscanf(fp,"%f",&pid_data[i]);
			 printf("%f\n",pid_data[i]);
		}
		fclose(fp);
		pidCrlMAV.pidPosx.flP = pid_data[0];
		pidCrlMAV.pidPosx.flI = pid_data[1];
		pidCrlMAV.pidPosx.flD = pid_data[2];

		pidCrlMAV.pidPosy.flP = pid_data[3];
		pidCrlMAV.pidPosy.flI = pid_data[4];
		pidCrlMAV.pidPosy.flD = pid_data[5];

		pidCrlMAV.pidPosz.flP = pid_data[6];
		pidCrlMAV.pidPosz.flI = pid_data[7];
		pidCrlMAV.pidPosz.flD = pid_data[8];

		pidCrlMAV.pidRoll.flP = pid_data[9];
		pidCrlMAV.pidRoll.flI = pid_data[10];
		pidCrlMAV.pidRoll.flD = pid_data[11];

		pidCrlMAV.pidYaw.flP = pid_data[12];
		pidCrlMAV.pidYaw.flI = pid_data[13];
		pidCrlMAV.pidYaw.flD = pid_data[14];

		pidCrlMAV.pidPitch.flP = pid_data[15];
		pidCrlMAV.pidPitch.flI = pid_data[16];
		pidCrlMAV.pidPitch.flD = pid_data[17];

		pidCrlMAV.pidVeltx.flP = pid_data[18];
		pidCrlMAV.pidVeltx.flI = pid_data[19];
		pidCrlMAV.pidVeltx.flD = pid_data[20];

		pidCrlMAV.pidVelty.flP = pid_data[21];
		pidCrlMAV.pidVelty.flI = pid_data[22];
		pidCrlMAV.pidVelty.flD = pid_data[23];

		pidCrlMAV.pidVeltz.flP = pid_data[24];
		pidCrlMAV.pidVeltz.flI = pid_data[25];
		pidCrlMAV.pidVeltz.flD = pid_data[26];

		land_speed = (short)pid_data[27];
		takeoff_speed = (short)pid_data[28];
		printf("data reload done\n");
	}



}

void mav_takeoff(void)
{
	if(enTakeoffState == NEVER)
	{
		enTakeoffState = FIRST;
	}
	else if(enTakeoffState == FIRST)
	{
		printf("takeoff...........\n");
		desPosMAV.flx = realPosMAV.flx;
		desPosMAV.fly = realPosMAV.fly;
		desPosMAV.flz = 0.8f;
		desVeltMAV.flvz = 0.25f;

		RC_controller_output.channels[2] = 1400;
		desAttMAV.flyaw = 0;

		enTakeoffState = RUN;
	}
	else if(enTakeoffState == RUN)
	{
		if(enControlStateX == POS)
		{
			pidCrlMAV.pidPosx.flE = desPosMAV.flx - realPosMAV.flx;
			CalAbsPID(&(pidCrlMAV.pidPosx));
		}

		if(enControlStateY == POS)
		{
			pidCrlMAV.pidPosy.flE = desPosMAV.fly - realPosMAV.fly;
			CalAbsPID(&(pidCrlMAV.pidPosy));

		}
		RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidPosx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidPosy.flU * sin(realAttMAV.flyaw);
		RC_controller_output.channels[0] = 1500 - pidCrlMAV.pidPosx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidPosy.flU * cos(realAttMAV.flyaw);

		/*Control of z-axis*/
		if(fabs(realPosMAV.flz-desPosMAV.flz) < 0.1 && enControlStateZ == VELT)
		{
			printf("Postion close loop\n");
			enControlStateZ = POS;
			pidCrlMAV.pidPosz.flU = pidCrlMAV.pidVeltz.flU;
		}
		if(realVeltMAV.flvz > 0.15f && enControlStateZ == OPEN)
		{
			printf("Velt close loop\n");
			pidCrlMAV.pidVeltz.flU = RC_controller_output.channels[2] - 1100;
			enControlStateZ = VELT;
		}
		if(enControlStateZ == OPEN)
		{
			RC_controller_output.channels[2] = RC_controller_output.channels[2] + takeoff_speed;
		}
		else if(enControlStateZ == VELT)
		{
			pidCrlMAV.pidVeltz.flE = desVeltMAV.flvz - realVeltMAV.flvz;
			CalIncPID(&(pidCrlMAV.pidVeltz));
			RC_controller_output.channels[2] = 1100 + pidCrlMAV.pidVeltz.flU;
		}
		else if(enControlStateZ == POS)
		{
			pidCrlMAV.pidPosz.flE = desPosMAV.flz - realPosMAV.flz;
			CalIncPID(&(pidCrlMAV.pidPosz));
			RC_controller_output.channels[2] = 1100 + pidCrlMAV.pidPosz.flU;
		}


		pidCrlMAV.pidYaw.flE = desAttMAV.flyaw - realAttMAV.flyaw;
		CalAbsPID(&(pidCrlMAV.pidYaw));
		RC_controller_output.channels[3] = 1500 + pidCrlMAV.pidYaw.flU;


		RC_controller_output.channels[0]=ClipShort(RC_controller_output.channels[0],1400,1600);
		RC_controller_output.channels[1]=ClipShort(RC_controller_output.channels[1],1400,1600);
		RC_controller_output.channels[2]=ClipShort(RC_controller_output.channels[2],1100,1680);
		RC_controller_output.channels[3]=ClipShort(RC_controller_output.channels[3],1400,1600);

		//RC_controller_output.channels[0]=ClipShort(RC_controller_output.channels[0],1400,1600);
		//RC_controller_output.channels[1]=ClipShort(RC_controller_output.channels[1],1400,1600);
		//RC_controller_output.channels[2]=ClipShort(RC_controller_output.channels[2],1100,1650);
		//RC_controller_output.channels[3]=ClipShort(RC_controller_output.channels[3],1400,1600);
	}

}

void mav_land(void)
{
	RC_controller_output.channels[0] = 1500;
	RC_controller_output.channels[1] = 1500;
	RC_controller_output.channels[2] = RC_controller_output.channels[2]-land_speed;
	RC_controller_output.channels[3] = 1500;
	if(RC_controller_output.channels[2] < 1530)
	{
		RC_controller_output.channels[2] = 1530;
	}
}


void mav_takeoff_data_clear(void)
{
	pidCrlMAV.pidPosx.flESum = 0;
	pidCrlMAV.pidPosx.flPreE = 0;
	pidCrlMAV.pidPosx.flPrePreE = 0;
	pidCrlMAV.pidPosx.flU = 0;

	pidCrlMAV.pidPosy.flESum = 0;
	pidCrlMAV.pidPosy.flPreE = 0;
	pidCrlMAV.pidPosy.flPrePreE = 0;
	pidCrlMAV.pidPosy.flU = 0;

	pidCrlMAV.pidPosz.flESum = 0;
	pidCrlMAV.pidPosz.flPreE = 0;
	pidCrlMAV.pidPosz.flPrePreE = 0;
	pidCrlMAV.pidPosz.flU = 0;

	pidCrlMAV.pidRoll.flESum = 0;
	pidCrlMAV.pidRoll.flPreE = 0;
	pidCrlMAV.pidRoll.flPrePreE = 0;
	pidCrlMAV.pidRoll.flU = 0;

	pidCrlMAV.pidYaw.flESum = 0;
	pidCrlMAV.pidYaw.flPreE = 0;
	pidCrlMAV.pidYaw.flPrePreE = 0;
	pidCrlMAV.pidYaw.flU = 0;

	pidCrlMAV.pidPitch.flESum = 0;
	pidCrlMAV.pidPitch.flPreE = 0;
	pidCrlMAV.pidPitch.flPrePreE = 0;
	pidCrlMAV.pidPitch.flU = 0;

	pidCrlMAV.pidVeltx.flESum = 0;
	pidCrlMAV.pidVeltx.flPreE = 0;
	pidCrlMAV.pidVeltx.flPrePreE = 0;
	pidCrlMAV.pidVeltx.flU = 0;

	pidCrlMAV.pidVelty.flESum = 0;
	pidCrlMAV.pidVelty.flPreE = 0;
	pidCrlMAV.pidVelty.flPrePreE = 0;
	pidCrlMAV.pidVelty.flU = 0;

	pidCrlMAV.pidVeltz.flESum = 0;
	pidCrlMAV.pidVeltz.flPreE = 0;
	pidCrlMAV.pidVeltz.flPrePreE = 0;
	pidCrlMAV.pidVeltz.flU = 0;

	data_init();
}
/*
 * mav_action.cpp

 *
 *  Created on: Dec 6, 2016
 *      Author: yw
 */
#include <ros/ros.h>
#include <mav_client/mav_action.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PID_DATA_LEN_POS_VELT 29
#define PID_DATA_LEN_PENDULUM 6
short land_speed = 2;
short takeoff_speed = 2;
bool bpendulumvalid;

POS_MAV desPosMAV;
POS_MAV PendumStartPos;
VELT_MAV desVeltMAV;
ATT_MAV desAttMAV;
ATT_PENDULUM desAttPendulum = {0};

POS_MAV realPosMAV;
VELT_MAV realVeltMAV;
ATT_MAV realAttMAV;
ATT_PENDULUM realAttPendulum;

PID_CRL_MAV pidCrlMAV;
EN_RUN_STATE enTakeoffState = NEVER;
EN_CONTROL_STATE enControlStateX = POS_VELT;
EN_CONTROL_STATE enControlStateY = POS_VELT;
EN_CONTROL_STATE enControlStateZ = OPEN;
EN_MAV_STATE enMAVState = MAV_NEVER;

mavros_msgs::OverrideRCIn RC_controller_output;


void pid_data_pos_velt_init(void)
{
	FILE *fp;
	float pid_data[PID_DATA_LEN_POS_VELT];
	fp=fopen("/home/ubuntu/catkin_ws/src/mav_client/src/pid_pos_velt.txt","r+");
	if(!fp)
	{
		printf("data of pos and velt is not exist,check it\n");
	}
	else
	{
		printf("read PID data of pos and velt.......\n");
		for(int i=0;i<PID_DATA_LEN_POS_VELT;i++)
		{
			 fscanf(fp,"%f",&pid_data[i]);
			 printf("%f\t",pid_data[i]);
			 if(i%3 == 2)
			 {
				 printf("\n");
			 }
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
		printf("\npid data of pos and velt reload done\n");
	}
}

void pid_data_pendulum_init(void)
{
	FILE *fp;
	float pid_data[PID_DATA_LEN_PENDULUM];
	fp=fopen("/home/ubuntu/catkin_ws/src/mav_client/src/pid_pendulum.txt","r+");
	if(!fp)
	{
		printf("data of pos and velt is not exist,check it\n");
	}
	else
	{
		printf("read PID data of pendulum.......\n");
		for(int i=0;i<PID_DATA_LEN_PENDULUM;i++)
		{
			 fscanf(fp,"%f",&pid_data[i]);
			 printf("%f\t",pid_data[i]);
			 if(i%3 == 2)
			 {
				 printf("\n");
			 }
		}
		fclose(fp);
		pidCrlMAV.pidPendulumx.flP = pid_data[0];
		pidCrlMAV.pidPendulumx.flI = pid_data[1];
		pidCrlMAV.pidPendulumx.flD = pid_data[2];

		pidCrlMAV.pidPendulumy.flP = pid_data[3];
		pidCrlMAV.pidPendulumy.flI = pid_data[4];
		pidCrlMAV.pidPendulumy.flD = pid_data[5];

		printf("pid data of pendulum reload done\n");
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
		ROS_INFO("Takeoff...........");
		enControlStateZ = OPEN;
		desPosMAV.flx = realPosMAV.flx;
		desPosMAV.fly = realPosMAV.fly;
		desPosMAV.flz = 0.6f;
		desVeltMAV.flvz = 0.2f;
		desVeltMAV.flvx = 0.0f;
		desVeltMAV.flvy = 0.0f;

		RC_controller_output.channels[2] = 1400;
		desAttMAV.flyaw = 0;

		enTakeoffState = RUN;
	}
	else if(enTakeoffState == RUN)
	{
		/*Control of x-axis*/
		if(enControlStateX == POS_VELT)
		{
			pidCrlMAV.pidPosx.flE = desPosMAV.flx - realPosMAV.flx;
			CalAbsPID(&(pidCrlMAV.pidPosx));

			desVeltMAV.flvx = pidCrlMAV.pidPosx.flU;
			pidCrlMAV.pidVeltx.flE = desVeltMAV.flvx - realVeltMAV.flvx;
			CalAbsPID(&(pidCrlMAV.pidVeltx));
		}
		else if(enControlStateX == VELT)
		{
			pidCrlMAV.pidVeltx.flE = desVeltMAV.flvx - realVeltMAV.flvx;
			CalAbsPID(&(pidCrlMAV.pidVeltx));
		}
		else if(enControlStateX == POS)
		{
			pidCrlMAV.pidPosx.flE = desPosMAV.flx - realPosMAV.flx;
			CalAbsPID(&(pidCrlMAV.pidPosx));
		}

		/*Control of y-axis*/
		if(enControlStateY == POS_VELT)
		{
			pidCrlMAV.pidPosy.flE = desPosMAV.fly - realPosMAV.fly;
			CalAbsPID(&(pidCrlMAV.pidPosy));

			desVeltMAV.flvy = pidCrlMAV.pidPosy.flU;
			pidCrlMAV.pidVelty.flE = desVeltMAV.flvy - realVeltMAV.flvy;
			CalAbsPID(&(pidCrlMAV.pidVelty));

		}
		else if(enControlStateY == VELT)
		{
			pidCrlMAV.pidVelty.flE = desVeltMAV.flvy - realVeltMAV.flvy;
			CalAbsPID(&(pidCrlMAV.pidVelty));
		}
		else if(enControlStateY == POS)
		{
			pidCrlMAV.pidPosy.flE = desPosMAV.fly - realPosMAV.fly;
			CalAbsPID(&(pidCrlMAV.pidPosy));
		}

		/*****Output of x-y axis*****/
		if(enControlStateY == POS && enControlStateX == POS)
		{
			RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidPosx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidPosy.flU * sin(realAttMAV.flyaw);
			RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidPosx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidPosy.flU * cos(realAttMAV.flyaw);
		}
		else if((enControlStateY == VELT && enControlStateX == VELT) || (enControlStateY == POS_VELT && enControlStateX == POS_VELT))
		{
			RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidVeltx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidVelty.flU * sin(realAttMAV.flyaw);
			RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidVeltx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidVelty.flU * cos(realAttMAV.flyaw);
		}

		/*Control of z-axis*/
		if(fabs(realPosMAV.flz-desPosMAV.flz) < 0.1 &&fabs(realPosMAV.flx-desPosMAV.flx) < 0.1 &&fabs(realPosMAV.fly-desPosMAV.fly) < 0.1 && enControlStateZ == POS_VELT)
		{
			ROS_INFO("Enable Hover Mode");
			enMAVState = MAV_HOVER;
		}
		if(fabs(realPosMAV.flz-desPosMAV.flz) < 0.1 && enControlStateZ == VELT)
		{
			ROS_INFO("z-axis Postion close loop");
			enControlStateZ = POS_VELT;
		}
		if(realVeltMAV.flvz > 0.15f && enControlStateZ == OPEN)
		{
			ROS_INFO("z-axis Velt close loop");
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
		else if(enControlStateZ == POS_VELT)
		{
			pidCrlMAV.pidPosz.flE = desPosMAV.flz - realPosMAV.flz;
			CalAbsPID(&(pidCrlMAV.pidPosz));

			desVeltMAV.flvz = pidCrlMAV.pidPosz.flU;
			pidCrlMAV.pidVeltz.flE = desVeltMAV.flvz - realVeltMAV.flvz;
			CalIncPID(&(pidCrlMAV.pidVeltz));
			RC_controller_output.channels[2] = 1100 + pidCrlMAV.pidVeltz.flU;
		}


		pidCrlMAV.pidYaw.flE = desAttMAV.flyaw - realAttMAV.flyaw;
		CalAbsPID(&(pidCrlMAV.pidYaw));
		RC_controller_output.channels[3] = 1500 - pidCrlMAV.pidYaw.flU;


		RC_controller_output.channels[0]=ClipShort(RC_controller_output.channels[0],1350,1650);
		RC_controller_output.channels[1]=ClipShort(RC_controller_output.channels[1],1350,1650);
		RC_controller_output.channels[2]=ClipShort(RC_controller_output.channels[2],1100,1700);
		RC_controller_output.channels[3]=ClipShort(RC_controller_output.channels[3],1350,1650);


		//RC_controller_output.channels[0]=ClipShort(RC_controller_output.channels[0],1400,1600);
		//RC_controller_output.channels[1]=ClipShort(RC_controller_output.channels[1],1400,1600);
		//RC_controller_output.channels[2]=ClipShort(RC_controller_output.channels[2],1100,1650);
		//RC_controller_output.channels[3]=ClipShort(RC_controller_output.channels[3],1400,1600);
	}

}

void mav_hover(void)
{
	/*Control of x-axis*/
	if(fabs(realPosMAV.flx-desPosMAV.flx) < 0.1 && enControlStateX == VELT)
	{
		ROS_INFO("x-axis Postion close loop");
		enControlStateX = POS_VELT;
	}
	if(enControlStateX == POS_VELT)
	{
		pidCrlMAV.pidPosx.flE = desPosMAV.flx - realPosMAV.flx;
		CalAbsPlusPID(&(pidCrlMAV.pidPosx));

		desVeltMAV.flvx = pidCrlMAV.pidPosx.flU;
		pidCrlMAV.pidVeltx.flE = desVeltMAV.flvx - realVeltMAV.flvx;
		CalAbsPlusPID(&(pidCrlMAV.pidVeltx));
	}
	else if(enControlStateX == VELT)
	{
		pidCrlMAV.pidVeltx.flE = desVeltMAV.flvx - realVeltMAV.flvx;
		CalAbsPID(&(pidCrlMAV.pidVeltx));
	}
	else if(enControlStateX == POS)
	{
		pidCrlMAV.pidPosx.flE = desPosMAV.flx - realPosMAV.flx;
		CalAbsPID(&(pidCrlMAV.pidPosx));
	}

	/*Control of y-axis*/
	if(fabs(realPosMAV.fly-desPosMAV.fly) < 0.1 && enControlStateY == VELT)
	{
		ROS_INFO("y-axis Postion close loop");
		enControlStateY = POS_VELT;
	}
	if(enControlStateY == POS_VELT)
	{
		pidCrlMAV.pidPosy.flE = desPosMAV.fly - realPosMAV.fly;
		CalAbsPlusPID(&(pidCrlMAV.pidPosy));

		desVeltMAV.flvy = pidCrlMAV.pidPosy.flU;
		pidCrlMAV.pidVelty.flE = desVeltMAV.flvy - realVeltMAV.flvy;
		CalAbsPlusPID(&(pidCrlMAV.pidVelty));

	}
	else if(enControlStateY == VELT)
	{
		pidCrlMAV.pidVelty.flE = desVeltMAV.flvy - realVeltMAV.flvy;
		CalAbsPID(&(pidCrlMAV.pidVelty));
	}
	else if(enControlStateY == POS)
	{
		pidCrlMAV.pidPosy.flE = desPosMAV.fly - realPosMAV.fly;
		CalAbsPID(&(pidCrlMAV.pidPosy));
	}

	/*****Output of x-y axis*****/
	if(enControlStateY == POS && enControlStateX == POS)
	{
		RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidPosx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidPosy.flU * sin(realAttMAV.flyaw);
		RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidPosx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidPosy.flU * cos(realAttMAV.flyaw);
	}
	else if((enControlStateX == VELT || enControlStateX == POS_VELT) && (enControlStateY == POS_VELT ||enControlStateY == VELT))
	{
		RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidVeltx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidVelty.flU * sin(realAttMAV.flyaw);
		RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidVeltx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidVelty.flU * cos(realAttMAV.flyaw);
	}

	/*Control of z-axis*/
	if(fabs(realPosMAV.flz-desPosMAV.flz) < 0.1 && enControlStateZ == VELT)
	{
		ROS_INFO("z-axis Postion close loop");
		enControlStateZ = POS_VELT;
	}
	if(enControlStateZ == POS_VELT)
	{
		pidCrlMAV.pidPosz.flE = desPosMAV.flz - realPosMAV.flz;
		CalAbsPlusPID(&(pidCrlMAV.pidPosz));

		desVeltMAV.flvz = pidCrlMAV.pidPosz.flU;
		pidCrlMAV.pidVeltz.flE = desVeltMAV.flvz - realVeltMAV.flvz;
		CalIncPID(&(pidCrlMAV.pidVeltz));
		RC_controller_output.channels[2] = 1100 + pidCrlMAV.pidVeltz.flU;
	}
	else if(enControlStateZ == VELT)
	{
		pidCrlMAV.pidVeltz.flE = desVeltMAV.flvz - realVeltMAV.flvz;
		CalIncPID(&(pidCrlMAV.pidVeltz));
		RC_controller_output.channels[2] = 1100 + pidCrlMAV.pidVeltz.flU;
	}

	pidCrlMAV.pidYaw.flE = desAttMAV.flyaw - realAttMAV.flyaw;
	CalAbsPID(&(pidCrlMAV.pidYaw));
	RC_controller_output.channels[3] = 1500 - pidCrlMAV.pidYaw.flU;


	RC_controller_output.channels[0]=ClipShort(RC_controller_output.channels[0],1350,1650);
	RC_controller_output.channels[1]=ClipShort(RC_controller_output.channels[1],1350,1650);
	RC_controller_output.channels[2]=ClipShort(RC_controller_output.channels[2],1100,1700);
	RC_controller_output.channels[3]=ClipShort(RC_controller_output.channels[3],1350,1650);
}

void mav_pendulum(void)
{
	/*Control of x-axis*/
	if(enControlStateX == POS_VELT)
	{
		pidCrlMAV.pidPendulumx.flE = desAttPendulum.flx - realAttPendulum.flx;
		CalAbsPlusPID(&(pidCrlMAV.pidPendulumx));

		pidCrlMAV.pidPendulumx.flU = -pidCrlMAV.pidPendulumx.flU;
		desVeltMAV.flvx = pidCrlMAV.pidPendulumx.flU;
		pidCrlMAV.pidVeltx.flE = desVeltMAV.flvx - realVeltMAV.flvx;
		CalAbsPlusPID(&(pidCrlMAV.pidVeltx));
	}
	else if(enControlStateX == POS)
	{
		pidCrlMAV.pidPendulumx.flE = desAttPendulum.flx - realAttPendulum.flx;
		CalAbsPlusPID(&(pidCrlMAV.pidPendulumx));
		pidCrlMAV.pidPendulumx.flU = -pidCrlMAV.pidPendulumx.flU;
	}

	/*Control of y-axis*/
	if(enControlStateY == POS_VELT)
	{
		pidCrlMAV.pidPendulumy.flE = desAttPendulum.fly - realAttPendulum.fly;
		CalAbsPlusPID(&(pidCrlMAV.pidPendulumy));

		pidCrlMAV.pidPendulumy.flU = -pidCrlMAV.pidPendulumy.flU;
		desVeltMAV.flvy = pidCrlMAV.pidPendulumy.flU;
		pidCrlMAV.pidVelty.flE = desVeltMAV.flvy - realVeltMAV.flvy;
		CalAbsPlusPID(&(pidCrlMAV.pidVelty));

	}
	else if(enControlStateY == POS)
	{
		pidCrlMAV.pidPendulumy.flE = desAttPendulum.fly - realAttPendulum.fly;
		CalAbsPID(&(pidCrlMAV.pidPendulumy));
		pidCrlMAV.pidPendulumy.flU = -pidCrlMAV.pidPendulumy.flU;
	}

	/*****Output of x-y axis*****/
	if(enControlStateY == POS && enControlStateX == POS)
	{
		RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidPendulumx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidPendulumy.flU * sin(realAttMAV.flyaw);
		RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidPendulumx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidPendulumy.flU * cos(realAttMAV.flyaw);
	}
	else if(enControlStateY == POS_VELT && enControlStateX == POS_VELT)
	{
		RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidVeltx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidVelty.flU * sin(realAttMAV.flyaw);
		RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidVeltx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidVelty.flU * cos(realAttMAV.flyaw);
	}

	/*Control of z-axis*/
	if(enControlStateZ == POS_VELT)
	{
		pidCrlMAV.pidPosz.flE = desPosMAV.flz - realPosMAV.flz;
		CalAbsPlusPID(&(pidCrlMAV.pidPosz));

		desVeltMAV.flvz = pidCrlMAV.pidPosz.flU;
		pidCrlMAV.pidVeltz.flE = desVeltMAV.flvz - realVeltMAV.flvz;
		CalIncPID(&(pidCrlMAV.pidVeltz));
		RC_controller_output.channels[2] = 1100 + pidCrlMAV.pidVeltz.flU;
	}


	pidCrlMAV.pidYaw.flE = desAttMAV.flyaw - realAttMAV.flyaw;
	CalAbsPID(&(pidCrlMAV.pidYaw));
	RC_controller_output.channels[3] = 1500 - pidCrlMAV.pidYaw.flU;


	RC_controller_output.channels[0]=ClipShort(RC_controller_output.channels[0],1350,1650);
	RC_controller_output.channels[1]=ClipShort(RC_controller_output.channels[1],1350,1650);
	RC_controller_output.channels[2]=ClipShort(RC_controller_output.channels[2],1100,1700);
	RC_controller_output.channels[3]=ClipShort(RC_controller_output.channels[3],1350,1650);
}

void mav_land(void)
{
	RC_controller_output.channels[0] = 1500;
	RC_controller_output.channels[1] = 1500;
	RC_controller_output.channels[2] = RC_controller_output.channels[2]-land_speed;
	RC_controller_output.channels[3] = 1500;
	if(RC_controller_output.channels[2] < 1430)
	{
		RC_controller_output.channels[2] = 1430;
	}
}

void is_pendulum_start(void)
{
	if(bpendulumvalid == 1)
	{
		if(desAttPendulum.flx - realAttPendulum.flx > -PENDULUM_START_X/180*pi && desAttPendulum.flx - realAttPendulum.flx < PENDULUM_START_X/180*pi)
		{
			if(desAttPendulum.fly - realAttPendulum.fly > -PENDULUM_START_Y/180*pi && desAttPendulum.fly - realAttPendulum.fly < PENDULUM_START_Y/180*pi)
			{
				mav_pendulum_data_clear();
				ROS_INFO("Pendulum mode start");
				PendumStartPos.flx = realPosMAV.flx;
				PendumStartPos.fly = realPosMAV.fly;
				enMAVState = MAV_PENDULUM;
			}
		}
	}
}

bool is_pendulum_fail(void)
{
	if(bpendulumvalid == 1)
	{
		if(desAttPendulum.flx - realAttPendulum.flx < -PENDULUM_FAIL_X/180*pi || desAttPendulum.flx - realAttPendulum.flx > PENDULUM_FAIL_X/180*pi
				|| desAttPendulum.fly - realAttPendulum.fly < -PENDULUM_FAIL_Y/180*pi || desAttPendulum.fly - realAttPendulum.fly > PENDULUM_FAIL_Y/180*pi)
		{
			ROS_WARN("The angle of pendulum is too large:[%f\t%f]",realAttPendulum.flx/3.1415926*180,realAttPendulum.fly/3.1415926*180);
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		ROS_WARN("lost the pendulum");
		return 1;
	}
	if(realPosMAV.flx - PendumStartPos.flx > PENDULUM_LIMIT_X || realPosMAV.flx - PendumStartPos.flx < -PENDULUM_LIMIT_X)
	{
		ROS_WARN("Out of X-axis limit in pendulum mode:[%f\t%f]",realPosMAV.flx,PendumStartPos.flx);
		return 1;
	}
	if(realPosMAV.fly - PendumStartPos.fly > PENDULUM_LIMIT_Y || realPosMAV.fly - PendumStartPos.fly < -PENDULUM_LIMIT_Y)
	{
		ROS_WARN("Out of Y-axis limit in pendulum mode:[%f\t%f]",realPosMAV.fly,PendumStartPos.fly);
		return 1;
	}
}

void mav_pendulum_data_clear(void)
{
	pidCrlMAV.pidPendulumx.flESum = 0;
	pidCrlMAV.pidPendulumx.flPreE = 0;
	pidCrlMAV.pidPendulumx.flPrePreE = 0;
	pidCrlMAV.pidPendulumx.flU = 0;

	pidCrlMAV.pidPendulumy.flESum = 0;
	pidCrlMAV.pidPendulumy.flPreE = 0;
	pidCrlMAV.pidPendulumy.flPrePreE = 0;
	pidCrlMAV.pidPendulumy.flU = 0;


	pidCrlMAV.pidPosx.flESum = 0;
	pidCrlMAV.pidPosx.flPreE = 0;
	pidCrlMAV.pidPosx.flPrePreE = 0;
	pidCrlMAV.pidPosx.flU = 0;

/*	pidCrlMAV.pidPosy.flESum = 0;
	pidCrlMAV.pidPosy.flPreE = 0;
	pidCrlMAV.pidPosy.flPrePreE = 0;
	pidCrlMAV.pidPosy.flU = 0;*/


	for(int ii=0;ii<N_PRE;ii++)
	{
		pidCrlMAV.pidPosx.flNPreE[ii] = 0;
	//	pidCrlMAV.pidPosy.flNPreE[ii] = 0;

		pidCrlMAV.pidPendulumx.flNPreE[ii] = 0;
		pidCrlMAV.pidPendulumy.flNPreE[ii] = 0;
	}

	pid_data_pendulum_init();
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

	for(int ii=0;ii<N_PRE;ii++)
	{
		pidCrlMAV.pidPosx.flNPreE[ii] = 0;
		pidCrlMAV.pidPosy.flNPreE[ii] = 0;
		pidCrlMAV.pidPosz.flNPreE[ii] = 0;

		pidCrlMAV.pidVeltx.flNPreE[ii] = 0;
		pidCrlMAV.pidVelty.flNPreE[ii] = 0;
		pidCrlMAV.pidVeltz.flNPreE[ii] = 0;

		pidCrlMAV.pidRoll.flNPreE[ii] = 0;
		pidCrlMAV.pidYaw.flNPreE[ii] = 0;
		pidCrlMAV.pidPitch.flNPreE[ii] = 0;
	}

	pid_data_pendulum_init();
}

void mav_posloopx_data_clear(void)
{
	pidCrlMAV.pidPosx.flESum = 0;
	pidCrlMAV.pidPosx.flPreE = 0;
	pidCrlMAV.pidPosx.flPrePreE = 0;
	pidCrlMAV.pidPosx.flU = 0;
	for(int ii=0;ii<N_PRE;ii++)
	{
		pidCrlMAV.pidPosx.flNPreE[ii] = 0;
	}
}
void mav_posloopy_data_clear(void)
{
	pidCrlMAV.pidPosy.flESum = 0;
	pidCrlMAV.pidPosy.flPreE = 0;
	pidCrlMAV.pidPosy.flPrePreE = 0;
	pidCrlMAV.pidPosy.flU = 0;
	for(int ii=0;ii<N_PRE;ii++)
	{
		pidCrlMAV.pidPosy.flNPreE[ii] = 0;
	}
}

void mav_posloopz_data_clear(void)
{
	pidCrlMAV.pidPosz.flESum = 0;
	pidCrlMAV.pidPosz.flPreE = 0;
	pidCrlMAV.pidPosz.flPrePreE = 0;
	pidCrlMAV.pidPosz.flU = 0;
	for(int ii=0;ii<N_PRE;ii++)
	{
		pidCrlMAV.pidPosz.flNPreE[ii] = 0;
	}
}



void mav_pendulum_x(void)
{
	/*Control of x-axis*/
	if(enControlStateX == POS_VELT)
	{
		pidCrlMAV.pidPendulumx.flE = desAttPendulum.flx - realAttPendulum.flx;
		CalAbsPlusPID(&(pidCrlMAV.pidPendulumx));

		pidCrlMAV.pidPendulumx.flU = -pidCrlMAV.pidPendulumx.flU;
		desVeltMAV.flvx = pidCrlMAV.pidPendulumx.flU;
		pidCrlMAV.pidVeltx.flE = desVeltMAV.flvx - realVeltMAV.flvx;
		CalAbsPlusPID(&(pidCrlMAV.pidVeltx));
	}
	else if(enControlStateX == POS)
	{
		pidCrlMAV.pidPendulumx.flE = desAttPendulum.flx - realAttPendulum.flx;
		CalAbsPlusPID(&(pidCrlMAV.pidPendulumx));
		pidCrlMAV.pidPendulumx.flU = -pidCrlMAV.pidPendulumx.flU;
	}

	/*Control of y-axis*/
	if(enControlStateY == POS_VELT)
	{
		pidCrlMAV.pidPosy.flE = desPosMAV.fly - realPosMAV.fly;
		CalAbsPlusPID(&(pidCrlMAV.pidPosy));

		desVeltMAV.flvy = pidCrlMAV.pidPosy.flU;
		pidCrlMAV.pidVelty.flE = desVeltMAV.flvy - realVeltMAV.flvy;
		CalAbsPlusPID(&(pidCrlMAV.pidVelty));

	}
	else if(enControlStateY == VELT)
	{
		pidCrlMAV.pidVelty.flE = desVeltMAV.flvy - realVeltMAV.flvy;
		CalAbsPID(&(pidCrlMAV.pidVelty));
	}
	else if(enControlStateY == POS)
	{
		pidCrlMAV.pidPosy.flE = desPosMAV.fly - realPosMAV.fly;
		CalAbsPID(&(pidCrlMAV.pidPosy));
	}

	/*****Output of x-y axis*****/
	if(enControlStateY == POS && enControlStateX == POS)
	{
		RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidPendulumx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidPendulumy.flU * sin(realAttMAV.flyaw);
		RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidPendulumx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidPendulumy.flU * cos(realAttMAV.flyaw);
	}
	else if(enControlStateY == POS_VELT && enControlStateX == POS_VELT)
	{
		RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidVeltx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidVelty.flU * sin(realAttMAV.flyaw);
		RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidVeltx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidVelty.flU * cos(realAttMAV.flyaw);
	}

	/*Control of z-axis*/
	if(enControlStateZ == POS_VELT)
	{
		pidCrlMAV.pidPosz.flE = desPosMAV.flz - realPosMAV.flz;
		CalAbsPlusPID(&(pidCrlMAV.pidPosz));

		desVeltMAV.flvz = pidCrlMAV.pidPosz.flU;
		pidCrlMAV.pidVeltz.flE = desVeltMAV.flvz - realVeltMAV.flvz;
		CalIncPID(&(pidCrlMAV.pidVeltz));
		RC_controller_output.channels[2] = 1100 + pidCrlMAV.pidVeltz.flU;
	}


	pidCrlMAV.pidYaw.flE = desAttMAV.flyaw - realAttMAV.flyaw;
	CalAbsPID(&(pidCrlMAV.pidYaw));
	RC_controller_output.channels[3] = 1500 - pidCrlMAV.pidYaw.flU;


	RC_controller_output.channels[0]=ClipShort(RC_controller_output.channels[0],1350,1650);
	RC_controller_output.channels[1]=ClipShort(RC_controller_output.channels[1],1350,1650);
	RC_controller_output.channels[2]=ClipShort(RC_controller_output.channels[2],1100,1700);
	RC_controller_output.channels[3]=ClipShort(RC_controller_output.channels[3],1350,1650);
}



void mav_pendulum_y(void)
{
	/*Control of x-axis*/
	if(enControlStateX == POS_VELT)
	{
		pidCrlMAV.pidPosx.flE = desPosMAV.flx - realPosMAV.flx;
		CalAbsPlusPID(&(pidCrlMAV.pidPosx));

		desVeltMAV.flvx = pidCrlMAV.pidPosx.flU;
		pidCrlMAV.pidVeltx.flE = desVeltMAV.flvx - realVeltMAV.flvx;
		CalAbsPlusPID(&(pidCrlMAV.pidVeltx));
	}
	else if(enControlStateX == VELT)
	{
		pidCrlMAV.pidVeltx.flE = desVeltMAV.flvx - realVeltMAV.flvx;
		CalAbsPID(&(pidCrlMAV.pidVeltx));
	}
	else if(enControlStateX == POS)
	{
		pidCrlMAV.pidPosx.flE = desPosMAV.flx - realPosMAV.flx;
		CalAbsPID(&(pidCrlMAV.pidPosx));
	}

	/*Control of y-axis*/
	if(enControlStateY == POS_VELT)
	{
		pidCrlMAV.pidPendulumy.flE = desAttPendulum.fly - realAttPendulum.fly;
		CalAbsPlusPID(&(pidCrlMAV.pidPendulumy));

		pidCrlMAV.pidPendulumy.flU = -pidCrlMAV.pidPendulumy.flU;
		desVeltMAV.flvy = pidCrlMAV.pidPendulumy.flU;
		pidCrlMAV.pidVelty.flE = desVeltMAV.flvy - realVeltMAV.flvy;
		CalAbsPlusPID(&(pidCrlMAV.pidVelty));

	}
	else if(enControlStateY == POS)
	{
		pidCrlMAV.pidPendulumy.flE = desAttPendulum.fly - realAttPendulum.fly;
		CalAbsPID(&(pidCrlMAV.pidPendulumy));
		pidCrlMAV.pidPendulumy.flU = -pidCrlMAV.pidPendulumy.flU;
	}

	/*****Output of x-y axis*****/
	if(enControlStateY == POS && enControlStateX == POS)
	{
		RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidPendulumx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidPendulumy.flU * sin(realAttMAV.flyaw);
		RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidPendulumx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidPendulumy.flU * cos(realAttMAV.flyaw);
	}
	else if(enControlStateY == POS_VELT && enControlStateX == POS_VELT)
	{
		RC_controller_output.channels[1] = 1500 + pidCrlMAV.pidVeltx.flU * cos(realAttMAV.flyaw) + pidCrlMAV.pidVelty.flU * sin(realAttMAV.flyaw);
		RC_controller_output.channels[0] = 1500 + pidCrlMAV.pidVeltx.flU * sin(realAttMAV.flyaw) - pidCrlMAV.pidVelty.flU * cos(realAttMAV.flyaw);
	}

	/*Control of z-axis*/
	if(enControlStateZ == POS_VELT)
	{
		pidCrlMAV.pidPosz.flE = desPosMAV.flz - realPosMAV.flz;
		CalAbsPlusPID(&(pidCrlMAV.pidPosz));

		desVeltMAV.flvz = pidCrlMAV.pidPosz.flU;
		pidCrlMAV.pidVeltz.flE = desVeltMAV.flvz - realVeltMAV.flvz;
		CalIncPID(&(pidCrlMAV.pidVeltz));
		RC_controller_output.channels[2] = 1100 + pidCrlMAV.pidVeltz.flU;
	}


	pidCrlMAV.pidYaw.flE = desAttMAV.flyaw - realAttMAV.flyaw;
	CalAbsPID(&(pidCrlMAV.pidYaw));
	RC_controller_output.channels[3] = 1500 - pidCrlMAV.pidYaw.flU;


	RC_controller_output.channels[0]=ClipShort(RC_controller_output.channels[0],1350,1650);
	RC_controller_output.channels[1]=ClipShort(RC_controller_output.channels[1],1350,1650);
	RC_controller_output.channels[2]=ClipShort(RC_controller_output.channels[2],1100,1700);
	RC_controller_output.channels[3]=ClipShort(RC_controller_output.channels[3],1350,1650);
}




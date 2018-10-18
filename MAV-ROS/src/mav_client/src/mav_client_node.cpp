#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <mav_client/lib_trajectory_planning.h>
#include <actionlib/client/terminal_state.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <network_client/Optitrack.h>
#include <network_client/Optitrack_data.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mav_client/lib_pid_controller.h>
#include <mav_client/mav_action.h>
#include <mav_client/Debug_data.h>
#include <mavros_msgs/BatteryStatus.h>


typedef enum
{
	SYS_START,
	SYS_RUN
}EN_SYS_STATE;
EN_SYS_STATE enSysState = SYS_START;

ros::Subscriber n_network_cmd_sub;
ros::Subscriber n_network_data_sub;
ros::Subscriber mav_battery_sub;
ros::Publisher mav_override_pub;
ros::Publisher mav_pid_controller_pub;
ros::Publisher mav_debug_data_pub;

double SysTime;
double DesTime_1ms;
double DesTime_10ms;
double DesTime_20ms;
double SysTimePrv_1ms;
double SysTimePrv_10ms;
double SysTimePrv_20ms;

int TimeOut_Net;
bool TimeOut_Start = 0;

double Timer_10ms;
double Timer_20ms;

mavros_msgs::OverrideRCIn RC_cmd;
network_client::Optitrack_data Optitrack_data;
mav_client::Debug_data Debug_data_out;
network_client::Optitrack Mav_net_cmd;
mavros_msgs::BatteryStatus Mav_battery_status;



int timecount;
int maintimecount = 0;

char mav_lock_flag = 1; //1--lock 0--unlock

void n_network_cmd_callback(network_client::Optitrack net_cmd);
void n_network_data_callback(network_client::Optitrack_data net_data);
void mav_battery_callback(mavros_msgs::BatteryStatus battery_status);

void mav_override_pub_cb(void);
void mav_pid_controller_pub_cb(void);
void mav_debug_data_pub_cb(void);

void rc_init_mav(void);
void rc_unlock_mav(void);
void rc_lock_mav(void);

static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+------< Main menu > -----+\n");
	printf("| [a] Init RC Output      |\n");
	printf("| [b] Unlock MAV          |\n");
	printf("| [c] Lock MAV            |\n");
	printf("| [d] Takeoff             |\n");
	printf("| [e] Landing             |\n");
	printf("| [f] reload pid data     |\n");
    printf("+-------------------------+\n");
    printf("use `rostopic echo` to query drone status\r\n");
}
int main(int argc, char **argv)
{

	ros::init(argc, argv, "mav_client");
	ROS_INFO("mav_service_client start");

	ros::NodeHandle mav_client_nh;

	pid_data_pos_velt_init();
	pid_data_pendulum_init();
	Display_Main_Menu();
	rc_init_mav();

	RC_controller_output.channels[2] = 1100;

    n_network_cmd_sub = mav_client_nh.subscribe("/network_client/network_cmd", 1, n_network_cmd_callback);
    n_network_data_sub = mav_client_nh.subscribe("/network_client/network_optitrack_data", 1, n_network_data_callback);
    mav_battery_sub = mav_client_nh.subscribe("/mavros/battery", 1, mav_battery_callback);
	mav_override_pub = mav_client_nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 100);	
	mav_pid_controller_pub = mav_client_nh.advertise<mavros_msgs::OverrideRCIn>("/mav_client/rc/pid", 100);
	mav_debug_data_pub = mav_client_nh.advertise<mav_client::Debug_data>("/mav_client/debug", 100);

	while(ros::ok())
	{
		ros::spinOnce();
		SysTime = ros::Time::now().toSec()*1000; //ms
		if(enSysState == SYS_START)
		{
			DesTime_1ms = SysTime;
			DesTime_10ms = SysTime;
			DesTime_20ms = SysTime;
			enSysState = SYS_RUN;
		}
		if(SysTime - DesTime_10ms > 100000)
		{
			enSysState = SYS_START;
			ROS_ERROR("System time is error,reset!");
		}

		if(SysTime >= DesTime_1ms)
		{
			TimeOut_Net++;
			if(TimeOut_Net%100 == 0 && TimeOut_Start == 1 && Optitrack_data.network_health_flag == 1)
			{
				ROS_WARN("Timeout of Network:%d",TimeOut_Net);
			}
			if(TimeOut_Net >= NET_TIMEOUT && TimeOut_Start == 1 && enMAVState == MAV_TAKEOFF && enMAVState == MAV_TAKEOFF
			&&enMAVState == MAV_TAKEOFF && Optitrack_data.network_health_flag == 1)
			{
				ROS_WARN("Timeout of Network,start land");
				enMAVState = MAV_LAND;
				timecount = 0;
				enTakeoffState = NEVER;
				TimeOut_Start = 0;
			}
			DesTime_1ms = DesTime_1ms + 1;
		}
		if(SysTime >= DesTime_10ms)
		{
		/*	if(Tra_Plan_X.enTraState == TRA_NEVER)
			{
				Tra_Plan_X.flHz = 100;
				Tra_Plan_X.flAccUp = 0.25;
				Tra_Plan_X.flAccDown = 0.25;
				Tra_Plan_X.flVeltMax = 0.25;
				Tra_Plan_X.flStartPos = 0.613873;
				Tra_Plan_X.flEndPos = 0;
				Tra_Plan_X.enTraState = TRA_FIRST;
			}*/
			if(Tra_Plan_X.enTraState != TRA_NEVER)
			{
				CalTraPlanning(&Tra_Plan_X);
				desPosMAV.flx = Tra_Plan_X.flDesPosOut;
				pidCrlMAV.pidPosx.chEflag = 0;
			}
			else
			{
				pidCrlMAV.pidPosx.chEflag = 1;
			}
			if(Tra_Plan_Y.enTraState != TRA_NEVER)
			{
				CalTraPlanning(&Tra_Plan_Y);
				desPosMAV.fly = Tra_Plan_Y.flDesPosOut;
				pidCrlMAV.pidPosy.chEflag = 0;
			}
			else
			{
				pidCrlMAV.pidPosy.chEflag = 1;
			}
			if(Tra_Plan_Z.enTraState != TRA_NEVER)
			{
				CalTraPlanning(&Tra_Plan_Z);
				desPosMAV.flz = Tra_Plan_Z.flDesPosOut;
				//pidCrlMAV.pidPosz.chEflag = 0;
			}
			/*else
			{
				pidCrlMAV.pidPosz.chEflag = 1;
			}*/


			if(enMAVState != MAV_PENDULUM)
			{
				is_pendulum_start();
			}
			//printf("time is: %fms\n",ros::Time::now().toSec()*1000 - SysTime);
			Timer_10ms = SysTime -SysTimePrv_10ms;
			SysTimePrv_10ms = SysTime;
			if(enMAVState == MAV_LAND)
			{
				Tra_Plan_X.enTraState = TRA_NEVER;
				Tra_Plan_Y.enTraState = TRA_NEVER;
				Tra_Plan_Z.enTraState = TRA_NEVER;
				timecount++;
				mav_land();
				RC_cmd.channels[0] = RC_controller_output.channels[0];
				RC_cmd.channels[1] = RC_controller_output.channels[1];
				RC_cmd.channels[2] = RC_controller_output.channels[2];
				RC_cmd.channels[3] = RC_controller_output.channels[3];
				if(Optitrack_data.health_flag == 1)
				{
					if(realPosMAV.flz < 0.2)
					{
						enMAVState = MAV_LOCK;
						ROS_INFO("locking..........");
						rc_lock_mav();
						timecount = 0;
					}
				}
				else if(Optitrack_data.health_flag == 0)
				{
					//printf("%d\n",timecount);
					if(timecount >= 900)
					{
						enMAVState = MAV_LOCK;
						ROS_INFO("locking..........");
						rc_lock_mav();
						timecount = 0;
					}
				}
			}
			if(enMAVState == MAV_TAKEOFF)
			{
				mav_takeoff();
				if(Optitrack_data.health_flag == 0)
				{
					enMAVState = MAV_LAND;
					timecount = 0;
					ROS_WARN("Lose MAV,start land");
					enTakeoffState = NEVER;
				}
				RC_cmd.channels[0] = RC_controller_output.channels[0];
				RC_cmd.channels[1] = RC_controller_output.channels[1];
				RC_cmd.channels[2] = RC_controller_output.channels[2];
				RC_cmd.channels[3] = RC_controller_output.channels[3];
			}

			if(enMAVState == MAV_HOVER)
			{
				mav_hover();
				if(Optitrack_data.health_flag == 0)
				{
					enMAVState = MAV_LAND;
					timecount = 0;
					ROS_WARN("Lose MAV,start land");
					enTakeoffState = NEVER;
				}
				RC_cmd.channels[0] = RC_controller_output.channels[0];
				RC_cmd.channels[1] = RC_controller_output.channels[1];
				RC_cmd.channels[2] = RC_controller_output.channels[2];
				RC_cmd.channels[3] = RC_controller_output.channels[3];
			}
			if(enMAVState == MAV_PENDULUM)
			{
				mav_pendulum_x();
				if(is_pendulum_fail() == 1)
				{
					mav_pendulum_data_clear();
					ROS_INFO("hover mode start");
					desPosMAV.flx = realPosMAV.flx;
					//desPosMAV.fly = realPosMAV.fly;
					enMAVState = MAV_HOVER;
				}
				if(Optitrack_data.health_flag == 0)
				{
					enMAVState = MAV_LAND;
					timecount = 0;
					ROS_WARN("Lose MAV,start land");
					enTakeoffState = NEVER;
				}
				RC_cmd.channels[0] = RC_controller_output.channels[0];
				RC_cmd.channels[1] = RC_controller_output.channels[1];
				RC_cmd.channels[2] = RC_controller_output.channels[2];
				RC_cmd.channels[3] = RC_controller_output.channels[3];
			}
			mav_debug_data_pub_cb();
			mav_pid_controller_pub_cb();

			if(enMAVState == MAV_UNLOCK)
			{
				timecount++;
				if(timecount > 600)
				{
					enMAVState = MAV_READY;
					mav_lock_flag = 0;
					ROS_INFO("unlock OK");
					rc_init_mav();
				}
			}
			else if(enMAVState == MAV_LOCK)
			{
				timecount++;
				if(timecount > 600)
				{
					enMAVState = MAV_NEVER;
					enTakeoffState = NEVER;
					mav_lock_flag = 1;
					ROS_INFO("lock OK");
					rc_init_mav();
				}
			}
			DesTime_10ms = DesTime_10ms + 10;
		}
		/*pub of override*/
		if(SysTime >= DesTime_20ms)
		{
			Timer_20ms = SysTime -SysTimePrv_20ms;
			SysTimePrv_20ms = SysTime;
			mav_override_pub_cb();
			DesTime_20ms = DesTime_20ms + 20;
		}
	}
}
void mav_battery_callback(mavros_msgs::BatteryStatus battery_status)
{
	Mav_battery_status = battery_status;
}
void n_network_data_callback(network_client::Optitrack_data net_data)
{
	Optitrack_data = net_data;
    if(net_data.network_health_flag == 1)
    {
    	TimeOut_Net = 0; //reset time out
    	TimeOut_Start = 1;
    	realPosMAV.flx = net_data.posx;
    	realPosMAV.fly = net_data.posy;
    	realPosMAV.flz = net_data.posz;

    	//realVeltMAV.flvx = net_data.veltx;
    	//realVeltMAV.flvy = net_data.velty;
    	//realVeltMAV.flvz = net_data.veltz;

    	realVeltMAV.flvx = net_data.filterveltx;
    	realVeltMAV.flvy = net_data.filtervelty;
    	realVeltMAV.flvz = net_data.filterveltz;

    	realAttPendulum.flx = net_data.pendulum_attx;
    	realAttPendulum.fly = net_data.pendulum_atty;
    	bpendulumvalid = net_data.pendulum_health_flag;

    	realAttMAV.flroll = net_data.roll;
    	realAttMAV.flyaw = net_data.yaw;
    	realAttMAV.flpitch = net_data.pitch;
    }
    else
    {
    	if(enMAVState != MAV_NEVER)
    	{
			enMAVState = MAV_LAND;
			timecount = 0;
			ROS_WARN("Network disconnect,start land");
			enTakeoffState = NEVER;
    	}
    }
}

void n_network_cmd_callback(network_client::Optitrack net_cmd)
{
		Mav_net_cmd = net_cmd;
        Display_Main_Menu();
        if(net_cmd.health_flag == 1)
        {
            if(net_cmd.cmdtype == 0x02) //Command
            {
            	ROS_INFO("Cmd is:%c",net_cmd.control_cmd);
                switch(net_cmd.control_cmd)
                {
                  case 'a':             
                	  ROS_INFO("MAV SDK version:0.0.2");
                	  rc_init_mav();
                    break;
                  case 'b':
                	  enMAVState = MAV_UNLOCK;
                	  timecount = 0;
                	  ROS_INFO("unlocking........");
                	  rc_unlock_mav();
                    break;
                  case 'c':
                	  enMAVState = MAV_LOCK;
                	  timecount = 0;
                	  ROS_INFO("locking..........");
                	  rc_lock_mav();
                    break;
                  case 'd':
                    /* take off */
                	  if(enMAVState == MAV_READY)
                	  {
                		  mav_takeoff_data_clear();
                		  enMAVState = MAV_TAKEOFF;
                		  Tra_Plan_X.enTraState = TRA_NEVER;
                		  Tra_Plan_Y.enTraState = TRA_NEVER;
                		  Tra_Plan_Z.enTraState = TRA_NEVER;
                		  ROS_INFO("Start takeoff");
                	  }
                	  else
                	  {
                		  ROS_INFO("Error,MAV is locked,Check it!");
                	  }
                    break;
                  case 'e':
                    /* landing*/
                	  enMAVState = MAV_LAND;
                	  timecount = 0;
                	  ROS_INFO("Start land");
                    break;
                  case 'n':
                    /* reload pid data*/
                	  ROS_INFO("reload pid data of pos and velt");
                	  pid_data_pos_velt_init();
                    break;
                  case 'm':
                	  ROS_INFO("reload pid data of pendulum");
                	  pid_data_pendulum_init();
                    break;
                  case 'z':
                    break;
                  default:
                    break;
                }
            }
            else if(net_cmd.cmdtype == 0x01) //Velt
            {

            }
            else if(net_cmd.cmdtype == 0x03) //Aittitude
            {

            }
            else if(net_cmd.cmdtype == 0x04) //protect
            {
            	if(net_cmd.x < 1000)
            	{
            		ROS_INFO("set des positon is [%f\t%f\t%f]",net_cmd.x,desPosMAV.fly,desPosMAV.flz);
            		Tra_Plan_X.flHz = 100;
            		Tra_Plan_X.flAccUp = 0.25;
            		Tra_Plan_X.flAccDown = 0.125;
            		Tra_Plan_X.flVeltMax = 1.0;
            		Tra_Plan_X.flStartPos = realPosMAV.flx;
            		Tra_Plan_X.flEndPos = net_cmd.x;
            		Tra_Plan_X.enTraState = TRA_FIRST;
            		ROS_INFO("X-axis");
            	}
            	if(net_cmd.y < 1000)
            	{
            		ROS_INFO("set des positon is [%f\t%f\t%f]",desPosMAV.flx,net_cmd.y,desPosMAV.flz);
            		Tra_Plan_Y.flHz = 100;
					Tra_Plan_Y.flAccUp = 0.25;
					Tra_Plan_Y.flAccDown = 0.125;
					Tra_Plan_Y.flVeltMax = 1.0;
					Tra_Plan_Y.flStartPos = realPosMAV.fly;
					Tra_Plan_Y.flEndPos = net_cmd.y;
					Tra_Plan_Y.enTraState = TRA_FIRST;
					ROS_INFO("Y-axis");

            	}
            	if(net_cmd.z < 1000)
            	{
            		ROS_INFO("set des positon is [%f\t%f\t%f]",desPosMAV.flx,desPosMAV.fly,net_cmd.z);
            		Tra_Plan_Z.flHz = 100;
					Tra_Plan_Z.flAccUp = 0.25;
					Tra_Plan_Z.flAccDown = 0.125;
					Tra_Plan_Z.flVeltMax = 0.5;
					Tra_Plan_Z.flStartPos = realPosMAV.flz;
					Tra_Plan_Z.flEndPos = net_cmd.z;
					Tra_Plan_Z.enTraState = TRA_FIRST;
					ROS_INFO("Z-axis");

            	/*	desVeltMAV.flvz = SgnFloat(net_cmd.z - realPosMAV.flz)*0.2f;
            		enControlStateZ = VELT;
            		mav_posloopz_data_clear();*/
            	}
            }
        }
        else
        {

        }
}

void mav_override_pub_cb(void) 
{
	mavros_msgs::OverrideRCIn rmsg;

	rmsg.channels[0] = RC_cmd.channels[0];
	rmsg.channels[1] = RC_cmd.channels[1];
	rmsg.channels[2] = RC_cmd.channels[2];
	rmsg.channels[3] = RC_cmd.channels[3];
	rmsg.channels[4] = RC_cmd.channels[4];
	rmsg.channels[5] = RC_cmd.channels[5];
	rmsg.channels[6] = RC_cmd.channels[6];
	rmsg.channels[7] = RC_cmd.channels[7];

	mav_override_pub.publish(rmsg);
};

void mav_pid_controller_pub_cb(void)
{
	mavros_msgs::OverrideRCIn rmsg;

	rmsg.channels[0] = RC_cmd.channels[0];
	rmsg.channels[1] = RC_cmd.channels[1];
	rmsg.channels[2] = RC_cmd.channels[2];
	rmsg.channels[3] = RC_cmd.channels[3];
	rmsg.channels[4] = RC_cmd.channels[4];
	rmsg.channels[5] = RC_cmd.channels[5];
	rmsg.channels[6] = RC_cmd.channels[6];
	rmsg.channels[7] = RC_cmd.channels[7];

	mav_pid_controller_pub.publish(rmsg);
};

void mav_debug_data_pub_cb(void)
{
	mav_client::Debug_data msg_pub;

	msg_pub.header.frame_id = "mav_debug";
	msg_pub.header.stamp    = ros::Time::now();

	msg_pub.opt_pos[0] = realPosMAV.flx;
	msg_pub.opt_pos[1] = realPosMAV.fly;
	msg_pub.opt_pos[2] = realPosMAV.flz;

	msg_pub.opt_velt[0] = Optitrack_data.veltx;
	msg_pub.opt_velt[1] = Optitrack_data.velty;
	msg_pub.opt_velt[2] = Optitrack_data.veltz;

	msg_pub.opt_attitude_pendulum[0] = Optitrack_data.pendulum_attx;
	msg_pub.opt_attitude_pendulum[1] = Optitrack_data.pendulum_atty;

	msg_pub.filter_velt[0] = Optitrack_data.filterveltx;
	msg_pub.filter_velt[1] = Optitrack_data.filtervelty;
	msg_pub.filter_velt[2] = Optitrack_data.filterveltz;

	msg_pub.opt_attitude[0] = realAttMAV.flroll;
	msg_pub.opt_attitude[1] = realAttMAV.flyaw;
	msg_pub.opt_attitude[2] = realAttMAV.flpitch;

	msg_pub.des_pos[0] = desPosMAV.flx;
	msg_pub.des_pos[1] = desPosMAV.fly;
	msg_pub.des_pos[2] = desPosMAV.flz;

	msg_pub.des_velt[0] = desVeltMAV.flvx;
	msg_pub.des_velt[1] = desVeltMAV.flvy;
	msg_pub.des_velt[2] = desVeltMAV.flvz;

	msg_pub.des_attitude_pendulum[0] = desAttPendulum.flx;
	msg_pub.des_attitude_pendulum[1] = desAttPendulum.fly;

	msg_pub.des_attitude[0] = desAttMAV.flroll;
	msg_pub.des_attitude[1] = desAttMAV.flyaw;
	msg_pub.des_attitude[2] = desAttMAV.flpitch;

	msg_pub.err_pos[0] = pidCrlMAV.pidPosx.flE;
	msg_pub.err_pos[1] = pidCrlMAV.pidPosy.flE;
	msg_pub.err_pos[2] = pidCrlMAV.pidPosz.flE;

	msg_pub.err_velt[0] = pidCrlMAV.pidVeltx.flE;
	msg_pub.err_velt[1] = pidCrlMAV.pidVelty.flE;
	msg_pub.err_velt[2] = pidCrlMAV.pidVeltz.flE;

	msg_pub.err_attitude_pendulum[0] = pidCrlMAV.pidPendulumx.flE;
	msg_pub.err_attitude_pendulum[1] = pidCrlMAV.pidPendulumy.flE;

	msg_pub.err_attitude[0] = pidCrlMAV.pidRoll.flE;
	msg_pub.err_attitude[1] = pidCrlMAV.pidYaw.flE;
	msg_pub.err_attitude[2] = pidCrlMAV.pidPitch.flE;

	msg_pub.u_pos[0] = pidCrlMAV.pidPosx.flU;
	msg_pub.u_pos[1] = pidCrlMAV.pidPosy.flU;
	msg_pub.u_pos[2] = pidCrlMAV.pidPosz.flU;

	msg_pub.u_velt[0] = pidCrlMAV.pidVeltx.flU;
	msg_pub.u_velt[1] = pidCrlMAV.pidVelty.flU;
	msg_pub.u_velt[2] = pidCrlMAV.pidVeltz.flU;

	msg_pub.u_attitude_pendulum[0] = pidCrlMAV.pidPendulumx.flU;
	msg_pub.u_attitude_pendulum[1] = pidCrlMAV.pidPendulumy.flU;

	msg_pub.u_attitude[0] = pidCrlMAV.pidRoll.flU;
	msg_pub.u_attitude[1] = pidCrlMAV.pidYaw.flU;
	msg_pub.u_attitude[2] = pidCrlMAV.pidPitch.flU;

	msg_pub.network_health_flag = Optitrack_data.network_health_flag;
	msg_pub.opt_health_flag = Optitrack_data.health_flag;
	msg_pub.rc2mav = RC_cmd.channels;
	msg_pub.rc_controller = RC_controller_output.channels;
	msg_pub.msgid = Mav_net_cmd.control_cmd;
	msg_pub.mavstate = enMAVState;
	msg_pub.takeoffstate = enTakeoffState;

	msg_pub.timer_10ms = Timer_10ms;
	msg_pub.timer_20ms = Timer_20ms;
	msg_pub.net_delay = Optitrack_data.delay;
	msg_pub.lost_data = Optitrack_data.lost_data;
	msg_pub.pendulum_health_flag = Optitrack_data.pendulum_health_flag;

	msg_pub.pendulum[0] = Optitrack_data.pendulum_posx;
	msg_pub.pendulum[1] = Optitrack_data.pendulum_posy;
	msg_pub.pendulum[2] = Optitrack_data.pendulum_posz;
	msg_pub.pendulum[3] = Optitrack_data.pendulum_attx;
	msg_pub.pendulum[4] = Optitrack_data.pendulum_atty;

	msg_pub.test_1[0] = pidCrlMAV.pidPosx.flDE;
	msg_pub.test_1[1] = pidCrlMAV.pidPosy.flDE;
	msg_pub.test_1[2] = 0;

	msg_pub.test_2[0] = pidCrlMAV.pidVeltx.flDE;
	msg_pub.test_2[1] = pidCrlMAV.pidVelty.flDE;
	msg_pub.test_2[2] = 0;

	msg_pub.test_3[0] = Mav_battery_status.current;
	msg_pub.test_3[1] = Mav_battery_status.voltage;

	msg_pub.esum_pos[0] = pidCrlMAV.pidPosx.flESum * pidCrlMAV.pidPosx.flI;
	msg_pub.esum_pos[1] = pidCrlMAV.pidPosy.flESum * pidCrlMAV.pidPosy.flI;
	msg_pub.esum_pos[2] = pidCrlMAV.pidPosz.flESum * pidCrlMAV.pidPosz.flI;

	msg_pub.esum_velt[0] = pidCrlMAV.pidVeltx.flESum * pidCrlMAV.pidVeltx.flI;
	msg_pub.esum_velt[1] = pidCrlMAV.pidVelty.flESum * pidCrlMAV.pidVelty.flI;
	msg_pub.esum_velt[2] = pidCrlMAV.pidVeltz.flESum * pidCrlMAV.pidVeltz.flI;

	mav_debug_data_pub.publish(msg_pub);
}

void rc_unlock_mav(void)
{
	RC_cmd.channels[0] = 1500; //CH1
 	RC_cmd.channels[1] = 1500; //CH2
	RC_cmd.channels[2] = 1100; //CH3
	RC_cmd.channels[3] = 1100; //CH4
	RC_cmd.channels[4] = 1900; //CH5
	RC_cmd.channels[5] = 1900; //CH6
	RC_cmd.channels[6] = 1900; //CH7
	RC_cmd.channels[7] = 1900; //CH8
}

void rc_lock_mav(void)
{
	RC_cmd.channels[0] = 1500; //CH1
 	RC_cmd.channels[1] = 1500; //CH2
	RC_cmd.channels[2] = 1100; //CH3
	RC_cmd.channels[3] = 1900; //CH4
	RC_cmd.channels[4] = 1900; //CH5
	RC_cmd.channels[5] = 1900; //CH6
	RC_cmd.channels[6] = 1900; //CH7
	RC_cmd.channels[7] = 1900; //CH8
}

void rc_init_mav(void)
{
	RC_cmd.channels[0] = 1500; //CH1
 	RC_cmd.channels[1] = 1500; //CH2
	RC_cmd.channels[2] = 1100; //CH3
	RC_cmd.channels[3] = 1500; //CH4
	RC_cmd.channels[4] = 1900; //CH5
	RC_cmd.channels[5] = 1900; //CH6
	RC_cmd.channels[6] = 1900; //CH7
	RC_cmd.channels[7] = 1900; //CH8
}




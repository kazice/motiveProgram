#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
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


typedef enum
{
	MAV_NEVER,
	MAV_UNLOCK,
	MAV_LOCK,
	MAV_INIT,
	MAV_READY,
	MAV_TAKEOFF,
	MAV_LAND,
	MAV_HOVER

}EN_MAV_STATE;
EN_MAV_STATE enMAVState = MAV_NEVER;
ros::Subscriber n_network_cmd_sub;
ros::Subscriber n_network_data_sub;
ros::Publisher mav_override_pub;
ros::Publisher mav_pid_controller_pub;
ros::Publisher mav_debug_data_pub;

mavros_msgs::OverrideRCIn RC_cmd;
network_client::Optitrack_data Optitrack_data;
mav_client::Debug_data Debug_data_out;
network_client::Optitrack Mav_net_cmd;



int timecount;
int maintimecount = 0;

char mav_lock_flag = 1; //1--lock 0--unlock

void n_network_cmd_callback(network_client::Optitrack net_cmd);
void n_network_data_callback(network_client::Optitrack_data net_data);

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

	ros::Rate loop_rate(1000);

	data_init();
	Display_Main_Menu();
	rc_init_mav();

	RC_controller_output.channels[2] = 1100;

    n_network_cmd_sub = mav_client_nh.subscribe("/network_client/network_cmd", 1, n_network_cmd_callback);
    n_network_data_sub = mav_client_nh.subscribe("/network_client/network_optitrack_data", 1, n_network_data_callback);
	mav_override_pub = mav_client_nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 100);	
	mav_pid_controller_pub = mav_client_nh.advertise<mavros_msgs::OverrideRCIn>("/mav_client/rc/pid", 100);
	mav_debug_data_pub = mav_client_nh.advertise<mav_client::Debug_data>("/mav_client/debug", 100);

	while(ros::ok())
	{
		ros::spinOnce();
		maintimecount++;
		if(maintimecount%20 == 1)
		{
			if(enMAVState == MAV_LAND)
			{
				timecount++;
				mav_land();
				RC_cmd.channels[0] = RC_controller_output.channels[0];
				RC_cmd.channels[1] = RC_controller_output.channels[1];
				RC_cmd.channels[2] = RC_controller_output.channels[2];
				RC_cmd.channels[3] = RC_controller_output.channels[3];
				if(Optitrack_data.health_flag == 1)
				{
					if(realPosMAV.flz < 0.2 || timecount >= 200)
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
					if(timecount >= 150)
					{
						enMAVState = MAV_LOCK;
						ROS_INFO("locking..........");
						rc_lock_mav();
						timecount = 0;
					}
				}
			}
		}
		if(maintimecount%10 == 0)
		{
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
			mav_debug_data_pub_cb();
			mav_pid_controller_pub_cb();
		}

		if(enMAVState == MAV_UNLOCK)
		{
			timecount++;
			if(timecount > 6000)
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
			if(timecount > 6000)
			{
				enMAVState = MAV_NEVER;
				enTakeoffState = NEVER;
				mav_lock_flag = 1;
				ROS_INFO("lock OK");
				rc_init_mav();
			}
		}
		/*pub of override*/
		if(maintimecount%20 == 19)
		{
			mav_override_pub_cb();
		}

		loop_rate.sleep();
	}
}

void n_network_data_callback(network_client::Optitrack_data net_data)
{
	Optitrack_data = net_data;
    if(net_data.network_health_flag == 1)
    {
    	realPosMAV.flx = net_data.posx;
    	realPosMAV.fly = net_data.posy;
    	realPosMAV.flz = net_data.posz;

    	realVeltMAV.flvx = net_data.veltx;
    	realVeltMAV.flvy = net_data.velty;
    	realVeltMAV.flvz = net_data.veltz;

    	realAttMAV.flroll = net_data.roll;
    	realAttMAV.flyaw = net_data.yaw;
    	realAttMAV.flpitch = net_data.pitch;
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
                printf("Cmd is:%c\n",net_cmd.control_cmd);
                switch(net_cmd.control_cmd)
                {
                  case 'a':             
                	  printf("MAV SDK version:0.0.1\n");
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
                  case 'f':
                    /* reload pid data*/
                	  ROS_INFO("reload pid data");
                	  data_init();
                    break;
                  case 'g':
                	  ROS_INFO("override time test");
                	  mav_override_pub_cb();
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

	msg_pub.opt_velt[0] = realVeltMAV.flvx;
	msg_pub.opt_velt[1] = realVeltMAV.flvy;
	msg_pub.opt_velt[2] = realVeltMAV.flvz;

	msg_pub.opt_attitude[0] = realAttMAV.flroll;
	msg_pub.opt_attitude[1] = realAttMAV.flyaw;
	msg_pub.opt_attitude[2] = realAttMAV.flpitch;

	msg_pub.des_pos[0] = desPosMAV.flx;
	msg_pub.des_pos[1] = desPosMAV.fly;
	msg_pub.des_pos[2] = desPosMAV.flz;

	msg_pub.des_velt[0] = desVeltMAV.flvx;
	msg_pub.des_velt[1] = desVeltMAV.flvy;
	msg_pub.des_velt[2] = desVeltMAV.flvz;

	msg_pub.des_attitude[0] = desAttMAV.flroll;
	msg_pub.des_attitude[1] = desAttMAV.flyaw;
	msg_pub.des_attitude[2] = desAttMAV.flpitch;

	msg_pub.err_pos[0] = pidCrlMAV.pidPosx.flE;
	msg_pub.err_pos[1] = pidCrlMAV.pidPosy.flE;
	msg_pub.err_pos[2] = pidCrlMAV.pidPosz.flE;

	msg_pub.err_velt[0] = pidCrlMAV.pidVeltx.flE;
	msg_pub.err_velt[1] = pidCrlMAV.pidVelty.flE;
	msg_pub.err_velt[2] = pidCrlMAV.pidVeltz.flE;

	msg_pub.err_attitude[0] = pidCrlMAV.pidRoll.flE;
	msg_pub.err_attitude[1] = pidCrlMAV.pidYaw.flE;
	msg_pub.err_attitude[2] = pidCrlMAV.pidPitch.flE;

	msg_pub.u_pos[0] = pidCrlMAV.pidPosx.flU;
	msg_pub.u_pos[1] = pidCrlMAV.pidPosy.flU;
	msg_pub.u_pos[2] = pidCrlMAV.pidPosz.flU;

	msg_pub.u_velt[0] = pidCrlMAV.pidVeltx.flU;
	msg_pub.u_velt[1] = pidCrlMAV.pidVelty.flU;
	msg_pub.u_velt[2] = pidCrlMAV.pidVeltz.flU;

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




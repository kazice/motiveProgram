#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <mav_client/Debug_data.h>

/**UDP socket create**/
int sockServerUDP;
sockaddr_in addrServer;
union Send
{
	char chSendBuf[512];
	float flSendBuf[128];
};
typedef struct
{
	unsigned char Head;
	unsigned short Len;
	unsigned char Seq;
	unsigned char SysID;
	unsigned char CompID;
	unsigned char MsgID;
	unsigned char CheckA;
	unsigned char CheckB;
	Send Data;
	unsigned char End;
}NETLINK_DATA;
NETLINK_DATA net_link_data_send;
char SendBuf[522];

mav_client::Debug_data mav_debug_send;
ros::Subscriber mav_debug_data_sub;
void mav_debug_callback(mav_client::Debug_data mav_debug_data);
void init_debug_UDP(void);
void send_debug_data(void);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mav_debug");
	ROS_INFO("mav_service_debug start");
	ros::NodeHandle mav_debug_nh;

	net_link_data_send.Data.chSendBuf[0] = 0;
	mav_debug_data_sub = mav_debug_nh.subscribe("/mav_client/debug", 1, mav_debug_callback);
	init_debug_UDP();
	while(ros::ok())
	{
		ros::spinOnce();
	}
}

void mav_debug_callback(mav_client::Debug_data mav_debug_data)
{
	mav_debug_send = mav_debug_data;
	net_link_data_send.Head = 0xFE;
	net_link_data_send.End = 0xAA;
	net_link_data_send.Len = 372;
	net_link_data_send.CompID = 1;
	net_link_data_send.SysID = 1;
	net_link_data_send.MsgID = 0x88;
	net_link_data_send.CheckA = 0;
	net_link_data_send.CheckB = 0;
	net_link_data_send.Data.chSendBuf[0] = mav_debug_send.msgid;
	net_link_data_send.Data.chSendBuf[1] = mav_debug_send.opt_health_flag;
	net_link_data_send.Data.chSendBuf[2] = mav_debug_send.network_health_flag;
	net_link_data_send.Data.chSendBuf[3] = mav_debug_send.pendulum_health_flag;
	net_link_data_send.Data.chSendBuf[4] = mav_debug_send.mavstate;
	net_link_data_send.Data.chSendBuf[5] = mav_debug_send.takeoffstate;
	net_link_data_send.Data.chSendBuf[6] = mav_debug_send.lost_data;

	net_link_data_send.Data.flSendBuf[2] = mav_debug_send.net_delay;
	net_link_data_send.Data.flSendBuf[3] = (float)mav_debug_send.timer_10ms;
	net_link_data_send.Data.flSendBuf[4] = (float)mav_debug_send.timer_20ms;
	//pos
	net_link_data_send.Data.flSendBuf[5] = mav_debug_send.opt_pos[0];
	net_link_data_send.Data.flSendBuf[6] = mav_debug_send.opt_pos[1];
	net_link_data_send.Data.flSendBuf[7] = mav_debug_send.opt_pos[2];

	net_link_data_send.Data.flSendBuf[8] = mav_debug_send.des_pos[0];
	net_link_data_send.Data.flSendBuf[9] = mav_debug_send.des_pos[1];
	net_link_data_send.Data.flSendBuf[10] = mav_debug_send.des_pos[2];

	net_link_data_send.Data.flSendBuf[11] = mav_debug_send.err_pos[0];
	net_link_data_send.Data.flSendBuf[12] = mav_debug_send.err_pos[1];
	net_link_data_send.Data.flSendBuf[13] = mav_debug_send.err_pos[2];

	net_link_data_send.Data.flSendBuf[14] = mav_debug_send.u_pos[0];
	net_link_data_send.Data.flSendBuf[15] = mav_debug_send.u_pos[1];
	net_link_data_send.Data.flSendBuf[16] = mav_debug_send.u_pos[2];

	net_link_data_send.Data.flSendBuf[17] = mav_debug_send.esum_pos[0];
	net_link_data_send.Data.flSendBuf[18] = mav_debug_send.esum_pos[1];
	net_link_data_send.Data.flSendBuf[19] = mav_debug_send.esum_pos[2];
	//velt
	net_link_data_send.Data.flSendBuf[20] = mav_debug_send.opt_velt[0];
	net_link_data_send.Data.flSendBuf[21] = mav_debug_send.opt_velt[1];
	net_link_data_send.Data.flSendBuf[22] = mav_debug_send.opt_velt[2];

	net_link_data_send.Data.flSendBuf[23] = mav_debug_send.filter_velt[0];
	net_link_data_send.Data.flSendBuf[24] = mav_debug_send.filter_velt[1];
	net_link_data_send.Data.flSendBuf[25] = mav_debug_send.filter_velt[2];

	net_link_data_send.Data.flSendBuf[26] = mav_debug_send.des_velt[0];
	net_link_data_send.Data.flSendBuf[27] = mav_debug_send.des_velt[1];
	net_link_data_send.Data.flSendBuf[28] = mav_debug_send.des_velt[2];

	net_link_data_send.Data.flSendBuf[29] = mav_debug_send.err_velt[0];
	net_link_data_send.Data.flSendBuf[30] = mav_debug_send.err_velt[1];
	net_link_data_send.Data.flSendBuf[31] = mav_debug_send.err_velt[2];

	net_link_data_send.Data.flSendBuf[32] = mav_debug_send.u_velt[0];
	net_link_data_send.Data.flSendBuf[33] = mav_debug_send.u_velt[1];
	net_link_data_send.Data.flSendBuf[34] = mav_debug_send.u_velt[2];

	net_link_data_send.Data.flSendBuf[35] = mav_debug_send.esum_velt[0];
	net_link_data_send.Data.flSendBuf[36] = mav_debug_send.esum_velt[1];
	net_link_data_send.Data.flSendBuf[37] = mav_debug_send.esum_velt[2];
	//attitude
	net_link_data_send.Data.flSendBuf[38] = mav_debug_send.opt_attitude[0];
	net_link_data_send.Data.flSendBuf[39] = mav_debug_send.opt_attitude[1];
	net_link_data_send.Data.flSendBuf[40] = mav_debug_send.opt_attitude[2];

	net_link_data_send.Data.flSendBuf[41] = mav_debug_send.des_attitude[0];
	net_link_data_send.Data.flSendBuf[42] = mav_debug_send.des_attitude[1];
	net_link_data_send.Data.flSendBuf[43] = mav_debug_send.des_attitude[2];

	net_link_data_send.Data.flSendBuf[44] = mav_debug_send.err_attitude[0];
	net_link_data_send.Data.flSendBuf[45] = mav_debug_send.err_attitude[1];
	net_link_data_send.Data.flSendBuf[46] = mav_debug_send.err_attitude[2];

	net_link_data_send.Data.flSendBuf[47] = mav_debug_send.u_attitude[0];
	net_link_data_send.Data.flSendBuf[48] = mav_debug_send.u_attitude[1];
	net_link_data_send.Data.flSendBuf[49] = mav_debug_send.u_attitude[2];

	net_link_data_send.Data.flSendBuf[50] = mav_debug_send.esum_attitude[0];
	net_link_data_send.Data.flSendBuf[51] = mav_debug_send.esum_attitude[1];
	net_link_data_send.Data.flSendBuf[52] = mav_debug_send.esum_attitude[2];
	//attitude pendulum
	net_link_data_send.Data.flSendBuf[53] = mav_debug_send.opt_attitude_pendulum[0];
	net_link_data_send.Data.flSendBuf[54] = mav_debug_send.opt_attitude_pendulum[1];

	net_link_data_send.Data.flSendBuf[55] = mav_debug_send.des_attitude_pendulum[0];
	net_link_data_send.Data.flSendBuf[56] = mav_debug_send.des_attitude_pendulum[1];

	net_link_data_send.Data.flSendBuf[57] = mav_debug_send.err_attitude_pendulum[0];
	net_link_data_send.Data.flSendBuf[58] = mav_debug_send.err_attitude_pendulum[1];

	net_link_data_send.Data.flSendBuf[59] = mav_debug_send.u_attitude_pendulum[0];
	net_link_data_send.Data.flSendBuf[60] = mav_debug_send.u_attitude_pendulum[1];

	net_link_data_send.Data.flSendBuf[61] = mav_debug_send.esum_attitude_pendulum[0];
	net_link_data_send.Data.flSendBuf[62] = mav_debug_send.esum_attitude_pendulum[1];

	net_link_data_send.Data.flSendBuf[63] = mav_debug_send.pendulum[0];
	net_link_data_send.Data.flSendBuf[64] = mav_debug_send.pendulum[1];
	net_link_data_send.Data.flSendBuf[65] = mav_debug_send.pendulum[2];
	net_link_data_send.Data.flSendBuf[66] = mav_debug_send.pendulum[3];
	net_link_data_send.Data.flSendBuf[67] = mav_debug_send.pendulum[4];

	net_link_data_send.Data.flSendBuf[68] = mav_debug_send.test_1[0];
	net_link_data_send.Data.flSendBuf[69] = mav_debug_send.test_1[1];
	net_link_data_send.Data.flSendBuf[70] = mav_debug_send.test_1[2];

	net_link_data_send.Data.flSendBuf[71] = mav_debug_send.test_2[0];
	net_link_data_send.Data.flSendBuf[72] = mav_debug_send.test_2[1];
	net_link_data_send.Data.flSendBuf[73] = mav_debug_send.test_2[2];

	net_link_data_send.Data.flSendBuf[74] = mav_debug_send.test_3[0];
	net_link_data_send.Data.flSendBuf[75] = mav_debug_send.test_3[1];
	net_link_data_send.Data.flSendBuf[76] = mav_debug_send.test_3[2];
	//rc
	net_link_data_send.Data.flSendBuf[77] = (float)mav_debug_send.rc_controller[0];
	net_link_data_send.Data.flSendBuf[78] = (float)mav_debug_send.rc_controller[1];
	net_link_data_send.Data.flSendBuf[79] = (float)mav_debug_send.rc_controller[2];
	net_link_data_send.Data.flSendBuf[80] = (float)mav_debug_send.rc_controller[3];
	net_link_data_send.Data.flSendBuf[81] = (float)mav_debug_send.rc_controller[4];
	net_link_data_send.Data.flSendBuf[82] = (float)mav_debug_send.rc_controller[5];
	net_link_data_send.Data.flSendBuf[83] = (float)mav_debug_send.rc_controller[6];
	net_link_data_send.Data.flSendBuf[84] = (float)mav_debug_send.rc_controller[7];

	net_link_data_send.Data.flSendBuf[85] = (float)mav_debug_send.rc2mav[0];
	net_link_data_send.Data.flSendBuf[86] = (float)mav_debug_send.rc2mav[1];
	net_link_data_send.Data.flSendBuf[87] = (float)mav_debug_send.rc2mav[2];
	net_link_data_send.Data.flSendBuf[88] = (float)mav_debug_send.rc2mav[3];
	net_link_data_send.Data.flSendBuf[89] = (float)mav_debug_send.rc2mav[4];
	net_link_data_send.Data.flSendBuf[90] = (float)mav_debug_send.rc2mav[5];
	net_link_data_send.Data.flSendBuf[91] = (float)mav_debug_send.rc2mav[6];
	net_link_data_send.Data.flSendBuf[92] = (float)mav_debug_send.rc2mav[7];
	send_debug_data();
}

void init_debug_UDP(void)
{
	ROS_INFO("start init UDP for debug");
	bzero(&addrServer,sizeof(addrServer));
	addrServer.sin_addr.s_addr = inet_addr("192.168.2.252");
	addrServer.sin_family=AF_INET;
	addrServer.sin_port=htons(4999);
	sockServerUDP=socket(AF_INET,SOCK_DGRAM,0);

	ROS_INFO("init of UDP finish,ready to send data");
}

void send_debug_data(void)
{
    /*int PASCAL FAR sendto( SOCKET s, const char FAR* buf, int len, int flags,const struct sockaddr FAR* to, int tolen);
     * s：一个标识套接口的描述字。
     * buf：包含待发送数据的缓冲区。
     * len：buf缓冲区中数据的长度。
     * flags：调用方式标志位。
     * to：（可选）指针，指向目的套接口Rec的地址。
     * tolen：to所指地址的长度。
    */
	SendBuf[0] = net_link_data_send.Head;
	SendBuf[1] = net_link_data_send.Len;
	SendBuf[2] = net_link_data_send.Len>>8;
	SendBuf[3] = net_link_data_send.Seq;
	SendBuf[4] = net_link_data_send.SysID;
	SendBuf[5] = net_link_data_send.CompID;
	SendBuf[6] = net_link_data_send.MsgID;

	for(int data_i=0;data_i<net_link_data_send.Len;data_i++)
	{
		SendBuf[7+data_i] = net_link_data_send.Data.chSendBuf[data_i];
	}
	SendBuf[7+net_link_data_send.Len] = net_link_data_send.CheckA;
	SendBuf[8+net_link_data_send.Len] = net_link_data_send.CheckB;
	SendBuf[9+net_link_data_send.Len] = net_link_data_send.End;
	sendto(sockServerUDP,SendBuf,522,0,(struct sockaddr *)&addrServer,sizeof(addrServer));
}

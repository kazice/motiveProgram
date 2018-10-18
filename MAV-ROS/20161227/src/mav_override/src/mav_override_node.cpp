/************
*yanwei 20161123
*test mav_rcoverride_publisher by node
*************/
#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>

ros::Publisher mav_override_pub;

void mav_override_pub_cb(void) 
{
	mavros_msgs::OverrideRCIn rmsg;

	rmsg.channels[0] = 1500;
	rmsg.channels[1] = 1500;
	rmsg.channels[2] = 1500;
	rmsg.channels[3] = 1500;
	rmsg.channels[4] = 1500;
	rmsg.channels[5] = 1500;
	rmsg.channels[6] = 1500;
	rmsg.channels[7] = 1500;
	mav_override_pub.publish(rmsg);
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mav_override");
	ROS_INFO("mav_override test start");
	ros::NodeHandle mav_override_nh("mav_override");
	ros::Rate loop_rate(50);
	mav_override_pub = mav_override_nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);	
	while(ros::ok())
	{
		mav_override_pub_cb();
		loop_rate.sleep();
	}
	return 0;
}


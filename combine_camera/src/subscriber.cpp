#include "ros/ros.h"
#include "targetDetection_topic/TargetPosition.h"

void msgCallBack(const targetDetection_topic::TargetPosition::ConstPtr& msg)
{
	ROS_INFO("receive x_coord = %d", msg->x_coord);
	ROS_INFO("receive y_coord = %d\n", msg->y_coord);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle nh;
	
        ros::Subscriber sub = nh.subscribe("target_msg", 100, msgCallBack);

	ros::spin();
	return 0;
}


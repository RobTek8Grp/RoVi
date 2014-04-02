#include <ros/ros.h>
#include "rx60_wrapper/command.h"
#include "RX60wrapper.hpp"

void printState(sensor_msgs::JointState::Ptr state)
{
	for(int i = 0 ; i < state->position.size() ; i++)
	{
		ROS_INFO("%s: Joint state: %f",ros::this_node::getName().c_str(),  state->position[i]);
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "RX60_wrapper");

	RX60_wrapper rx60;

	ros::Rate r(20);

	while(ros::ok())
	{
		//rx60.test();
		rx60.publishRobotState();
		r.sleep();
	}

	return 0;
}

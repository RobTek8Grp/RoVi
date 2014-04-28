/*
 * RX60wrapper.hpp
 *
 *  Created on: Mar 29, 2014
 *      Author: leon
 */

#ifndef RX60WRAPPER_HPP_
#define RX60WRAPPER_HPP_

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "rx60_wrapper/command.h"

class RX60_wrapper
{
private:
    const double deg_to_rad = -M_PI / 180.0;
    const double rad_to_deg = -180.0 / M_PI ;

protected:
    ros::NodeHandle _local_node_handler, _global_node_handler;
	ros::ServiceClient _client_handle;
//	sensor_msgs::JointState::Ptr _message;
	ros::Publisher statePublisher;

public:
	RX60_wrapper();
	virtual ~RX60_wrapper();

	void setJointState(sensor_msgs::JointState::Ptr);
	sensor_msgs::JointState::Ptr getJointState(void);
	void test(void);
	void publishRobotState (void);
};

#endif /* RX60WRAPPER_HPP_ */

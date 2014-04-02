/*
 * RX60wrapper.cpp
 *
 *  Created on: Mar 29, 2014
 *      Author: leon
 */

#include "RX60wrapper.hpp"

RX60_wrapper::RX60_wrapper()
:_local_node_handler("~"), _global_node_handler()
{

  _client_handle = _global_node_handler.serviceClient<rx60_wrapper::command>("rx60_controller/rx60_command");
	this->statePublisher = this->_global_node_handler.advertise<sensor_msgs::JointState>("/robot_rx60b/controller_joint_states", 10);

}

RX60_wrapper::~RX60_wrapper()
{

}

void RX60_wrapper::setJointState(sensor_msgs::JointState::Ptr msg)
{
	rx60_wrapper::command service_object;

	service_object.request.command_number = rx60_wrapper::command::Request::SET_JOINT_CONFIGURATION;

	service_object.request.joint1 = msg->position.at(0);
	service_object.request.joint2 = msg->position.at(1);
	service_object.request.joint3 = msg->position.at(2);
	service_object.request.joint4 = msg->position.at(3);
	service_object.request.joint5 = msg->position.at(4);
	service_object.request.joint6 = msg->position.at(5);

	if (_client_handle.call(service_object))
	{
		ROS_INFO("%s: Moving robot",ros::this_node::getName().c_str());
	}
	else
	{
	    	ROS_ERROR("%s: Failed in setJointState method",ros::this_node::getName().c_str());
	}
}

sensor_msgs::JointState::Ptr RX60_wrapper::getJointState(void)
{
	// get current joint states
	rx60_wrapper::command service_object;
	auto message = sensor_msgs::JointState::Ptr(new sensor_msgs::JointState());

  	service_object.request.command_number = rx60_wrapper::command::Request::GET_JOINT_CONFIGURATION;

	if (_client_handle.call(service_object))
	{
		message->position.push_back(service_object.response.joint1);
		message->position.push_back(service_object.response.joint2);
		message->position.push_back(service_object.response.joint3);
		message->position.push_back(service_object.response.joint4);
		message->position.push_back(service_object.response.joint5);
		message->position.push_back(service_object.response.joint6); 

		for(int i = 0 ; i < message->position.size() ; i++)
		{
			ROS_INFO("%s: Joint %d: %f",ros::this_node::getName().c_str(), i+1, message->position[i]);
		}
	}
	else
	{
		ROS_ERROR("%s: Failed in getJointState method",ros::this_node::getName().c_str());
	}

	return message;
}

void RX60_wrapper::test(void)
{
	double q1,q2,q3,q4,q5,q6;

	rx60_wrapper::command service_object;
  	service_object.request.command_number = rx60_wrapper::command::Request::GET_JOINT_CONFIGURATION;
	
	if (_client_handle.call(service_object))
	{
		q1 = service_object.response.joint1;
		q2 = service_object.response.joint2;
		q3 = service_object.response.joint3;
		q4 = service_object.response.joint4;
		q5 = service_object.response.joint5;
		q6 = service_object.response.joint6; 
	}
	else
	{
		ROS_ERROR("%s: Failed in getJointStates method",ros::this_node::getName().c_str());
	}

	if(q1+q2+q3+q4+q5+q6) //sanity check
	{
		//set joint states
		rx60_wrapper::command service_object2;

		service_object2.request.command_number = rx60_wrapper::command::Request::SET_JOINT_CONFIGURATION;

		service_object2.request.joint1 = q1;
		service_object2.request.joint2 = q2;
		service_object2.request.joint3 = q3;
		service_object2.request.joint4 = q4;
		service_object2.request.joint5 = q5;
		service_object2.request.joint6 = q6;

		if (_client_handle.call(service_object2))
		{
			ROS_INFO("%s: Moving robot",ros::this_node::getName().c_str());
		}
		else
		{
		    	ROS_ERROR("%s: Failed in test method",ros::this_node::getName().c_str());
		}

		// Read and print new joint states
		rx60_wrapper::command service_object3;
		//auto message = sensor_msgs::JointState::Ptr(new sensor_msgs::JointState());

	  	service_object3.request.command_number = rx60_wrapper::command::Request::GET_JOINT_CONFIGURATION;

		if (_client_handle.call(service_object3))
		{
			ROS_INFO("%s: Joint state: [%f,%f,%f,%f,%f,%f]",ros::this_node::getName().c_str(), 
				service_object3.response.joint1,
				service_object3.response.joint2,
				service_object3.response.joint3,
				service_object3.response.joint4,
				service_object3.response.joint5,
				service_object3.response.joint6 );
		}
		else
		{
			ROS_ERROR("%s: Failed in test method",ros::this_node::getName().c_str());
		}
	}
}

void RX60_wrapper::publishRobotState (void)
{
	auto msg = sensor_msgs::JointState::Ptr(new sensor_msgs::JointState());
	rx60_wrapper::command service;
	service.request.command_number = rx60_wrapper::command::Request::GET_JOINT_CONFIGURATION;

	double deg_to_rad = M_PI / 180.0;

	if (_client_handle.call(service))
	{
		msg->position.push_back(service.response.joint1 * deg_to_rad);
		msg->position.push_back(service.response.joint2 * deg_to_rad);
		msg->position.push_back(service.response.joint3 * deg_to_rad);
		msg->position.push_back(service.response.joint4 * deg_to_rad);
		msg->position.push_back(service.response.joint5 * deg_to_rad);
		msg->position.push_back(service.response.joint6 * deg_to_rad);
	}
	else
	{
		ROS_ERROR("%s: Failed in publishing joint states ...",ros::this_node::getName().c_str());
	}

	msg->header.stamp = ros::Time::now();
	statePublisher.publish(*msg);
}

/*************************************************************************************
# File:     plc_controller_node.cpp
# Purpose:  Create a interface node to handle communication with the PLC
# Project:  RSD - group 4
# Author:   Jeppe Pedersen <jepe009@student.sdu.dk>
# Created:  2013/02/18 - Jeppe Pedersen
# Version:  0.1 Initial version
*************************************************************************************/
#include <string>

#include "ros/ros.h"

#include "RX60Driver.hpp"

int main(int argc, char **argv)
{
  /* parameters */
  std::string rx60_controller_ip;
  std::string rx60_controller_port;
  std::string rx60_command_service_name;
  double rx60_max_velocity;
  double rx60_max_acceleration;
  double rx60_max_deceleration;
  double rx60_max_rotational_velocity;
  double rx60_max_cartesian_velocity;
  bool rx60_blend;
  double rx60_reach;
  double rx60_leave;

  /* initialize ros usage */
  ros::init(argc, argv, "rx60_controller");
 
  /* private nodehandlers */
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  /* read parameters from ros parameter server if available otherwise use default values */
  n.param<std::string> ("rx60_controller_ip", rx60_controller_ip, "172.16.1.25");
  n.param<std::string> ("rx60_controller_port", rx60_controller_port, "22222");
  n.param<std::string> ("rx60_command_service_name", rx60_command_service_name, "rx60_command");
  n.param<double> ("rx60_max_velocity", rx60_max_velocity, 100.0);
  n.param<double> ("rx60_max_acceleration", rx60_max_acceleration, 100.0);
  n.param<double> ("rx60_max_deceleration", rx60_max_deceleration, 100.0);
  n.param<double> ("rx60_max_rotational_velocity", rx60_max_rotational_velocity, 1000000000.0);
  n.param<double> ("rx60_max_cartesian_velocity", rx60_max_cartesian_velocity, 1000.0);
  n.param<bool> ("rx60_blend", rx60_blend, true);
  n.param<double> ("rx60_reach", rx60_reach, 0);
  n.param<double> ("rx60_leave", rx60_leave, 0);

  RX60Driver * robot;
  ROS_INFO("Connection to RX60B at ip: %s and port: %s", rx60_controller_ip.c_str(), rx60_controller_port.c_str());
  robot = new RX60Driver(rx60_controller_ip, rx60_controller_port);
  robot->rx60_controller_ip = rx60_controller_ip;
  robot->rx60_controller_port = rx60_controller_port;

  if (!robot->isConnected()) {
	std::cout << "Could not connect to robot!" << std::endl;
	abort();
  }
  else
  {
	std::cout << "Connection established" << std::endl;
	// Set robot parameters
	robot->setMovementParameters(	rx60_max_velocity,
									rx60_max_acceleration,
									rx60_max_deceleration,
									rx60_max_rotational_velocity,
									rx60_max_cartesian_velocity,
									rx60_blend,
									rx60_reach,
									rx60_leave);
	std::cout <<"Node name " << rx60_command_service_name << std::endl;
	robot->rx60_service = n.advertiseService(rx60_command_service_name, &RX60Driver::commandServiceHandler, robot);
	ros::spin();
  }

  return 0;
}


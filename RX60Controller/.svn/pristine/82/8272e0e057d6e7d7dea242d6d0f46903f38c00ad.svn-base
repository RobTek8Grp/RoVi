#include "ros/ros.h"
 #include "rx60controller/command.h"
 #include <cstdlib>
 
 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "rx60client");
   //if (argc != 3)
   //{
     ROS_INFO("usage: rx60client");
   // return 1;
   //}
 
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<rx60controller::command>("rx60_controller/rx60_command");
   rx60controller::command srv;
   srv.request.command_number = rx60controller::command::Request::GET_JOINT_CONFIGURATION;
  if (client.call(srv))
  {
     ROS_INFO("Cool %lf",(double)srv.response.joint1);
     ROS_INFO("Cool %lf",(double)srv.response.joint2);
     ROS_INFO("Cool %lf",(double)srv.response.joint3);
     ROS_INFO("Cool %lf",(double)srv.response.joint4);
     ROS_INFO("Cool %lf",(double)srv.response.joint5);
     ROS_INFO("Cool %lf",(double)srv.response.joint6);
  }
   else
   {
    ROS_ERROR("Failed to call service add_two_ints");
     return 1;
  }
 
   for(int i = 0; i < 100; i++){

   rx60controller::command srv1;
   srv1.request.command_number = rx60controller::command::Request::SET_JOINT_CONFIGURATION;
   if( i % 2 == 0)
   	srv1.request.joint1 = 10.0;
   else   	
	srv1.request.joint1 = -10.0;

   srv1.request.joint2 = 0.0;
   srv1.request.joint3 = 0.0;
   srv1.request.joint4 = 0.0;
   srv1.request.joint5 = 0.0;
   srv1.request.joint6 = 0.0;
  if (client.call(srv1))
  {
     ROS_INFO("send new joints");
  }
   else
   {
    ROS_ERROR("Failed to call service command");
     return 1;
  }
 
   }
   rx60controller::command srv2;
   srv2.request.command_number = rx60controller::command::Request::GET_JOINT_CONFIGURATION;
  if (client.call(srv2))
  {
     ROS_INFO("Cool %lf",(double)srv2.response.joint1);
     ROS_INFO("Cool %lf",(double)srv2.response.joint2);
     ROS_INFO("Cool %lf",(double)srv2.response.joint3);
     ROS_INFO("Cool %lf",(double)srv2.response.joint4);
     ROS_INFO("Cool %lf",(double)srv2.response.joint5);
     ROS_INFO("Cool %lf",(double)srv2.response.joint6);
  }
   else
   {
    ROS_ERROR("Failed to call service add_two_ints");
	return 1;
	}
   return 0;
 }

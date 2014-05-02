#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <controller_manager/controller_manager.h>
#include "RX60wrapper.hpp"

class RX60Robot : public hardware_interface::RobotHW //, public RX60_wrapper
{
public:
    RX60Robot()
    {
        //RX60_wrapper();

        for (int j = 0; j <= 6; j++)
        {
            pos[j] = 0;
            vel[j] = 0;
            eff[j] = 0;
            cmd[j] = 0;
        }
        // Joint state handles
        hardware_interface::JointStateHandle joint_a1("a1", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(joint_a1);
        hardware_interface::JointStateHandle joint_a2("a2", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(joint_a2);
        hardware_interface::JointStateHandle joint_a3("a3", &pos[2], &vel[2], &eff[2]);
        jnt_state_interface.registerHandle(joint_a3);
        hardware_interface::JointStateHandle joint_a4("a4", &pos[3], &vel[3], &eff[3]);
        jnt_state_interface.registerHandle(joint_a4);
        hardware_interface::JointStateHandle joint_a5("a5", &pos[4], &vel[4], &eff[4]);
        jnt_state_interface.registerHandle(joint_a5);
        hardware_interface::JointStateHandle joint_a6("a6", &pos[5], &vel[5], &eff[5]);
        jnt_state_interface.registerHandle(joint_a6);

        this->registerInterface(&jnt_state_interface);

        // joint position interface
        hardware_interface::JointHandle pos_a1(jnt_state_interface.getHandle("a1"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_a1);
        hardware_interface::JointHandle pos_a2(jnt_state_interface.getHandle("a2"), &cmd[1]);
        jnt_pos_interface.registerHandle(pos_a2);
        hardware_interface::JointHandle pos_a3(jnt_state_interface.getHandle("a3"), &cmd[2]);
        jnt_pos_interface.registerHandle(pos_a3);
        hardware_interface::JointHandle pos_a4(jnt_state_interface.getHandle("a4"), &cmd[3]);
        jnt_pos_interface.registerHandle(pos_a4);
        hardware_interface::JointHandle pos_a5(jnt_state_interface.getHandle("a5"), &cmd[4]);
        jnt_pos_interface.registerHandle(pos_a5);
        hardware_interface::JointHandle pos_a6(jnt_state_interface.getHandle("a6"), &cmd[5]);
        jnt_pos_interface.registerHandle(pos_a6);

        this->registerInterface(&jnt_pos_interface);

    }
    void read(const sensor_msgs::JointState::Ptr robotState)
    {

//        sensor_msgs::JointState::Ptr robotState = getJointState();

        for (int j = 0; j <= 6; j++)
        {
            pos[j] = robotState->position[j];
            vel[j] = 0;
            eff[j] = 0;
        }
    }
    sensor_msgs::JointState::Ptr write()
    {
        //std::cout << "Joint 1: cmd: " << cmd[0] << " pos: " << pos[0] << " vel: " << vel[0] << " eff: " << eff[0] << std::endl;
        //std::cout << "Joint 2: cmd: " << cmd[1] << " pos: " << pos[1] << " vel: " << vel[1] << " eff: " << eff[1] << std::endl;
        /*std::cout << "Joint 3: " << cmd[2] << std::endl;
        std::cout << "Joint 4: " << cmd[3] << std::endl;
        std::cout << "Joint 5: " << cmd[4] << std::endl;
        std::cout << "Joint 6: " << cmd[5] << std::endl;*/

        auto msg = sensor_msgs::JointState::Ptr(new sensor_msgs::JointState());

        msg->name.push_back("a1");
        msg->name.push_back("a2");
        msg->name.push_back("a3");
        msg->name.push_back("a4");
        msg->name.push_back("a5");
        msg->name.push_back("a6");

        msg->position.push_back(cmd[0]);
        msg->position.push_back(cmd[1]);
        msg->position.push_back(cmd[2]);
        msg->position.push_back(cmd[3]);
        msg->position.push_back(cmd[4]);
        msg->position.push_back(cmd[5]);

        msg->header.stamp = ros::Time::now();

        return msg;
    }
    void info()
    {
        for (int i = 0; i < 6; i++)
            ROS_INFO("Joint %i: cmd: %f pos: %f vel: %f eff: %f" , i, cmd[i], pos[i], vel[i], eff[i]);
    }

private:
//    const double deg_to_rad = -M_PI / 180.0;
//    const double rad_to_deg = -180.0 / M_PI ;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double cmd[6];
    double pos[6];
    double vel[6];
    double eff[6];
};
int main( int argc, char** argv )
{

    ros::init(argc, argv, "rx60_hardware");
    ros::NodeHandle nh_("robot_rx60b");
    RX60Robot robot;
    RX60_wrapper wrapper;
    controller_manager::ControllerManager cm(&robot, nh_);
    ros::Publisher state_publisher = nh_.advertise<sensor_msgs::JointState>("/robot_rx60b/controller_joint_states", 10);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    //ros::MultiThreadedSpinner spinner(4);
    //spinner.spin();
    ROS_INFO("Spinner started!!!");

    ros::Time last = ros::Time::now();
    ros::Rate r(20);
    int alive_count = 0;


    while (ros::ok())
    {
        ros::Duration period = ros::Time::now() - last;

        ROS_INFO("GetJointState");
        const sensor_msgs::JointState::Ptr robotState = wrapper.getJointState();
        robot.read(robotState);

        ROS_INFO("Update controller");
        cm.update(ros::Time::now(), period);

        ROS_INFO("Get controller update");
        const sensor_msgs::JointState::Ptr robotCmd = robot.write();
        wrapper.setJointState(robotCmd);

        last = ros::Time::now();

        state_publisher.publish(robotState);

        ros::spinOnce();
        r.sleep();
    }
}

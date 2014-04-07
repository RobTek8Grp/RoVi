#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <controller_manager/controller_manager.h>

// Test
#include <sensor_msgs/JointState.h>

class RX60Robot : public hardware_interface::RobotHW
{
public:
    RX60Robot()
    {
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
    void read()
    {
        //std::cout << "Reading from robot .." << std::endl;
        /*
        for (int j = 0; j <= 6; j++)
        {
            pos[j] = -1;
            vel[j] = 0;
            eff[j] = 0;
        }
        pos[1] = 100;
        pos[2] = 3;
        cmd[0] = 1;
        */
    }
    void write()
    {
        //std::cout << "Joint 1: cmd: " << cmd[0] << " pos: " << pos[0] << " vel: " << vel[0] << " eff: " << eff[0] << std::endl;
        //std::cout << "Joint 2: cmd: " << cmd[1] << " pos: " << pos[1] << " vel: " << vel[1] << " eff: " << eff[1] << std::endl;
        /*std::cout << "Joint 3: " << cmd[2] << std::endl;
        std::cout << "Joint 4: " << cmd[3] << std::endl;
        std::cout << "Joint 5: " << cmd[4] << std::endl;
        std::cout << "Joint 6: " << cmd[5] << std::endl;*/
    }
    void info()
    {
        for (int i = 0; i < 6; i++)
            ROS_INFO("Joint %i: cmd: %f pos: %f vel: %f eff: %f" , i, cmd[i], pos[i], vel[i], eff[i]);
    }

//private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double cmd[6];
    double pos[6];
    double vel[6];
    double eff[6];
};

RX60Robot robot;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i = 0; i < 6; i++)
        robot.pos[i] = msg->position[i];
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "rx60_hardware");
    ros::NodeHandle nh_("robot_rx60b");
    controller_manager::ControllerManager cm(&robot, nh_);


    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time last = ros::Time::now();
    ros::Rate r(100);
    int alive_count = 0;

    ros::Subscriber sub = nh_.subscribe("/joint_states", 1000, chatterCallback);

    while (ros::ok())
    {
        ros::Duration period = ros::Time::now() - last;

        robot.read();
        cm.update(ros::Time::now(), period);
        robot.write();

        last = ros::Time::now();

        //ros::spinOnce();
        if (alive_count++ > 100)
        {
            alive_count = 0;
            ROS_INFO("alive @ %6.4f ", ros::Time::now().toSec());
            robot.info();
        }
        r.sleep();
    }
}

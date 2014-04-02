#include <ros/ros.h>
//#include <time.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>


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
        hardware_interface::JointStateHandle joint_a2("a2", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(joint_a2);
        hardware_interface::JointStateHandle joint_a3("a3", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(joint_a3);
        hardware_interface::JointStateHandle joint_a4("a4", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(joint_a4);
        hardware_interface::JointStateHandle joint_a5("a5", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(joint_a5);
        hardware_interface::JointStateHandle joint_a6("a6", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(joint_a6);

        // joint position interface
        hardware_interface::JointHandle pos_a1(jnt_state_interface.getHandle("a1"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_a1);
        hardware_interface::JointHandle pos_a2(jnt_state_interface.getHandle("a2"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_a2);
        hardware_interface::JointHandle pos_a3(jnt_state_interface.getHandle("a3"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_a3);
        hardware_interface::JointHandle pos_a4(jnt_state_interface.getHandle("a4"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_a4);
        hardware_interface::JointHandle pos_a5(jnt_state_interface.getHandle("a5"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_a5);
        hardware_interface::JointHandle pos_a6(jnt_state_interface.getHandle("a6"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_a6);

        this->registerInterface(&jnt_state_interface);
        this->registerInterface(&jnt_pos_interface);
    }
    void read()
    {
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
        std::cout << "Joint 1: cmd: " << cmd[0] << " pos: " << pos[0] << " vel: " << vel[0] << " eff: " << eff[0] << std::endl;
        //std::cout << "Joint 2: cmd: " << cmd[1] << " pos: " << pos[1] << " vel: " << vel[1] << " eff: " << eff[1] << std::endl;
        /*std::cout << "Joint 3: " << cmd[2] << std::endl;
        std::cout << "Joint 4: " << cmd[3] << std::endl;
        std::cout << "Joint 5: " << cmd[4] << std::endl;
        std::cout << "Joint 6: " << cmd[5] << std::endl;*/
    }

private:
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


    /*
    RX60Robot robot;
    controller_manager::ControllerManager cm(&robot);

    struct timespec ts = {0,0};
    ros::Time
        last(ts.tv_sec, ts.tv_nsec),
        now(ts.tv_sec, ts.tv_nsec);
    while (ros::ok())
    {
        ros::Duration period;
        if (!clock_gettime(CLOCK_REALTIME, &ts))
        {
            now.sec = ts.tv_sec;
            now.nsec = ts.tv_nsec;
            period = now - last;
            last = now;
        } else {
            ROS_FATAL("Failed to poll realtime clock!");
            break;
        }

        robot.read();
        cm.update(now, period);
        robot.write();

        ros::Duration(5).sleep();
    }
    */
}

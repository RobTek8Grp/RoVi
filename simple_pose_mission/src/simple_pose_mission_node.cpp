#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!

#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene/planning_scene.h>
//#include <moveit/planning_interface/planning_interface.h>

//#include <moveit/planning_interface/planning_interface.h>

//#include <moveit_msgs/PlanningScene.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/GetStateValidity.h>
//#include <moveit_msgs/DisplayRobotState.h>

//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/robot_state/conversions.h>


// Tools
#include <tf_conversions/tf_kdl.h>

//tf::Quaternion createQuaternionFromRPY(double roll,double pitch,double yaw)

#include "marker_pose.h"

geometry_msgs::Pose object_pose;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    object_pose = feedback->pose;
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char **argv)
{
    const double radius = 0.5;

    ros::init (argc, argv, "simple_pose_mission");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;

    // Marker
    MarkerPose marker("object_center");
    marker.add6DOFBoxMarker("Object center","Object center",processFeedback);


    // Robot and scene
    moveit::planning_interface::MoveGroup group("robot");

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //moveit::planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));


    // Visualizing topic
    //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //moveit_msgs::DisplayTrajectory display_trajectory;

    // Demo pose
    while (ros::ok())
    {
        double angle;
        bool success;
        geometry_msgs::Pose target_pose1;
        moveit::planning_interface::MoveGroup::Plan my_plan;
        for (int itry = 0; itry < 10; itry++)
        {

            angle = rand() % 314;
            angle /= 100;

            tf::Quaternion target_pos_orientation;
            target_pos_orientation.setEuler(0,0,-2.35619449 - angle);

            target_pose1.orientation.x = target_pos_orientation.x();
            target_pose1.orientation.y = target_pos_orientation.y();
            target_pose1.orientation.z = target_pos_orientation.z();
            target_pose1.orientation.w = target_pos_orientation.w();
            target_pose1.position = object_pose.position;
            target_pose1.position.x += std::sin(angle)*radius;
            target_pose1.position.y += std::cos(angle)*radius;

            group.setPoseTarget(target_pose1);

            success = group.plan(my_plan);
            if (success)
                break;
        }

        //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
        /* Sleep to give Rviz time to visualize the plan. */
        if (success)
        {
            ROS_INFO("Moving to angle %f (%f, %f)", angle, target_pose1.position.x, target_pose1.position.y);
            success = group.execute(my_plan);
            if (success)
                ROS_INFO("Plan executed");
            else
                ROS_INFO("Plan did not execute");
            sleep(5.0);
        }
        else
            ROS_INFO("Could not move to angle %f (%f, %f)", angle, target_pose1.position.x, target_pose1.position.y);
    }

}

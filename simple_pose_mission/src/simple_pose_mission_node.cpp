#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Tools
#include <tf_conversions/tf_kdl.h>

#include "marker_pose.h"
#include <std_msgs/Bool.h>
#include <group4_msgs/PointCloudPose.h>

geometry_msgs::Pose object_pose;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    object_pose = feedback->pose;
}

geometry_msgs::Pose tfPoseToGeometryPose(const tf::Pose tPose)
{
    geometry_msgs::Pose gPose;
    gPose.position.x = tPose.getOrigin().x();
    gPose.position.y = tPose.getOrigin().y();
    gPose.position.z = tPose.getOrigin().z();
    gPose.orientation.x = tPose.getOrigin().x();
    gPose.orientation.y = tPose.getOrigin().y();
    gPose.orientation.z = tPose.getOrigin().z();
    gPose.orientation.w = tPose.getOrigin().w();

    return gPose;
}


int main(int argc, char **argv)
{
    const double radius = 0.5; // 0.5
    const int circle_points = 8;
    const int pose_id_max = circle_points;
    const std::string end_effector = ""; //bumblebee_cam1
    const std::string plannerId = "PRMkConfigDefault";
    const std::string frame_id = "base_link";

    ros::init (argc, argv, "simple_pose_mission");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;
    ros::Publisher cameratate_pub = node_handle.advertise<group4_msgs::PointCloudPose>("/robot_rx60b/camerapose", 5);

    // Marker
    MarkerPose marker("object_center");
    //marker.add6DOFBoxMarker("Object center","Object center",processFeedback);
    tf::Pose marker_pose;
    tf::Quaternion marker_rotation;
    marker_rotation.setRPY(0,0,0.5-M_PI);
    marker_pose.setRotation(marker_rotation);
    marker_pose.setOrigin(tf::Vector3(0,0,0.7));
    marker.add6DOFBoxMarkerTF("object_center","Object center", marker_pose);


    // Robot and scene
    moveit::planning_interface::MoveGroup group("robot");
    group.setPlannerId(plannerId);

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //moveit::planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));


    // Visualizing topic
    //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //moveit_msgs::DisplayTrajectory display_trajectory;


    group4_msgs::PointCloudPose msg;
    msg.header.frame_id = frame_id;

    int pose_id = 0;
    const double angle_step = 2* M_PI / circle_points;
    // Demo pose
    while (ros::ok())
    {
        bool success;
        double angle = M_PI + angle_step * pose_id++;

        ROS_INFO("Moving to pose %i of %i", pose_id, pose_id_max);


        tf::Pose object_center = marker.getPose("object_center");
        geometry_msgs::Pose target_pose1;
        moveit::planning_interface::MoveGroup::Plan my_plan;

        tf::Transform pose_rotation;
        tf::Quaternion pose_rotation_quaternion;
        pose_rotation_quaternion.setRPY(0,0,(double)angle);
        pose_rotation.setRotation(pose_rotation_quaternion);

        tf::Transform pose_translation;
        pose_translation.setOrigin(tf::Vector3(radius,0,0));
        pose_rotation_quaternion.setRPY(0,0,2.35619449);
        pose_translation.setRotation(pose_rotation_quaternion);

        tf::Pose goal_pose = object_center * pose_rotation * pose_translation;

        target_pose1.orientation.x = goal_pose.getRotation().getX();
        target_pose1.orientation.y = goal_pose.getRotation().getY();
        target_pose1.orientation.z = goal_pose.getRotation().getZ();
        target_pose1.orientation.w = goal_pose.getRotation().getW();
        target_pose1.position.x = goal_pose.getOrigin().getX(); // + std::sin(angle)*radius;
        target_pose1.position.y = goal_pose.getOrigin().getY(); // + std::cos(angle)*radius;
        target_pose1.position.z = goal_pose.getOrigin().getZ();

        group.setPoseTarget(target_pose1, end_effector);

//        for (int itry = 0; itry < 10; itry++)
//        {
//            success = group.plan(my_plan);
//            if (success)
//                break;
//        }


        ROS_INFO("Moving to angle %f (%f, %f, %f)", angle, target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);
        success = false;
        for (int itry = 0; itry < 3; itry++)
        {
            //success = group.execute(my_plan);
            group.setStartStateToCurrentState();
            group.setGoalJointTolerance(0.1f);
            success = group.move();
            if (success)
                break;
            else
                ROS_INFO("I got hacked to try to plan again..");
        }

        //if (success)
        if (true)
        {
            ros::Duration wait_settle(1,0);
            wait_settle.sleep();

            geometry_msgs::PoseStamped carmine_pose = group.getCurrentPose("camera_link");
            geometry_msgs::PoseStamped bumblebee_pose_left = group.getCurrentPose("bumblebee_cam1");
            geometry_msgs::PoseStamped bumblebee_pose_right = group.getCurrentPose("bumblebee_cam2");

            msg.header.stamp = ros::Time::now();
            msg.pose_id.data = pose_id;
            msg.pose_id_max.data = pose_id_max;
            msg.spin_center_pose = tfPoseToGeometryPose(object_center);
            msg.carmine_pose = carmine_pose.pose;
            msg.bumblebee_pose_left = bumblebee_pose_left.pose;
            msg.bumblebee_pose_right = bumblebee_pose_right.pose;

            cameratate_pub.publish(msg);

            ROS_INFO("Waiting 30 seconds for camera.");
            ros::Duration wait_camera(1,0);
            wait_camera.sleep();
            ROS_INFO("Moving on..");
        }
        else
            ROS_INFO("Could not move to angle %f (%f, %f, %f)", angle, target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);

        if (pose_id >= pose_id_max)
        {
            ROS_INFO("Last pose reached. Waiting 30 seconds before resetting");
            ros::Duration wait_reset(30);
            wait_reset.sleep();
            pose_id = 0;
        }
    }
    group.stop();
    return 0;
}

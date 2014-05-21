#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Tools
#include <tf_conversions/tf_kdl.h>
#include <boost/foreach.hpp>

#include <visualization_msgs/MarkerArray.h>

#include "marker_pose.h"
#include <std_msgs/Bool.h>
#include <group4_msgs/PointCloudPose.h>

#include <tf/transform_listener.h>

geometry_msgs::Pose tfPoseToGeometryPose(const tf::Pose tPose)
{
    geometry_msgs::Pose gPose;
    gPose.position.x = tPose.getOrigin().x();
    gPose.position.y = tPose.getOrigin().y();
    gPose.position.z = tPose.getOrigin().z();
    gPose.orientation.x = tPose.getRotation().x();
    gPose.orientation.y = tPose.getRotation().y();
    gPose.orientation.z = tPose.getRotation().z();
    gPose.orientation.w = tPose.getRotation().w();

    return gPose;
}

void addObjectCenterMarker(MarkerPose& marker)
{
    //MarkerPose marker("object_center");
    tf::Pose marker_pose;
    tf::Quaternion marker_rotation;
    marker_rotation.setRPY(0,0,0.5-M_PI);
    marker_pose.setRotation(marker_rotation);
    marker_pose.setOrigin(tf::Vector3(0,0,0.8));
    marker.add6DOFBoxMarkerTF("object_center","Object center", marker_pose);
    //return marker;
}

geometry_msgs::Pose tfTransformToGeometryPose(const tf::Pose& goal_pose)
{
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = goal_pose.getRotation().getX();
    target_pose1.orientation.y = goal_pose.getRotation().getY();
    target_pose1.orientation.z = goal_pose.getRotation().getZ();
    target_pose1.orientation.w = goal_pose.getRotation().getW();
    target_pose1.position.x = goal_pose.getOrigin().getX(); // + std::sin(angle)*radius;
    target_pose1.position.y = goal_pose.getOrigin().getY(); // + std::cos(angle)*radius;
    target_pose1.position.z = goal_pose.getOrigin().getZ();
    return target_pose1;
}

class NumberedPose
{
public:
    geometry_msgs::Pose pose;
    geometry_msgs::Pose markerpose;
    tf::Pose pose_tf;
    int pose_id;
    int pose_id_max;
};
typedef std::vector<NumberedPose> NumberedPoses;

NumberedPoses generatePoses(tf::Pose& object_center,const int circle_points, const double pitch_step, const double radius)
{
    std::vector<NumberedPose> poses;
    const int pose_id_max = circle_points * 3;
    int pose_id = 0;
    double angle_step = 2* M_PI / circle_points;
    double pitch_angle_base = pitch_step * -1;

    for (int altitude_pos = 1; altitude_pos < 3; altitude_pos++)
    {
        double angle_pitch = pitch_angle_base + (pitch_step * (double)altitude_pos);
        //angle_step *= -1;

        for (double yaw_pos = 0; yaw_pos < circle_points; yaw_pos++)
        {
            pose_id++;
            double angle_yaw = M_PI + angle_step * yaw_pos;
            //geometry_msgs::Pose target_pose1;
            NumberedPose pose;

            tf::Transform pose_rotation;
            tf::Quaternion pose_rotation_quaternion;
            pose_rotation_quaternion.setRPY(0,angle_pitch,angle_yaw);
            pose_rotation.setRotation(pose_rotation_quaternion);

            tf::Transform pose_translation;
            pose_translation.setOrigin(tf::Vector3(radius,0,0));
            pose_rotation_quaternion.setRPY(0,0,M_PI);
            pose_translation.setRotation(pose_rotation_quaternion);

            tf::Pose goal_pose = object_center * pose_rotation * pose_translation;
            pose.markerpose = tfTransformToGeometryPose(goal_pose);

            tf::Transform pose_orient;
            pose_orient.setOrigin(tf::Vector3(0,0,0));
            pose_rotation_quaternion.setRPY(0,0,0);
            pose_rotation_quaternion.setRPY(-M_PI/2,0,-M_PI/2);
            pose_orient.setRotation(pose_rotation_quaternion);

            pose.pose_tf = goal_pose*pose_orient;
            pose.pose = tfTransformToGeometryPose(pose.pose_tf);
            //pose.pose = pose.markerpose;

            pose.pose_id = pose_id;
            pose.pose_id_max = pose_id_max;

            poses.push_back(pose);
        }
    }

    return poses;
}

void visualizePoses(NumberedPoses& poses, const std::string frame_id,ros::Publisher& markerPublisher)
{
    if (poses.size() < 1)
            return;

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;

    // Default values
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "Markers";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    BOOST_FOREACH(NumberedPose pose, poses)
    {
        marker.pose = pose.markerpose;
        marker.id = pose.pose_id;
        markers.markers.push_back(marker);
    }

    markerPublisher.publish(markers);
}

enum altitude_t {HIGH, MID, LOW, Count};

int main(int argc, char **argv)
{
    const double radius = 0.54; // 0.5
    const double pitch_step = 0.2;
    const int circle_points = 6;
    const double plan_time = 30;
    //const int pose_id_max = circle_points * 3;
    const ros::Duration wait_settle(1.0);
    const ros::Duration wait_camera(0.5);
    const ros::Duration wait_notreached(3.0);
    const ros::Duration wait_reset(30.0);

    // Unable to construct goal representation
    //const std::string planning_group = "bumblebee"; //robot, bumblebee
    //const std::string end_effector = "bumblebee_cam1"; // bumblebee_cam1, tool_flange

    // Working with group.setGoalTolerance(0.1f); and pose.pose = pose.markerpose;
    //const std::string planning_group = "bumblebee"; //robot, bumblebee
    //const std::string end_effector = "tool_flange"; // bumblebee_cam1, tool_flange

    const std::string planning_group = "bumblebee"; //robot, bumblebee
    const std::string end_effector = "tool_flange"; // bumblebee_cam1, tool_flange

    const std::string plannerId = "PRMkConfigDefault";
    const std::string frame_id = "base_link";

    ros::init (argc, argv, "simple_pose_mission");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;
    ros::Publisher cameratate_pub = node_handle.advertise<group4_msgs::PointCloudPose>("/robot_rx60b/camerapose", 5);
    ros::Publisher markerPublisher = node_handle.advertise<visualization_msgs::MarkerArray>("/simple_pose_mission/poses", 10);

    tf::TransformListener listener;

    // Marker
    MarkerPose marker("object_center");
    addObjectCenterMarker(marker);

    // Robot and scene
    moveit::planning_interface::MoveGroup group(planning_group);
    group.setPlannerId(plannerId);
    group.setPoseReferenceFrame(frame_id);
    //group.setWorkspace(-1.5,-1.5,1.5,1.5,0,1);
    group.setGoalTolerance(0.01f);
    group.setPlanningTime(plan_time);

    group4_msgs::PointCloudPose msg;
    msg.header.frame_id = frame_id;

    tf::Pose object_center = marker.getPose("object_center");
    NumberedPoses poses = generatePoses(object_center, circle_points, pitch_step, radius);
    NumberedPoses::iterator poses_itt;
    bool poses_updated = true;


    while (ros::ok())
    {
        bool success;

        if (poses_updated)
        {
            poses_itt = poses.begin();
            poses_updated = false;
        }

        object_center = marker.getPose("object_center");
        //poses = generatePoses(object_center, circle_points, pitch_step, radius);
        visualizePoses(poses, frame_id,markerPublisher);

        NumberedPose numbered_pose = *poses_itt;

        ROS_INFO("Moving to pose %i of %i", numbered_pose.pose_id, numbered_pose.pose_id_max);



        tf::StampedTransform transform;
        try{
          listener.lookupTransform("/bumblebee_cam1", "/tool_flange",
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }

        tf::Pose p = numbered_pose.pose_tf * transform;

        geometry_msgs::Pose desired_pose;
        desired_pose = tfPoseToGeometryPose(p);
        std::cout << numbered_pose.pose;
        group.setPoseTarget(desired_pose, end_effector);
        //group.setPositionTarget(desired_pose.position.x,desired_pose.position.y,desired_pose.position.z,end_effector);
        //group.setOrientationTarget(numbered_pose.pose.orientation.x, numbered_pose.pose.orientation.y,numbered_pose.pose.orientation.z,numbered_pose.pose.orientation.w, end_effector);
        //group.setPositionTarget(0,0,0,end_effector);
        //group.setPoseReferenceFrame("object_center");


        //ROS_INFO("Planning frame %s",group.getPlanningFrame().c_str());
        //ROS_INFO("Planning pose ref frame %s", group.getPoseReferenceFrame().c_str());


        success = false;
        for (int itry = 0; itry < 2; itry++)
        {

            group.setStartStateToCurrentState();
            wait_notreached.sleep();
            success = group.move();
            if (success)
                break;
            else
                ROS_INFO("Plan unsuccessfull.. Retrying! (%i)", itry);
        }

        if (true)
        {

            wait_settle.sleep();

            geometry_msgs::PoseStamped carmine_pose = group.getCurrentPose("camera_link");
            geometry_msgs::PoseStamped bumblebee_pose_left = group.getCurrentPose("bumblebee_cam1");
            geometry_msgs::PoseStamped bumblebee_pose_right = group.getCurrentPose("bumblebee_cam2");

            msg.header.stamp = ros::Time::now();
            msg.pose_id.data = numbered_pose.pose_id;
            msg.pose_id_max.data = numbered_pose.pose_id_max;
            msg.spin_center_pose = tfPoseToGeometryPose(object_center);
            msg.carmine_pose = carmine_pose.pose;
            msg.bumblebee_pose_left = bumblebee_pose_left.pose;
            msg.bumblebee_pose_right = bumblebee_pose_right.pose;

            cameratate_pub.publish(msg);

            ROS_INFO("Waiting %f seconds for camera.", wait_camera.toSec());
            wait_camera.sleep();
        }

        if (poses_itt == poses.end())
        {
            ROS_INFO("Last pose reached. Waiting %f seconds before resetting", wait_reset.toSec());
            poses_itt = poses.begin();
            wait_reset.sleep();
        }
        else
            poses_itt++;

    }
    group.stop();
    return 0;
}

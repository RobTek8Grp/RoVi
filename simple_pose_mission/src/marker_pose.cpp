#include "marker_pose.h"

namespace vm = visualization_msgs;
namespace im = interactive_markers;

MarkerPose::MarkerPose(std::string marker_server_name) : marker_server(marker_server_name), default_frame_id("/base_link"), nh()
{
    tf_update_timer = nh.createTimer(ros::Duration(1), boost::bind(&MarkerPose::updateTFCallback,this,_1));
}

void MarkerPose::add6DOFBoxMarker(std::string name, std::string description, im::InteractiveMarkerServer::FeedbackCallback callback)
{
    vm::InteractiveMarker interactive_marker = getDefaultMarker(name, description);


    interactive_marker.pose.position.x = 0;
    interactive_marker.pose.position.y = 0;
    interactive_marker.pose.position.z = 0.7;

    addDefaultBoxMarker(interactive_marker);
    add6DOFControl(interactive_marker);

    marker_server.insert(interactive_marker);
    marker_server.setCallback(name, callback);
    marker_server.applyChanges();

    ROS_INFO("6DOF marker added");
}

void MarkerPose::add6DOFBoxMarkerTF(std::string name, std::string description, tf::Pose init_pose)
{
    vm::InteractiveMarker interactive_marker = getDefaultMarker(name, description);
    interactive_marker.pose = tfPoseToGeometryPose(init_pose);

    addDefaultBoxMarker(interactive_marker);
    add6DOFControl(interactive_marker);

    marker_pose_list[name] = init_pose;

    marker_server.insert(interactive_marker);
    marker_server.setCallback(name, boost::bind(&MarkerPose::markerFeedback,this,_1));
    marker_server.applyChanges();
}

tf::Pose MarkerPose::getPose(std::string name)
{
    return marker_pose_list[name];
}

vm::InteractiveMarker MarkerPose::getDefaultMarker(std::string name, std::string description)
{
    vm::InteractiveMarker marker;
    marker.header.frame_id = default_frame_id;
    marker.name = name;
    marker.description = description;
    marker.scale = 0.2;

    return marker;
}

vm::Marker MarkerPose::getDefaultBoxMarker()
{
    vm::Marker box_marker;
    box_marker.type = vm::Marker::CUBE;
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    return box_marker;
}

void MarkerPose::addDefaultBoxMarker(vm::InteractiveMarker &marker)
{
    vm::Marker box_marker = getDefaultBoxMarker();

    // Apply box control to
    vm::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );
    marker.controls.push_back( box_control );
}

void MarkerPose::add6DOFControl(vm::InteractiveMarker &marker)
{
    vm::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);
}

tf::Pose MarkerPose::geometryPoseToTfPose(const geometry_msgs::Pose gPose) const
{
    tf::Pose tfPose;
    tfPose.setOrigin(tf::Vector3(gPose.position.x,gPose.position.y,gPose.position.z));
    tfPose.setRotation(tf::Quaternion(gPose.orientation.x, gPose.orientation.y, gPose.orientation.z, gPose.orientation.w));
    return tfPose;
}

geometry_msgs::Pose MarkerPose::tfPoseToGeometryPose(const tf::Pose tPose) const
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

void MarkerPose::markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    //transform_pose.find(feedback->marker_name)->second = this->geometryPoseToTfPose(feedback->pose);
    marker_pose_list[feedback->marker_name] = this->geometryPoseToTfPose(feedback->pose);
    /*
    tf::Transform transform;
    tf::Pose pose = this->geometryPoseToTfPose(feedback->pose);
    transform.setOrigin(pose.getOrigin());
    transform.setRotation(pose.getRotation());
    transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), feedback->header.frame_id, feedback->marker_name));
    */
}

void MarkerPose::updateTFCallback(const ros::TimerEvent &)
{
    for (marker_pose_list_type::iterator it=marker_pose_list.begin(); it!=marker_pose_list.end(); ++it)
    {
        const std::string& marker_name = it->first;
        const tf::Pose& pose = it->second;
        transform_broadcaster.sendTransform(tf::StampedTransform(pose, ros::Time::now(), default_frame_id, marker_name));
    }
}

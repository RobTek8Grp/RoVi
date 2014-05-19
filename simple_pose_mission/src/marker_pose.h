#ifndef MARKER_POSE_H
#define MARKER_POSE_H

#include <string>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <map>

class MarkerPose
{
public:
    MarkerPose(std::string marker_server_name);
    void add6DOFBoxMarker(std::string name, std::string description, interactive_markers::InteractiveMarkerServer::FeedbackCallback callback);
    void add6DOFBoxMarkerTF(std::string name, std::string description, tf::Pose init_pose);
    tf::Pose getPose(std::string name);

protected:
    ros::NodeHandle nh;

    interactive_markers::InteractiveMarkerServer marker_server;
    std::string default_frame_id;
    tf::TransformBroadcaster transform_broadcaster;

    visualization_msgs::InteractiveMarker getDefaultMarker(std::string name, std::string description);
    visualization_msgs::Marker getDefaultBoxMarker();
    void addDefaultBoxMarker(visualization_msgs::InteractiveMarker &marker);
    void add6DOFControl(visualization_msgs::InteractiveMarker &marker);

    tf::Pose geometryPoseToTfPose(const geometry_msgs::Pose gPose) const;
    geometry_msgs::Pose tfPoseToGeometryPose(const tf::Pose tPose) const;

    // TF
    typedef std::map<std::string, tf::Pose> marker_pose_list_type;
    ros::Timer tf_update_timer;
    marker_pose_list_type marker_pose_list;
    void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void updateTFCallback(const ros::TimerEvent&);
};

#endif // MARKER_POSE_H

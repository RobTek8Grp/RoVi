#ifndef MARKER_POSE_H
#define MARKER_POSE_H

#include <string>
#include <interactive_markers/interactive_marker_server.h>

class MarkerPose
{
public:
    MarkerPose(std::string marker_server_name);
    void add6DOFBoxMarker(std::string name, std::string description, interactive_markers::InteractiveMarkerServer::FeedbackCallback callback);

protected:
    interactive_markers::InteractiveMarkerServer marker_server;
    std::string default_frame_id;

    visualization_msgs::InteractiveMarker getDefaultMarker(std::string name, std::string description);
    visualization_msgs::Marker getDefaultBoxMarker();
    void addDefaultBoxMarker(visualization_msgs::InteractiveMarker &marker);
    void add6DOFControl(visualization_msgs::InteractiveMarker &marker);
};

#endif // MARKER_POSE_H

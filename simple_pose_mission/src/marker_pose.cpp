#include "marker_pose.h"

namespace vm = visualization_msgs;
namespace im = interactive_markers;

MarkerPose::MarkerPose(std::string marker_server_name) : marker_server(marker_server_name), default_frame_id("/base_link")
{

}

void MarkerPose::add6DOFBoxMarker(std::string name, std::string description, im::InteractiveMarkerServer::FeedbackCallback callback)
{
    vm::InteractiveMarker interactive_marker = getDefaultMarker(name, description);

    addDefaultBoxMarker(interactive_marker);
    add6DOFControl(interactive_marker);

    marker_server.insert(interactive_marker);
    marker_server.setCallback(name, callback);
    marker_server.applyChanges();

    ROS_INFO("6DOF marker added");
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

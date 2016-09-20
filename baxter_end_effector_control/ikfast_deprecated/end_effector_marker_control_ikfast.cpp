
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <math.h>

using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;


// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
//interactive_markers::MenuHandler menu_handler;
ros::Publisher pose_publisher;
// %EndTag(vars)%


// %Tag(Box)%
Marker makeMarker( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::SPHERE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeMarkerControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeMarker(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  static unsigned int frameCount = 0;
  if (frameCount++ % 10 == 0) {
    return;
  }
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE: {
      Pose newPose(feedback->pose);
      Header header;
      header.frame_id = feedback->marker_name;
      header.stamp = ros::Time::now();
      PoseStamped newPoseStamped;
      newPoseStamped.pose = newPose;
      newPoseStamped.header = header;

      pose_publisher.publish(newPoseStamped);
      
    }
    break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP: {
      Pose newPose(feedback->pose);
      Header header;
      header.frame_id = feedback->marker_name;
      header.stamp = ros::Time::now();
      PoseStamped newPoseStamped;
      newPoseStamped.pose = newPose;
      newPoseStamped.header = header;

      pose_publisher.publish(newPoseStamped);
      //ROS_INFO_STREAM( s.str() << ": mouse up." );
    }
      
      break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%



////////////////////////////////////////////////////////////////////////////////////

// %Tag(6DOF)%
void make6DofMarker(std::string marker_limb, unsigned int interaction_mode, const Pose& pose, bool show_6dof)
{
  InteractiveMarker int_marker;
  //int_marker.header.frame_id = marker_limb + "_arm_mount";
  int_marker.header.frame_id = "base";
  int_marker.pose = pose;
  //tf::pointTFToMsg(tf::Vector3(1, 0, 0), int_marker.pose.position);
  //tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 0.2;

  int_marker.name = marker_limb + " marker";
  int_marker.description = marker_limb + " marker";

  // insert a box
  makeMarkerControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(6DOF)%

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;
  pose_publisher = n.advertise<geometry_msgs::PoseStamped>("end_effector_command_position", 1);
  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

  ros::Duration(0.1).sleep();

  ROS_INFO_STREAM("Initializing right marker");

  tf::TransformListener listener;
  tf::StampedTransform transform;
  while (n.ok()) {
    try {
      listener.lookupTransform("/base", "/right_gripper", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_INFO_STREAM("Initialize right marker failed, retrying...");
      ros::Duration(1.0).sleep();
      continue;
    }
    Pose rightPose;
    tf::pointTFToMsg(transform.getOrigin(), rightPose.position);
    tf::quaternionTFToMsg(transform.getRotation(), rightPose.orientation);
    make6DofMarker( "right", visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
      rightPose, true);
    break;
  }
  ROS_INFO_STREAM("Initialized right marker");

  ROS_INFO_STREAM("Initializing left marker");
  while (n.ok()) {
    try {
      listener.lookupTransform("/base", "/left_gripper", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_INFO_STREAM("Initialize marker failed, retrying...");
      ros::Duration(1.0).sleep();
      continue;
    }
    Pose leftPose;
    tf::pointTFToMsg(transform.getOrigin(), leftPose.position);
    tf::quaternionTFToMsg(transform.getRotation(), leftPose.orientation);
    make6DofMarker( "left", visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
      leftPose, true);
    break;
  }

  ROS_INFO_STREAM("Initialized left marker");

  server->applyChanges();

  ros::spin();

  server.reset();
}
// %EndTag(main)%

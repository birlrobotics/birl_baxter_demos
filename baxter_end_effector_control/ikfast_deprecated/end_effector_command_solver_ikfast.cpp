#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf/tf.h>

using namespace geometry_msgs;

ros::Publisher leftJointCommandPublisher;
ros::Publisher rightJointCommandPublisher;
tf::StampedTransform leftArmMountToBaseTransform;
tf::StampedTransform rightArmMountToBaseTransform;
tf::StampedTransform currentTransform;

void callBack(const PoseStamped &target) {
    currentTransform.setOrigin(tf::Vector3(target.pose.position.x, target.pose.position.y, target.pose.position.z));
    currentTransform.setRotation(tf::Quaternion(target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w));
    
    PoseStamped result = target;
    
    if (target.header.frame_id.find("left") != std::string::npos) {
        ROS_INFO_STREAM("left arm command recieved");
        tf::Transform targetTransform = leftArmMountToBaseTransform * currentTransform;
        tf::pointTFToMsg(targetTransform.getOrigin(), result.pose.position);
        tf::quaternionTFToMsg(targetTransform.getRotation(), result.pose.orientation);
        leftJointCommandPublisher.publish(result);
    } else {
        ROS_INFO_STREAM("right arm command recieved");
        tf::Transform targetTransform = rightArmMountToBaseTransform * currentTransform;
        tf::pointTFToMsg(targetTransform.getOrigin(), result.pose.position);
        tf::quaternionTFToMsg(targetTransform.getRotation(), result.pose.orientation);
        rightJointCommandPublisher.publish(result);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "end_effector_command_solver_ikfast");
    ros::NodeHandle n;

  tf::TransformListener listener;
  
  ROS_INFO_STREAM("Looking up transform: /right_arm_mount -> /base");
  while (n.ok()) {
    try {
      listener.lookupTransform("/right_arm_mount", "/base", ros::Time(0), rightArmMountToBaseTransform);
    } catch (tf::TransformException ex) {
      ROS_INFO_STREAM("Failed to look up transform: /right_arm_mount -> /base, retrying...");
      ros::Duration(1.0).sleep();
      continue;
    }
    ROS_INFO_STREAM("Got transform: /right_arm_mount -> /base");
    break;
  }
  
  ROS_INFO_STREAM("Looking up transform: /left_arm_mount -> /base");
  while (n.ok()) {
    try {
      listener.lookupTransform("/left_arm_mount", "/base", ros::Time(0), leftArmMountToBaseTransform);
    } catch (tf::TransformException ex) {
      ROS_INFO_STREAM("Failed to look up transform: /left_arm_mount -> /base, retrying...");
      ros::Duration(1.0).sleep();
      continue;
    }
    ROS_INFO_STREAM("Got transform: /left_arm_mount -> /base");
    break;
  }
    
    leftJointCommandPublisher = n.advertise<geometry_msgs::PoseStamped>("end_effector_left_end_command_position", 1);
    rightJointCommandPublisher = n.advertise<geometry_msgs::PoseStamped>("end_effector_right_end_command_position", 1);
    ros:: Subscriber s = n.subscribe("end_effector_command_position", 1, callBack);
    ROS_INFO_STREAM("end_effector_command_solver_ikfast subscribing...");
    ros::spin();
    return 0;
}

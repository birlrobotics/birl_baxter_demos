#!/usr/bin/env python

import argparse
import copy

import rospy
import numpy

import baxter_interface
import baxter_external_devices
import tf

from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

from baxter_core_msgs.msg import (
	JointCommand
)
from std_msgs.msg import (
	String,
	Header,
)
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose,
    PoseStamped,
)

pub = rospy.Publisher("end_effector_command_position", PoseStamped, queue_size=1)

current_limb = "right"
global_distance = 0.001
global_change_factor = 0.001
limbPose = Pose()

def initLimbPose():
	global limbPose
	global current_limb
	
	limb = baxter_interface.Limb(current_limb)
	currentPose = dict()
	while not ("position" in currentPose):
		currentPose = limb.endpoint_pose()
	
	limbPose.position = Point(
				currentPose["position"].x,
				currentPose["position"].y,
				currentPose["position"].z,
			)
	limbPose.orientation = Quaternion(
				currentPose["orientation"].x,
				currentPose["orientation"].y,
				currentPose["orientation"].z,
				currentPose["orientation"].w,
			)
	rospy.loginfo("init %s limb pose" % current_limb)
	rospy.loginfo(limbPose)

def callback(data):
	global current_limb
	global global_distance
	global limbPose
	
	command = data.data
	if not (command.find("switch") == -1):
		if (command.find("right") == -1):
			current_limb = "left"
		else:
			current_limb = "right"
		initLimbPose()
		return

	if (command == "up"):
		limbPose.position.z += global_distance
	elif (command == "down"):
		limbPose.position.z -= global_distance
	elif (command == "left"):
		limbPose.position.y += global_distance
	elif (command == "right"):
		limbPose.position.y -= global_distance
	elif (command == "backward"):
		limbPose.position.x -= global_distance
	elif (command == "forward"):
		limbPose.position.x += global_distance
	elif (command == "orientation_x"):
		limbPose.position.x -= global_distance
	elif (command == "keep"):
		pass
	elif (command == "further"):
		if (global_distance < 0.3):
			global_distance += global_change_factor
			rospy.loginfo(global_distance)
		else:
			rospy.loginfo("can not increase more")
		return
	elif (command == "closer"):
		if (global_distance > 0):
			global_distance -= global_change_factor
			if (global_distance < 0):
				global_distance = 0
			rospy.loginfo(global_distance)
		else:
			rospy.loginfo("can not decrease more")
		return
	else:
		rospy.loginfo("unknown command")
		return

	newHeader = Header()
	newHeader.frame_id = current_limb

	newHeader.stamp = rospy.Time().now()

	newPoseStamp = PoseStamped()
	newPoseStamp.pose = limbPose
	newPoseStamp.header = newHeader

	pub.publish(newPoseStamp)

def subscribe():
	initLimbPose()
	rospy.Subscriber("/end_effector_command", String, callback)
	rospy.spin();
	
def main():
	rospy.loginfo("Initializing node control_command_subscriber... ")
	rospy.init_node("ikfast_transform", anonymous=True)

	try:
		subscribe()
	except():
		pass

if __name__ == '__main__':
	main()

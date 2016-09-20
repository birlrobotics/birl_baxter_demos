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

commandPositionPublisher = rospy.Publisher("end_effector_command_position", PoseStamped, queue_size=1)

checked = True
def commandCheckCallback(data):
	global checked
	if not (data.data.find("done") < 0):
		#rospy.loginfo("command checked")
		checked = True

def callback(data):
	global checked

	if not checked:
		rospy.loginfo("command is not checked, reject...")
		return
	checked = False
	commandPositionPublisher.publish(data)
	

def subscribe():
	rospy.Subscriber("/end_effector_command_pose_stamped", PoseStamped, callback)
	rospy.Subscriber("/end_effector_command_check", String, commandCheckCallback)
	rospy.spin();
	
def main():
	rospy.init_node("ikfast_transform", anonymous=True)

	try:
		subscribe()
	except():
		pass

if __name__ == '__main__':
	main()

#!/usr/bin/env python

import argparse
import copy

import rospy
import numpy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

from baxter_core_msgs.msg import (
	JointCommand
)
from std_msgs.msg import (
	String,
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
)
import collections
Point = collections.namedtuple('Point', ['x', 'y', 'z'])
Quaternion = collections.namedtuple('Quaternion', ['x', 'y', 'z', 'w'])

commandSolutionPublisher = rospy.Publisher("end_effector_command_solution", JointCommand, queue_size=1)
commandCheckPublisher = rospy.Publisher("end_effector_command_check", String, queue_size=1)

def callback(data):
	#time_now = rospy.Time().now()
	#if (time_now.secs - data.header.stamp.secs > 1):
	#	rospy.loginfo("time stamp too old, ignore...")
	#	return

	commandPose = data.pose
	limb_name = data.header.frame_id;
	limb = baxter_interface.Limb(limb_name)
	newPose = dict()
	newPose = {
		'position': Point(
				commandPose.position.x,
				commandPose.position.y,
				commandPose.position.z,
			),
		'orientation': Quaternion(
				commandPose.orientation.x,
				commandPose.orientation.y,
				commandPose.orientation.z,
				commandPose.orientation.w,
			),
	}
	kinematics = baxter_kinematics(limb_name)
	inverseKinematics = kinematics.inverse_kinematics(newPose["position"], newPose["orientation"])

	if not (inverseKinematics == None):
		inverseKinematicsSolution = list()
		for num in inverseKinematics.tolist():
			inverseKinematicsSolution.append(num)
		inverseKinematicsSolutionJointCommand = JointCommand()
		inverseKinematicsSolutionJointCommand.mode = 1
		inverseKinematicsSolutionJointCommand.names = limb.joint_names()
		inverseKinematicsSolutionJointCommand.command = inverseKinematicsSolution
		commandSolutionPublisher.publish(inverseKinematicsSolutionJointCommand)
	
	# publish to command subscriber to check compute
	commandCheckPublisher.publish(String("done"))

def subscribe():
	rospy.Subscriber("end_effector_command_position", PoseStamped, callback)
	#rospy.loginfo("end_effector_command_position subscribing...")
	rospy.spin();


def main():
	#rospy.loginfo("Initializing node end_effector_command_solver... ")
	rospy.init_node("end_effector_command_solver", anonymous=True)

	try:
		subscribe()
	except():
		pass

if __name__ == '__main__':
	main()

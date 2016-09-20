#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import baxter_interface
from baxter_interface import CHECK_VERSION
import sys
class face_manger(object):
	"""docstring for face_manger"""
	def __init__(self):
		self.m1 = rospy.Publisher("face_s1_manager",String,queue_size=10)
	def face_s1_publish(self,msg):
		rospy.loginfo(msg)
		self.m1.publish(msg)
class action_manager(object):
	def __init__(self):
		self.m1 = rospy.Publisher("action_s1_manager",String,queue_size=10)
		self.m2 = rospy.Publisher("action_s2_manager",String,queue_size=10)
		self.m3 = rospy.Publisher("action_s3_manager",String,queue_size=10)
		self.m0 = rospy.Publisher("face_s0_manager",String,queue_size=10)
	def action_s1_publish(self,msg):
		rospy.loginfo(msg)
		self.m1.publish(msg)
	def action_s2_publish(self,msg):
		rospy.loginfo(msg)
		self.m2.publish(msg)
	def action_s3_publish(self,msg):
		rospy.loginfo(msg)
		self.m3.publish(msg)
	def face_s0_publish(self,msg):
		rospy.loginfo(msg)
		self.m0.publish(msg)
def  main():
	rospy.init_node("manager")
	face = face_manger()
	action = action_manager()
	raw_input("\nPress the enter key to continue stage0.")
	action.face_s0_publish('stage0')
	
	raw_input("\nPress the enter key to continue stage1.")
	# face.face_s1_publish('stage1')
	action.action_s1_publish('stage1')

	raw_input("\nPress the enter key to continue stage2.")
	action.action_s2_publish('stage2')
	
	raw_input("\nPress the enter key to continue stage3.")
	action.action_s3_publish('stage3')
	rospy.spin()


if __name__ == '__main__':
	 sys.exit(main())

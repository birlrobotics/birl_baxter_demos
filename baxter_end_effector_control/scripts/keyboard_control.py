#!/usr/bin/env python

import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

from std_msgs.msg import (
	String,
)

def map_keyboard():
	pub = rospy.Publisher("end_effector_command", String, queue_size=10);
	keyboard_binding = {}
	keyboard_binding["w"] = "up"
	keyboard_binding["s"] = "down"
	keyboard_binding["a"] = "left"
	keyboard_binding["d"] = "right"
	keyboard_binding["q"] = "backward"
	keyboard_binding["e"] = "forward"
	keyboard_binding["f"] = "switch limb "
	keyboard_binding["z"] = "orientation_x"
	keyboard_binding[" "] = "keep"
	keyboard_binding["k"] = "further"
	keyboard_binding["l"] = "closer"
	limb = ["right", "left"]
	current_limb = 0
	rospy.loginfo(limb[current_limb] + " limb under control...")
	rospy.loginfo('press ? to print help')
	while not rospy.is_shutdown():
		c = baxter_external_devices.getch()
		if c:
			#catch Esc or ctrl-c
			if c in ['\x1b', '\x03']:
				rospy.signal_shutdown("Finished.Exiting...")
				return
			
			if c == '?':
				printHelper(keyboard_binding)
			
			if(c in keyboard_binding.keys()):
				if (c == "f"):
					current_limb = 1 - current_limb
					rospy.loginfo("control switch to " + limb[current_limb])
					pub.publish(String(keyboard_binding[c] + limb[current_limb]))
				else:
					rospy.loginfo("sending command: " + limb[current_limb] + " limb " + keyboard_binding[c])
					pub.publish(String(keyboard_binding[c]))
			else:
				rospy.loginfo("invalid command: " + c)
				rospy.loginfo('press ? to print help')


def main():
	#rospy.loginfo("Initializing node keyboard_control... ")
	rospy.init_node("keyboard_control")

	try:
		map_keyboard()
	except():
		pass
	#rospy.loginfo("Done.")
	
def printHelper(keyboard_binding):
	print 'key maping:'
	for c in keyboard_binding.keys():
		print c + ' ---> ' + keyboard_binding[c]

if __name__ == '__main__':
	main()

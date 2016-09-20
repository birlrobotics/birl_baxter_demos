#!/usr/bin/env python

"""
    face_follower.py - Version 3.0 2014-11-18

    This node make the baxter's face follow yours. It seems to keep looking at you.

    Before running this node. You have to open the camera on the head of baxter.
    The author recommend using a resolution equal or less than 640x400 to ensure
    a fast enough image processing. But a higher resolution can make the face
    recognition more precise.

    Subscibe: /cameras/head_camera/camera_info (remap from /camera_info)
              /roi
    Publish : None

"""


import roslib; roslib.load_manifest('demos_sysu_baxter')
import rospy
from sensor_msgs.msg import RegionOfInterest, CameraInfo
from baxter_interface import head

class FaceFollower():
    def __init__(self):
        rospy.init_node("face_follower")
                
        # Set the shutdown function
        rospy.on_shutdown(self.shutdown)
        
        # The tate of updating the robot's motion
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)
        
        # The maximum rotation speed (speed range 0-100)
        self.max_speed = rospy.get_param("~max_speed", 100)
        
        # The minimum rotation speed (speed range 0-100)
        self.min_speed = rospy.get_param("~min_speed", 5)
        
        # Use PD Algorithm to control the robot
        self.dist_gain = rospy.get_param("~dist_gain", 2.0)
        self.speed_Kp = rospy.get_param("~speed_Kp", 2.0)
        self.speed_Kd = rospy.get_param("~speed_Kd", 0)
        
        # Only do some action if the displacement exceeds the threshold
        # This can reduce shaking (in percentage format)
        self.x_threshold = rospy.get_param("~x_threshold", 0.01)
        
        # Intialize the head movement command
        self.head_ = head.Head()
        self.pan_distance_ = 0
        self.pan_speed_ = 10
        self.pre_pan_speed_ = 0
        rospy.loginfo("Turning the head to the exactly middle...")
        self.head_.set_pan(0, 10, 10.0)
        
        # Get the image width and height from the camera_info topic
        self.image_width = 0
        self.image_height = 0
        
        # Set flag to indicate when the ROI stops updating
        self.target_visible = False
        
        # Wait for the camera_info topic to become available
        rospy.loginfo("Waiting for camera_info topic...")
        rospy.wait_for_message("camera_info", CameraInfo)
        
        # Subscribe the camera_info topic to get the image width and height
        rospy.Subscriber("camera_info", CameraInfo, self.get_camera_info)
        

        # Wait until we actually have the camera data
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)
                    
        # Subscribe to the ROI topic and set the callback to update the robot's motion
        rospy.Subscriber('roi', RegionOfInterest, self.PD_cal)
        
        # Wait until ROI is detected
        rospy.wait_for_message('roi', RegionOfInterest)
        rospy.loginfo("ROI messages detected. Starting follower...")
        
        # Begin the following loop
        while not rospy.is_shutdown():
            # If the target is not visible, stop the robot
            if not self.target_visible:
                rospy.loginfo("No visible target!")
            else:
                # Reset the flag to False by default
                self.target_visible = False
                # Output data for debug
                rospy.loginfo("Following the targrt! Moving head to location %f in speed %d", self.pan_distance_, self.pan_speed_)

            self.head_.set_pan(self.pan_distance_, self.pan_speed_, 10.0)
            
            # Sleep for 1/self.rate seconds
            r.sleep()

    def PD_cal(self, msg):
        # If the ROI has a width or height of 0, we have lost the target
        if msg.width == 0 or msg.height == 0:
            return
        
        # If the ROI stops updating this, the robot will stay the same
        self.target_visible = True

        # Compute the displacement of the ROI from the center of the image
        target_offset_x = msg.x_offset + msg.width / 2 - self.image_width / 2

        try:
            percent_offset_x = float(target_offset_x) / (float(self.image_width) / 2.0)
        except:
            percent_offset_x = 0

        # Rotate the head only if the displacement of the target exceeds the threshold
        if abs(percent_offset_x) > self.x_threshold:
            # Calculate the pan distance by using P gain only
            self.pan_distance_ -= self.dist_gain * percent_offset_x
            # Calculate the pan speed by using PD Algorithm
            try:
                speed = self.speed_Kp * abs(percent_offset_x) - self.speed_Kd * (self.pan_speed_ - self.pre_pan_speed_)
                if speed < 0:
                    direction = -1
                else:
                    direction = 1
                # Constrain the speed in range [min_speed, max_speed]
                self.pan_speed_ = abs(direction * max(self.min_speed, min(self.max_speed, abs(speed))))
                # Store the speed as a previous one for next calculation
                self.pre_pan_speed_ = self.pan_speed_
            except:
                self.pan_speed_ = self.min_speed
                self.pre_pan_speed_ = self.min_speed
            #rospy.loginfo("percent_offset_x=%f,speed=%f,pre_pan_speed_=%d", percent_offset_x, speed, self.pre_pan_speed_)
        else:
            # Otherwise stop the robot slowly. Here cannot set pan_distance_ to 0!
            self.pan_speed_ = 10

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)     

if __name__ == '__main__':
    try:
        FaceFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Face following node terminated.")


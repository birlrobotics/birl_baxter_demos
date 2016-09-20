#!/usr/bin/env python

"""
  voice_nav.py - Version 1.1 2013-12-20
  
  Allows controlling a mobile base using simple speech commands.
  
  Based on the voice_cmd_vel.py script by Michael Ferguson in
  the pocketsphinx ROS package.
  
  See http://www.ros.org/wiki/pocketsphinx
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign
from sound_play.libsoundplay import SoundClient
import os
import sys
import argparse

import rospy

import cv2
import cv_bridge

from sensor_msgs.msg import (
    Image,
)

def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)


def talkback(self, word):
    # Print the recognized words on the screen
    rospy.loginfo(word)
    # Speak the recognized words in the selected voice
    self.soundhandle.say(word, self.voice)


class VoiceNav:
    def __init__(self):
        rospy.init_node('voice_nav')
        
        rospy.on_shutdown(self.cleanup)
        
        # We don't have to run the script very fast
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        
        # A flag to determine whether or not voice control is paused
        self.paused = False

        # Create the sound client object
        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        self.soundhandle = SoundClient()

        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)
        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'hello': ['hello', 'hello baxter', 'hi'],
                                    'old': ['old', 'how old','how'],
                                    'where': ['where', 'where are','from'],
                                    'good': ['good', 'good bye', 'bye'],
                                    'sport': ['what is', 'sport'],
                                    'difficult': ['difficult']}
        
        rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel)
            r.sleep()                       
            
    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speech_callback(self, msg):
        # Get the motion command from the recognized phrase
        command = self.get_command(msg.data)
        
        # Log the command to the screen
        rospy.loginfo("Command: " + str(command))
       
        if command == 'hello':
            back = 'hello'
            talkback(self, back)
            send_image(sys.path[0] + '/pic/1.png')
            #reaction_speak('Greetings Humans. Take me to your leader.')
            pypath = "python " + sys.path[0] + '/polite.py' + " -f " + sys.path[0] + "/pro/shake_hand.dat"
            os.system(pypath)

        

        elif command == 'old':    
            back = 'i am one year old'
            talkback(self, back)
            send_image(sys.path[0] + '/pic/old.jpg')

        
        elif command == 'good':  
            back = 'bye bye'
            talkback(self, back)
            send_image(sys.path[0] + '/pic/goodbye.jpg')
            pypath = "python " + sys.path[0] + '/polite.py' + " -f " + sys.path[0] + "/pro/greet.dat"
            os.system(pypath)

        elif command == 'sport':
            back = 'my favorite sport is basketball'
            talkback(self, back)
            send_image(sys.path[0] + '/pic/basketball.jpg')

        else:
            return
            

        """elif command == 'where':
            send_image(path)
        """
                
        
                

    def cleanup(self):
        # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)

if __name__=="__main__":
    try:
        VoiceNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice navigation terminated.")


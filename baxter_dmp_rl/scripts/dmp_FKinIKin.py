#!/usr/bin/python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#-----------------------------------------
# Overview:
#-----------------------------------------
# Requires the installation of baxter_PyKDL found at: http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL
# Example class on how to call kinematics and move the arm.

#-----------------------------------------
# Imports
#-----------------------------------------
# To enable debug, uncomment the next line.
import ipdb
import os                                          # used to clear the screen
import math
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
#from rbx1_nav.transform_utils import quat_to_angle # Convert quaternions to euler

#-----------------------------------------
# Local Methods
#-----------------------------------------
def enable_Baxter():
    # Enable the robot's arms
    print("Getting robot state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state=rs.state().enabled
    print("Enabling robot...")
    rs.enable() 
    return rs

def shutdown():
    rospy.loginfo("Node has been terminated. Closing gracefully.")
    rospy.sleep(10)

def main():

    # If you want to debug, uncomment the next line.
    ipdb.set_trace()

    # Initialize node
    rospy.init_node('example_baxter_kins_right')

    # Call routine to enable the robot
    rs=enable_Baxter()
    
    # Set a ROS Rate for the while loop at 1Hz
    loop=rospy.Rate(5) 

    # Create Limb Objects
    rLimb=baxter_interface.Limb('right')
    lLimb=baxter_interface.Limb('left')

    # Create Kinematic Objects
    rKin = baxter_kinematics('right')
    lKin = baxter_kinematics('left')

    # Angles and Kinematics
    print 'Printing joing angles from baxter_interface, fkine, ikine. Please move baxter\'s joints'
    
    # Right Arm
    while not rospy.is_shutdown():
        print 'Right Arm'.center(35,'*')
        
        print 'Joint Angles from Baxter Interface'
        rAngles=rLimb.joint_angles()               # Angles
        print 's0\ts1\te0\te1\tw0\tw1\tw2'          # Angle sequence
        print (
            round(rAngles['right_s0'],4), round(rAngles['right_s1'],4), 
            round(rAngles['right_e0'],4), round(rAngles['right_e1'],4), 
            round(rAngles['right_w0'],4), round(rAngles['right_w1'],4), 
            round(rAngles['right_w2'],4)
        )
        print '' # Empty line

        print 'Pose from FKins...'
        pose=rKin.forward_position_kinematics() # Fkine
        print (
            'x:  ',round(pose[0],2),
            'y:  ',round(pose[1],2),
            'z:  ',round(pose[2],2),
            'vx: ',round(pose[3],2),
            'vy: ',round(pose[4],2),
            'vz: ',round(pose[5],2),
            'w:  ',round(pose[6],2),
            )
        print ''

        # Get End pose, without orientation
        print 'Cartesian End-Effector Pose'
        rPose = rLimb.endpoint_pose()
        print (
            'x:  ',round(rPose['position'][0],2),
            'y:  ',round(rPose['position'][1],2),
            'z:  ',round(rPose['position'][2],2),
            'vx: ',round(rPose['orientation'][0],2),
            'vy: ',round(rPose['orientation'][1],2),
            'vz: ',round(rPose['orientation'][2],2),
            'w:  ',round(rPose['orientation'][3],2),
            )
        print ''
        #print 'The RPY is: ', quat_to_angle(Quaternion(*orientation))

        print 'Joint Angles from IKins...'
        rIKin=rKin.inverse_kinematics(rPose['position'],rPose['orientation']) 
        print rIKin
        print round(rIKin[0],4),round(rIKin[1],4),round(rIKin[2],4),round(rIKin[3],4),round(rIKin[4],4),round(rIKin[5],4),round(rIKin[6],4)
        print ''

        # Set the loop speed
        loop.sleep()

        # Clear the screen for easier reading
        os.system('clear')

    # Set rospy to execute a shutdown function when exiting
    rospy.on_shutdown(shutdown)
    return rs                                                 


    # Jacobian
    #print '\n*** Baxter Jacobian ***\n'
    #print '\nUse Jacobian to move arm'
    #print kin.jacobian()
    # Jacobian Transpose
    #print '\n*** Baxter Jacobian Tranpose***\n'
    #print kin.jacobian_transpose()
    # Jacobian Pseudo-Inverse (Moore-Penrose)
    #print '\n*** Baxter Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
    #print kin.jacobian_pseudo_inverse()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.loginfo("example_baxter_kins_right node terminated.")        

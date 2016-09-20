#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
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

"""
Baxter RSDK Joint Trajectory Action Client Example
"""
import argparse
import sys

import rospy
import actionlib

import pdb
from copy import copy

# Action modules: gripper and trajectory
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

# Open and Close grippers
import pa_closeHand
import pa_openHand

# Pose Stamped and Transformation
from geometry_msgs.msg import (
    PoseStamped,
    Point,
)

from rbx1_nav.transform_utils import quat_to_angle

# Kinematics
from baxter_pykdl import baxter_kinematics
import PyKDL
from math import pi

# Baxter Messages
from baxter_core_msgs.msg import EndpointState

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
) 

# Flags
malePickup=0
assembly  =1

# Baxter Stuff
import baxter_interface
from baxter_interface import CHECK_VERSION

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient( # create simple client w/ topic and msg type
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal() 		# trajectory(header/points);
								# path_tolerance
                                                                # goal_tolerance 
                                                                # goal_time_tolerance
        self._goal_time_tolerance = rospy.Time(0.1) 		# Reach goal within this tolerance of final time.
        self._goal.goal_time_tolerance = self._goal_time_tolerance # Assign to goal object. 
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0)) # Connect to server within this time. 
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


def main():
    """RSDK Joint Trajectory Example: Simple Action Client

    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.

    Make sure to start the joint_trajectory_action_server.py
    first. Then run this example on a specified limb to
    command a short series of trajectory points for the arm
    to follow.
    """

    arg_fmt = argparse.RawDescriptionHelpFormatter # create ArgumentParser object
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments') # set required strings
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:]) # return objects
    
    # Set limb and gripper side
    #limb = args.limb
    limb='right'
    #gripper = args.limb

    print("Initializing node... ")
    rospy.init_node("PivotApproach_trajectory_client_%s" % (limb,))

    # Create Kinematic Objects
    kin = baxter_kinematics(limb)

    # Get robot State
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    
    # Create Joint Names List
    jNamesl=['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
      
    # Get Current Joint Positions first and command them
    arm = baxter_interface.limb.Limb(limb)    
    current_angles = [arm.joint_angle(joint) for joint in arm.joint_names()]

    # Create open and close gripper object
    pc=pa_closeHand.GripperClient(limb)
    po=pa_openHand.GripperClient(limb)
    pdb.set_trace()

    ################################# State Machine############################
    #--------------- Get Starting and Reference positions -----------------------
    rospy.loginfo('Starting pivot approach demo....')
    rospy.loginfo('Opening the gripper....')
    po.open()

    key=raw_input('When you are ready to close, press a key: \n')
    pc.close()

    rospy.loginfo('Lets capture the starting pose  of the robot')
    # TODO: Could call example_goHome()

    startPose=arm.endpoint_pose()
    startJoints=kin.inverse_kinematics(startPose['position'],startPose['orientation']).tolist()
    startPose_=dict(zip(jNamesl,startJoints))

    # Record reference position
    rospy.loginfo('Now please move to the reference location.\n Open a new terminal and use keyboard teleoperation: roslaunch baxter_end_effector_control end_effector_controarm.launch keyboard:=true')
    key=raw_input('When you have finished, pres any key. Tthen I will record this as the reference location, after that the assembly should run automatically: \n')

    referencePose=arm.endpoint_pose()
    referenceJoints=kin.inverse_kinematics(referencePose['position'],referencePose['orientation']).tolist()
    referencePose_=dict(zip(jNamesl,referenceJoints))
    
    # Save to file
    # file_=open('pa_refPos','w')
    # foreach(var i in referencePose.ToArray())
    #     file_.write(referencePose[i.position]=i.Value.Trim())
    # file_.close()

    #------------------ Compute Goal Position ---------------------------------------
    # Convert ortientation to RPY using PyKDL
    qref=referencePose['orientation']
    rot_mat=PyKDL.Rotation.Quaternion(qref.x,qref.y,qref.z,qref.w)
    #rot=rot_mat.GetRPY()
    # The idea is to set the roll to -pi and the pitch to 0. Keep the yaw
    #rot[0]=-pi, rot[1]=0
    rot_mat[0,0]=-1; rot_mat[1,0]=0; rot_mat[2,0]=0
    rot_mat[0,1]=0;  rot_mat[1,1]=1; rot_mat[2,1]=0
    q_goal=rot_mat.GetQuaternion()
    
    # Create the dictionary
    goalPose=referencePose
    # Change the orientation
    goalPose['orientation']=q_goal

    goalJoints=kin.inverse_kinematics(goalPose['position'],goalPose['orientation']).tolist()
    goalPose_=dict(zip(jNamesl,goalJoints))

    #----------------------------------- Approach # 1
    # Add 5 cm to referene pose
    approach1Pose=referencePose
    _x=referencePose['position'][0];
    _y=referencePose['position'][1];
    _z=referencePose['position'][2]+0.05
    del_z=baxter_interface.limb.Limb.Point(_x,_y,_z)
    
    approach1Pose['position']=del_z
    approach1Joints=kin.inverse_kinematics(approach1Pose['position'],approach1Pose['orientation']).tolist()
    approach1Pose_=dict(zip(jNamesl,approach1Joints))


    #----------------------------------- Approach # 2
    # Add 1 cm to referene pose
    approach2Pose=referencePose

    _z=referencePose['position'][2]+0.01
    del_z=baxter_interface.limb.Limb.Point(_x,_y,_z)
    
    approach2Pose['position']=del_z
    approach2Joints=kin.inverse_kinematics(approach2Pose['position'],approach2Pose['orientation']).tolist()
    approach2Pose_=dict(zip(jNamesl,approach2Joints))


    ############################## Moving to Points ####################################
    #1. Approach 0
    arm.move_to_joint_positions(startPose_)
    rospy.sleep(8)

    #2. Appraoch 1
    arm.move_to_joint_positions(approach1Pose_)
    rospy.sleep(1)

    #3. Appraoch 2
    arm.move_to_joint_positions(approach2Pose_)
    rospy.sleep(5)

    #4. Reference 
    arm.move_to_joint_positions(referencePose_)
    rospy.sleep(5)

    #5. Goal
    arm.move_to_joint_positions(goalPose_)
    rospy.sleep(5)
    po.open()

    # Move Back
    arm.move_to_joint_positions(startPose_)
    rospy.sleep(8)

    ## Reference: manually input from teleoperation
    # _pose=EndpointState()
    # _pose.pose.position.x=    0.55686099076
    # _pose.pose.position.y=   -0.296882237511
    # _pose.pose.poition.z=    -0.177124326921
    # _pose.pose.orientation.x=-0.00928787935556
    # _pose.pose.orientation.y= 0.989519808056
    # _pose.pose.orientation.z=-0.00726430566722
    # _pose.pose.orientation.w=-0.143914956223

    print("Exiting: Planner")

if __name__ == "__main__":
    main()

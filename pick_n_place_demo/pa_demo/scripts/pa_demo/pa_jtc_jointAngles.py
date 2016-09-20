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

#import pdb
from copy import copy

# Action modules: gripper and trajectory
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

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

# Grippper Control
#import gripper_action as gc

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
    limb = args.limb
    #gripper = args.limb

    print("Initializing node... ")
    rospy.init_node("PivotApproach_trajectory_client_%s" % (limb,))

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    
    # Create Joint Names List
    jNamesl=['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
      
    # Get Current Joint Positions first and command them
    l = baxter_interface.limb.Limb(limb)    
    current_angles = [l.joint_angle(joint) for joint in l.joint_names()]

    #  Consideration: Update error limits on joint_action_server
    # 1. Starting position
    home=dict(zip(jNamesl,current_angles))
    l.move_to_joint_positions(home)
    rospy.sleep(8)
    
    # 1. Midpoint    
    pos = [-0.36777189347534184, -0.4590437502502442, -0.562587453314209, 1.4381069869995118, 0.7804127249450684, 0.8659321537719727, -0.43219908649291994]
    ref=dict(zip(jNamesl,pos))
    l.move_to_joint_positions(ref)    

    if malePickup:
        # 2. Pre-Grabbing Male Part
        pos = [-0.4371845240478516, -0.2665291615905762, -0.5322913327880859, 1.3468351302246095, 0.9161700245178224, 0.704097180834961, -0.48205346204223637]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) #set_joint_positions(ref)    
        rospy.sleep(1)
    
    #    # 3. Grab Male Part
        pos = [-0.6948932961181641, -0.18944662708740237, -0.15378157380981447, 1.0757040262756348, 0.24006799302978518, 0.7769612681762695, -0.1702718672607422] # works ok. trying to improve
        #pos = [-0.6335340646728516, -0.1940485694458008, -0.2550243056945801, 1.121339954663086, 0.4237621921691895, 0.7481991284362793, -0.19711653101806642]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(5)
    
        # Close Gripper 
        #gc = gc.(gripper)
        #gc.command(position=35.0, effort=0.0)
    
        # 4. Come Up
        pos = [-0.8486748699279786, -0.45559229348144537, -0.03834951965332031, 1.5174904926818849, 0.1967330358215332, 0.7539515563842774, -0.19865051180419924]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(1)

    # 5. Tilt
    pos = [-0.7604709747253419, -0.3524320856140137, -0.05138835633544922, 1.4032089241149903, 0.08858739039916992, 0.694509800921631, -0.053689327514648444]
    ref=dict(zip(jNamesl,pos))
    l.move_to_joint_positions(ref) 
    rospy.sleep(1)

    # 7. Precontact point
    pos = [-0.7650729170837403, -0.3413107249145508, -0.05138835633544922, 1.4039759145080568, 0.08858739039916992, 0.6937428105285645, -0.05407282271118164]
    ref=dict(zip(jNamesl,pos))
    l.move_to_joint_positions(ref) 
    rospy.sleep(1)

    if assembly:
        # 8. Contact Point    
        pos = [-0.7639224314941406, -0.32482043146362305, -0.051004861138916016, 1.3859516402709962, 0.08897088559570313, 0.6979612576904297, -0.053689327514648444]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(1)
        
        # 9. Align and Insert
        pos = [-0.7547185467773438, -0.3815777205505371, -0.04985437554931641, 1.5508545747802736, 0.09203884716796876, 0.4667136541809082, -0.05560680349731446]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(10)
        
        # Open Gripper 
        #gc = gc.(gripper)
        #gc.command(position=100.0, effort=0.0)        
        
    # 10. Come Up
    pos=[-0.8636311825927735, -0.5472476454528808, 0.08053399127197267, 1.691213816711426, -0.11965050131835939, 0.47898550046997074, -0.005368932751464844]
    ref=dict(zip(jNamesl,pos))
    l.move_to_joint_positions(ref) 
    rospy.sleep(1)
    
    # 11. Go Home
    l.move_to_joint_positions(home)  
    print("Exiting: planner")

if __name__ == "__main__":
    main()

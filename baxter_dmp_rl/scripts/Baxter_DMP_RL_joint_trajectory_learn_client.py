#!/usr/bin/env python

import argparse
import operator
import sys
import threading

from bisect import bisect
from copy import copy
from os import path
import rospy
import actionlib
# import msg
from std_msgs.msg import Float32
import geometry_msgs
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
# import baxter SDK
import baxter_interface
from baxter_interface import CHECK_VERSION
# import RL agent
from agent import  Agent, LearningAgent
# import debug
import ipdb

# DMP class
class Trajectory(object):
    def __init__(self):
        # create our action server clients
        self._right_client = actionlib.SimpleActionClient(
            'robot/limb/right/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        # verify joint trajectory action servers are available
        r_server_up = self._right_client.wait_for_server(rospy.Duration(10.0))
        if not r_server_up:
            msg = ("Action server not available."
                   " Verify action server availability.")
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)
        # create our goal request
        self._r_goal = FollowJointTrajectoryGoal()
        # limb interface - current angles needed for start move
        self._r_arm = baxter_interface.Limb('right')
        # gripper interface - for gripper command playback
        # flag to signify the arm trajectories have begun executing
        self._arm_trajectory_started = False
        # reentrant lock to prevent same-thread lockout
        self._lock = threading.RLock()
        # Verify Grippers Have No Errors and are Calibrated
        # gripper goal trajectories
        # Timing offset to prevent gripper playback before trajectory has started
        self._slow_move_offset = 0.0
        self._trajectory_start_offset = rospy.Duration(0.0)
        self._trajectory_actual_offset = rospy.Duration(0.0)

        # param namespace
        self._param_ns = '/rsdk_joint_trajectory_action_server/'

        # gripper control rate
        self._gripper_rate = 20.0  # Hz

    def _execute_gripper_commands(self):
        start_time = rospy.get_time() - self._trajectory_actual_offset.to_sec()
        r_cmd = self._r_grip.trajectory.points
        pnt_times = [pnt.time_from_start.to_sec() for pnt in r_cmd]
        end_time = pnt_times[-1]
        rate = rospy.Rate(self._gripper_rate)
        now_from_start = rospy.get_time() - start_time
        while(now_from_start < end_time + (1.0 / self._gripper_rate) and
              not rospy.is_shutdown()):
            idx = bisect(pnt_times, now_from_start) - 1

            rate.sleep()
            now_from_start = rospy.get_time() - start_time

    def _clean_line(self, line, joint_names):
        """
        Cleans a single line of recorded joint positions

        @param line: the line described in a list to process
        @param joint_names: joint name keys

        @return command: returns dictionary {joint: value} of valid commands
        @return line: returns list of current line values stripped of commas
        """
        def try_float(x):
            try:
                return float(x)
            except ValueError:
                return None
        # convert the line of strings to a float or None
        line = [try_float(x) for x in line.rstrip().split(',')]
        # zip the values with the joint names
        combined = zip(joint_names[1:], line[1:])
        # take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        # convert it to a dictionary with only valid commands
        command = dict(cleaned)
        return (command, line,)

    def _add_point(self, positions, side, time):
        """
        Appends trajectory with new point

        @param positions: joint positions
        @param side: limb to command point
        @param time: time from start for point in seconds
        """
        # creates a point in trajectory with time_from_start and positions
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        if side == 'right':
            self._r_goal.trajectory.points.append(point)
        elif side == 'right_gripper':
            self._r_grip.trajectory.points.append(point)

    def parse_file(self, filename):
        """
        Parses input file into FollowJointTrajectoryGoal format

        @param filename: input filename
        """
        # open recorded file
        with open(filename, 'r') as f:
            lines = f.readlines()
        # read joint names specified in file
        joint_names = lines[0].rstrip().split(',')
        # parse joint names for the left and right limbs
        for name in joint_names:
            if 'right' == name[:-3]:
                self._r_goal.trajectory.joint_names.append(name)

        def find_start_offset(pos):
            # create empty lists
            cur = []
            cmd = []
            dflt_vel = []
            vel_param = self._param_ns + "%s_default_velocity"
            # for all joints find our current and first commanded position
            # reading default velocities from the parameter server if specified
            for name in joint_names:
                if 'right' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._r_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
            diffs = map(operator.sub, cmd, cur)
            diffs = map(operator.abs, diffs)
            # determine the largest time offset necessary across all joints
            offset = max(map(operator.div, diffs, dflt_vel))
            return offset

        for idx, values in enumerate(lines[1:]):
            # clean each line of file # lines[1:] record all the data except the first line
            cmd, values = self._clean_line(values, joint_names)
            # find allowable time offset for move to start position
            if idx == 0:
                # Set the initial position to be the current pose.
                # This ensures we move slowly to the starting point of the
                # trajectory from the current pose - The user may have moved
                # arm since recording
                cur_cmd = [self._r_arm.joint_angle(jnt) for jnt in self._r_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'right', 0.0)
                start_offset = find_start_offset(cmd)
                # Gripper playback won't start until the starting movement's
                # duration has passed, and the actual trajectory playback begins
                self._slow_move_offset = start_offset
                self._trajectory_start_offset = rospy.Duration(start_offset + values[0])
            # add a point for this set of commands with recorded time
            cur_cmd = [cmd[jnt] for jnt in self._r_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'right', values[0] + start_offset)



    def _feedback(self, data):
        # Test to see if the actual playback time has exceeded
        # the move-to-start-pose timing offset
        if (not self._get_trajectory_flag() and
              data.actual.time_from_start >= self._trajectory_start_offset):
            self._set_trajectory_flag(value=True)
            self._trajectory_actual_offset = data.actual.time_from_start

    def _set_trajectory_flag(self, value=False):
        with self._lock:
            # Assign a value to the flag
            self._arm_trajectory_started = value

    def _get_trajectory_flag(self):
        temp_flag = False
        with self._lock:
            # Copy to external variable
            temp_flag = self._arm_trajectory_started
        return temp_flag

    def start(self):
        """
        Sends FollowJointTrajectoryAction request
        """

        self._right_client.send_goal(self._r_goal, feedback_cb=self._feedback)
        # Syncronize playback by waiting for the trajectories to start
        while not rospy.is_shutdown() and not self._get_trajectory_flag():
            rospy.sleep(0.05)


    def stop(self):
        """
        Preempts trajectory execution by sending cancel goals
        """

        if (self._right_client.gh is not None and
            self._right_client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._right_client.cancel_goal()

        # delay to allow for terminating handshake
        rospy.sleep(0.1)

    def wait(self):
        """
        Waits for and verifies trajectory execution result
        """
        # create a timeout for our trajectory execution
        # total time trajectory expected for trajectory execution plus a buffer
        last_time = self._r_goal.trajectory.points[-1].time_from_start.to_sec()
        time_buffer = rospy.get_param(self._param_ns + 'goal_time', 0.0) + 1.5
        timeout = rospy.Duration(self._slow_move_offset +
                                 last_time +
                                 time_buffer + 10 )
        r_finish = self._right_client.wait_for_result(timeout)
        # error_code
        r_result = (self._right_client.get_result().error_code == 0)

        # verify result
        if all([r_finish,r_result]):
            return True
        else:
            msg = ("Trajectory action failed or did not finish before "
                   "timeout/interrupt.")
            rospy.logwarn(msg)
            return False

# global value & function
global_wrench_stamped = None
def callback_getWrench(wrench_stamped):
     #print "The  callback function data is: {}".format(data.wrench.force.z)
     global global_wrench_stamped
     global_wrench_stamped = wrench_stamped

def main():
    ipdb.set_trace()  # if you want to debug uncomment this line
    # initializing node and enable baxter
    rospy.init_node("Baxter_DMP_RL_joint_trajectory_learn")
    #print("Getting robot state, and initializing robot")
    right_limb = baxter_interface.Limb('right')
    robot_state = baxter_interface.RobotEnable(CHECK_VERSION)
    right_gripper_action = baxter_interface.Gripper('right',CHECK_VERSION)
    robot_state.enable()
    # initializing publisher
    pub_action1_reward = rospy.Publisher('action1_reward',Float32, queue_size = 10)
    pub_action2_reward = rospy.Publisher('action2_reward',Float32, queue_size = 10)
    pub_action3_reward = rospy.Publisher('action3_reward',Float32, queue_size = 10)
    pub_action4_reward = rospy.Publisher('action4_reward',Float32, queue_size = 10)
    pub_action5_reward = rospy.Publisher('action5_reward',Float32, queue_size = 10)
    # force torque sensor subscriber
    rospy.Subscriber("/robotiq_force_torque_wrench", geometry_msgs.msg.WrenchStamped, callback_getWrench)
    # RL agent
    current_reward = 0.0
    agent = LearningAgent( learning = True, epsilon = 1.0, alpha = 0.90, tolerance = 0.1 )
    agent_state = agent.build_state()
    agent.createQ(agent_state)
    trial_count = 1
    # MDP trajetory return to start
    return_to_start = "/home/karen/ros/indigo/baxter_ws/src/baxter_dmp/birl_baxter_dmp/dmp/datasets/right_limb_return_to_start0.csv"
    traj = Trajectory()
    # RL agent learn begin
    result = True
    while ( agent.epsilon > agent.tolerance and result == True and not rospy.is_shutdown()):
	result = False
	# agent state input, return an action
	limb_action = agent.choose_action(agent_state)
        traj.parse_file(limb_action)
        # for safe interrupt handling
        rospy.on_shutdown(traj.stop)
        traj.start()
        traj.wait()
	right_gripper_action.close()
	rospy.sleep(0.5)
        traj._r_goal.trajectory.points = list() # <- DMP class BUG
	# return to start angles
        traj = Trajectory()
	traj.parse_file(return_to_start)
	rospy.on_shutdown(traj.stop)
	traj.start()
	result = traj.wait()
	rospy.sleep(0.5)
	traj._r_goal.trajectory.points = list()
        # give a reward
        print "force data z derction is {}".format(global_wrench_stamped.wrench.force.z)
        if global_wrench_stamped.wrench.force.z > 21:
		current_reward = 1.0
	else:
		current_reward = -1.0
        #agent.get_maxQ(agent_state)
        agent.learn(agent_state, limb_action, current_reward)
        agent.reset()
        #agent.update(reward) s
        print "The current reward is: {}".format(current_reward)
        print "The trial count is: {}".format(trial_count)
	# publish the action reward
	pub_action1_reward.publish(agent.Q[agent.state][agent.valid_actions[0]])
	pub_action2_reward.publish(agent.Q[agent.state][agent.valid_actions[1]])
	pub_action3_reward.publish(agent.Q[agent.state][agent.valid_actions[2]])
	pub_action4_reward.publish(agent.Q[agent.state][agent.valid_actions[3]])
	pub_action5_reward.publish(agent.Q[agent.state][agent.valid_actions[4]])
	right_gripper_action.open()
	rospy.sleep(0.5)
	# reset and update value
	current_reward = 0.0
	trial_count += 1
	traj = Trajectory()

if __name__ == "__main__":
    try:
	main()
    except rospy.ROSInterruptException:
	print "error!!! pls cheak the code."

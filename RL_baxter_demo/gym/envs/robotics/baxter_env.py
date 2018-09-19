from __future__ import print_function

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Empty
import baxter_interface
from baxter_examples.cfg import JointSpringsExampleConfig
from baxter_interface import CHECK_VERSION

from gym.envs.robotics import baxter_utils as bu

##baxter的关节:'right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2'
##提供baxter机器人环境

class BaxterEnv(gym.Env):
    """
    The connection with the Baxter or Gazebo must be initialized before using
    this environment

    These environments require the usage of a modified Limb class to get
    the states of each joint.
    """

    # RIGHT_LIMB = "right"
    # LEFT_LIMB = "left"
    # BOTH_LIMBS = "both"
    #
    # TORQUE = "torque"
    # VELOCITY = "velocity"
    # POSITION = "position"

    def __init__(self, timesteps, control=None,limbs=None):
        print("Initializing node... ")
        rospy.init_node("rlcore_baxter_env")

        self.timesteps = timesteps

        if control == None:
            control = bu.POSITION
        self.control = control

        if limbs == None:
            limbs = bu.BOTH_LIMBS
        self.limbs = limbs

        # set control rate
        self.rate = 5.0
        self.missed_cmds = 20000.0  # Missed cycles before triggering timeout
        self.control_rate = rospy.Rate(self.rate)

        # create our limb instance
        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        if (self.limbs == bu.BOTH_LIMBS):
            self.llimb = baxter_interface.Limb(bu.LEFT_LIMB_NAME)
            self.rlimb = baxter_interface.Limb(bu.RIGHT_LIMB_NAME)
            self.llimb.set_command_timeout((1.0 / self.rate) * self.missed_cmds)
            self.rlimb.set_command_timeout((1.0 / self.rate) * self.missed_cmds)
            # robot state
            self.joint_space = 2*len(self.llimb.joint_angles())
            self.state = np.zeros(self.joint_space)
        elif (self.limbs == bu.LEFT_LIMB):
            self.llimb = baxter_interface.Limb(bu.LEFT_LIMB_NAME)
            self.llimb.set_command_timeout((1.0 / self.rate) * self.missed_cmds)
            # robot state
            self.joint_space = len(self.llimb.joint_angles())
            self.state = np.zeros(self.joint_space)
        elif (self.limbs == bu.RIGHT_LIMB):
            self.rlimb = baxter_interface.Limb(bu.RIGHT_LIMB_NAME)
            self.rlimb.set_command_timeout((1.0 / self.rate) * self.missed_cmds)
            # robot state
            self.joint_space = len(self.rlimb.joint_angles())
            self.state = np.zeros(self.joint_space)

        # verify robot is enabled
        print("Getting Baxter state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling Baxter... ")
        self._rs.enable()

        # prepare shutdown
        rospy.on_shutdown(self.clean_shutdown)

        #self.reset()
        # laction, raction = self.get_joint_action_dict(np.zeros(14))
        # print (laction, raction)
        # self.llimb.move_to_joint_positions(laction)
        # self.rlimb.move_to_joint_positions(raction)

        # print ("Trying velocity")
        # acts = np.empty(14)
        # acts.fill(.25)
        # laction, raction = self.get_joint_action_dict(acts)
        # self.llimb.set_joint_velocities(laction)
        # self.rlimb.set_joint_velocities(raction)
        #
        # print ("Trying torque")
        # import time
        # for i in range(10):
        #     print (i)
        #     acts = np.random.normal(0.0, 0.5, (14,))
        #     laction, raction = self.get_joint_action_dict(acts)
        #     self.llimb.set_joint_torques(laction)
        #     self.rlimb.set_joint_torques(raction)
        #     self.control_rate.sleep()
        #     time.sleep(1)
        #
        # time.sleep(5)
        # import sys
        # sys.exit(1)


    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("Exiting Baxter env...")
        if (self.limbs == bu.BOTH_LIMBS):
            self.llimb.exit_control_mode()
            self.rlimb.exit_control_mode()
        elif (self.limbs == bu.LEFT_LIMB):
            self.llimb.exit_control_mode()
        elif (self.limbs == bu.RIGHT_LIMB):
            self.rlimb.exit_control_mode()

        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()


    def get_joint_angles(self):
        if (self.limbs == bu.BOTH_LIMBS):
            ljointdict = self.llimb.joint_angles()
            rjointdict = self.rlimb.joint_angles()
            ljointangles = [ljointdict[x] for x in self.llimb._joint_names[bu.LEFT_LIMB_NAME]]
            rjointangles = [rjointdict[x] for x in self.rlimb._joint_names[bu.RIGHT_LIMB_NAME]]
            return np.array(ljointangles + rjointangles)
        elif (self.limbs == bu.LEFT_LIMB):
            ljointdict = self.llimb.joint_angles()
            ljointangles = [ljointdict[x] for x in self.llimb._joint_names[bu.LEFT_LIMB_NAME]]
            return np.array(ljointangles)
        elif (self.limbs == bu.RIGHT_LIMB):
            rjointdict = self.rlimb.joint_angles()
            rjointangles = [rjointdict[x] for x in self.rlimb._joint_names[bu.RIGHT_LIMB_NAME]]
            return np.array(rjointangles)


    def get_endeff_position(self):
        if (self.limbs == bu.BOTH_LIMBS):
            return np.array(list(self.llimb.endpoint_pose()['position']) +
                        list(self.rlimb.endpoint_pose()['position']))
        elif (self.limbs == bu.LEFT_LIMB):
            return np.array(list(self.llimb.endpoint_pose()['position']))
        elif (self.limbs == bu.RIGHT_LIMB):
            return np.array(list(self.rlimb.endpoint_pose()['position']))

    def get_joint_action_dict(self, action):
        if (self.limbs == bu.BOTH_LIMBS):
            lkeys = self.llimb._joint_names[bu.LEFT_LIMB_NAME]
            ldict = dict(zip(lkeys, action.tolist()[:self.joint_space/2]))
            rkeys = self.rlimb._joint_names[bu.RIGHT_LIMB_NAME]
            rdict = dict(zip(rkeys, action.tolist()[self.joint_space/2:]))
            return ldict, rdict
        elif (self.limbs == bu.LEFT_LIMB):
            lkeys = self.llimb._joint_names[bu.LEFT_LIMB_NAME]
            ldict = dict(zip(lkeys, action.tolist()[:self.joint_space]))
            return ldict
        elif (self.limbs == bu.RIGHT_LIMB):
            rkeys = self.rlimb._joint_names[bu.RIGHT_LIMB_NAME]
            rdict = dict(zip(rkeys, action.tolist()[:self.joint_space]))
            return rdict

##以下代码应该不会被调用
    def _reset(self):
        if (self.limbs == bu.BOTH_LIMBS):
            self.llimb.move_to_neutral()
            self.rlimb.move_to_neutral()
            self.state = self.get_joint_angles()
            return self.state
        elif (self.limbs == bu.LEFT_LIMB):
            self.llimb.move_to_neutral()
            self.state = self.get_joint_angles()
            return self.state
        if (self.limbs == bu.RIGHT_LIMB):
            self.rlimb.move_to_neutral()
            self.state = self.get_joint_angles()
            return self.state


    def _step(self, action):
        raise NotImplementedError


    def _render(self, mode='human', close=False):
        pass


    @property
    def action_space(self):
        raise NotImplementedError


    @property
    def observation_space(self):
        raise NotImplementedError

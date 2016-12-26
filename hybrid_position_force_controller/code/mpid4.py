#!/usr/bin/python
# -*- coding: utf-8 -*-

# jacobian column s0 s1 e0 e1 w0 w1 w2


# -----------------------------------------
# Imports
# -----------------------------------------
import pdb
import os                                          # used to clear the screen
import math
from numpy import *
from numpy.linalg import *
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from rbx1_nav.transform_utils import quat_to_angle # Convert quaternions to euler
import geometry_msgs
import baxter_core_msgs.msg
import PyKDL
from std_msgs.msg import Float32,ColorRGBA
import dynamic_reconfigure.client
from std_msgs.msg import Empty
import copy


# -----------------------------------------
# Local Methods
# -----------------------------------------
class torque(object):

    def __init__(self):
        print 'initial'
        self.enable_Baxter()
        self.jNamesR = ['right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1']
        self.rLimb = baxter_interface.limb.Limb("right")
        self.rKin = baxter_kinematics('right')
        self.pub2 = rospy.Publisher('/baxter/error', ColorRGBA, queue_size=10)
        # Call routine to enable the robot
        
        # position PID
        self._E_pre_position=matrix([[0],[0],[0]])
        self._E_all_position=matrix([[0],[0],[0]])
        self.refposition=matrix([[0.7],[-0.5],[-0.14]])
        # force PID
        self._E_pre_force=matrix([[0],[0],[0],[0],[0],[0]])
        self._E_all_force=matrix([[0],[0],[0],[0],[0],[0]])
        self.force_torque=matrix([[0],[0],[0],[0],[0],[0],[0]])
        self.refforce=matrix([[-0.03],[0],[0],[0],[0],[0]])
        # keep static PID 0.01 0 0.1
        self._E_pre=matrix([[0],[0],[0],[0],[0],[0],[0]])
        self._E_all=matrix([[0],[0],[0],[0],[0],[0],[0]])
        
        # self.static_torque=matrix([[0],[0],[0],[0],[0],[0],[0]])
        # [-0.5522330830078125, 0.21667478604125978, -0.03413107249145508, 1.4714710690979005, -1.699267215838623, -0.14726215546875002, 1.4450099005371095]

        self.test_msg=ColorRGBA()
        self.count = 0
        
        self.initial_position()        
        
        side = 'right'
        print("Suppressing Gravity compensation for the {} arm...".format(side))
        gravity_topic='/robot/limb/{}/suppress_gravity_compensation'.format(side)
        self.gravity_pub = rospy.Publisher(gravity_topic, Empty, queue_size=10)
        self.gravity_msg = Empty()
        start = rospy.Time.now().to_sec()
        rospy.Timer(rospy.Duration(0.00125), self.suppress_gravity_compensation)

        
        self.refvel = matrix([[0],[0],[0],[0],[0],[0],[0]])
        self.static_torque = matrix([[0.001],[0.001],[0.001],[0.001],[0.001],[0.001],[0.001]])
        
        self.rLimb.set_command_timeout(0.00125)
        self.sub = rospy.Subscriber("/robot/limb/right/gravity_compensation_torques", baxter_core_msgs.msg.SEAJointState, self.get_static_torque)
        
        # note

    def initial_position(self):
        print "initial position"
        Rposition = matrix([[-1],[0.217],[-0.034],[1.471],[-1.699],[-0.147],[1.445]])
        Rposition_dict=dict(zip(self.jNamesR,self.change_order(Rposition.tolist())))
        self.rLimb.move_to_joint_positions(Rposition_dict)
        
        

    def suppress_gravity_compensation(self,event):
        self.gravity_pub.publish(self.gravity_msg)
        """
        print self.count
        static_torque = matrix([[-1],[0],[0],[0],[0],[0],[0]])
        print static_torque
        static_torque_dict=dict(zip(self.jNamesR,self.change_order(static_torque.tolist())))
        print static_torque_dict
        self.rLimb.set_joint_torques(static_torque_dict)
        self.count = self.count + 1
        """
    def enable_Baxter(self):
        # Enable the robot's arms
        print("Getting robot state...")
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self.init_state=self.rs.state().enabled
        print("Enabling robot...")
        self.rs.enable()

    def change_order(self, s0s1e0e1w0w1w2):
        return [s0s1e0e1w0w1w2[0][0],s0s1e0e1w0w1w2[1][0],s0s1e0e1w0w1w2[4][0],s0s1e0e1w0w1w2[5][0],s0s1e0e1w0w1w2[6][0],s0s1e0e1w0w1w2[2][0],s0s1e0e1w0w1w2[3][0]]

    def get_position_vel(self):
        self._postion_vel_kp=rospy.get_param('/dynamic_pid_tutorials/p_param_position')
        self._postion_vel_ki=rospy.get_param('/dynamic_pid_tutorials/i_param_position')
        self._postion_vel_kd=rospy.get_param('/dynamic_pid_tutorials/d_param_position')
        print 'position_kp', self._postion_vel_kp, 'position_ki', self._postion_vel_ki, 'position_kd', self._postion_vel_kd
        actual_position = matrix([\
        self.rLimb.endpoint_pose()['position'].x, self.rLimb.endpoint_pose()['position'].y, self.rLimb.endpoint_pose()['position'].z\
        ]).T
        E_position = self.refposition - actual_position
        print 'Error_position is' ,E_position
        self._E_all_position = self._E_all_position + E_position
        position_vel = pinv(self.rKin.jacobian()[0:3,0:7]) * (self._postion_vel_kp * E_position + self._postion_vel_ki * self._E_all_position\
                                             + self._postion_vel_kd * ( E_position - self._E_pre_position))
        self._E_pre_position = E_position

        return position_vel

    def get_force_torque(self):
        self._force_kp=rospy.get_param('/dynamic_pid_tutorials/p_param_force')
        self._force_ki=rospy.get_param('/dynamic_pid_tutorials/i_param_force')
        self._force_kd=rospy.get_param('/dynamic_pid_tutorials/d_param_force')
        print 'force_kp', self._force_kp, 'force_ki', self._force_ki, 'force_kd', self._force_kd
        actual_force = matrix([\
            self.rLimb.endpoint_effort()['force'].x, self.rLimb.endpoint_effort()['force'].y, self.rLimb.endpoint_effort()['force'].z,\
            self.rLimb.endpoint_effort()['torque'].x, self.rLimb.endpoint_effort()['torque'].y, self.rLimb.endpoint_effort()['torque'].z\
            ]).T
        ddotE = self.refforce - actual_force
        print 'Error_effort is' , ddotE
        cartesian_inertia = self.rKin.cart_inertia()
        self._E_all_force = self._E_all_force + ddotE
        self.force_torque =self.force_torque + self.rKin.jacobian_transpose() * cartesian_inertia * \
                                                        (self._force_kp * ddotE + self._force_ki * self._E_all_force\
                                                         + self._force_kd *(ddotE - self._E_pre_force))
        self._E_pre_force = ddotE
        return self.force_torque

    def get_static_torque(self , SEAJointState):
        # os.system('clear')              
        actual_effort = matrix(SEAJointState.actual_effort).T
        actual_position = matrix(SEAJointState.actual_position).T 
        actual_velocity = matrix(SEAJointState.actual_velocity).T
            
        ref_effort = matrix([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
        ref_vel = matrix([[0.1],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]) 
        
        _E_now = ref_vel - actual_velocity
        self._E_all = self._E_all + _E_now
        self._kp=[60,1,0.1,0.1,0.2,1.5,0.1]
        self._ki=[0,0.0,0.0,0.0,0.0,0.0,0.0]
        self._kd=[0.2,0.0,0.0,0.0,0.0,0.0,0.0]
        for i in range (0,7):
            self.static_torque[i] = 0.0    
        for i in range (0,1):
            self.static_torque[i] = self.static_torque[i] + self._kp[i] * _E_now[i] + self._kd[i] * (_E_now[i] -self._E_pre[i]) + self._ki[i] * self._E_all[i]
        self._E_pre = _E_now
        # output
        static_torque_dict=dict(zip(self.jNamesR,self.change_order(self.static_torque.tolist())))
        print static_torque_dict 
        self.rLimb.set_joint_torques(static_torque_dict)
        self.count = self.count + 1
        print self.count

    def static_position_control(self, SEAJointState):
        os.system('clear')
        self._kp=rospy.get_param('/dynamic_pid_tutorials/p_param_static')
        self._ki=rospy.get_param('/dynamic_pid_tutorials/i_param_static')
        self._kd=rospy.get_param('/dynamic_pid_tutorials/d_param_static')
        print 'static kp', self._kp, 'static ki', self._ki, 'static kd', self._kd
        actual_velocity = matrix(SEAJointState.actual_velocity).T
        # get the design velocity
        position_vel = self.get_position_vel()
        _E_now = position_vel - actual_velocity
        self._E_all = self._E_all + _E_now
        self.static_torque = self.static_torque + self._kp * _E_now + self._kd * (_E_now -self._E_pre) + self._ki * self._E_all
        self._E_pre = _E_now
        static_torque_dict=dict(zip(self.jNamesR,self.change_order(self.static_torque.tolist())))
        self.rLimb.set_joint_torques(static_torque_dict)
        self.count = self.count + 1
        print self.count

    def static_position_force_control(self, SEAJointState):
        self._kp=rospy.get_param('/dynamic_pid_tutorials/p_param_static')
        self._ki=rospy.get_param('/dynamic_pid_tutorials/i_param_static')
        self._kd=rospy.get_param('/dynamic_pid_tutorials/d_param_static')
        print 'static kp', self._kp, 'static ki', self._ki, 'static kd', self._kd
        self.actual_velocity = matrix(SEAJointState.actual_velocity).T
        #增量PID
        position_vel = self.get_position_vel()
        _E_now = position_vel - self.actual_velocity
        self._E_all = self._E_all + _E_now
        self.static_torque = self.static_torque + self._kp * _E_now + self._kd * (_E_now -self._E_pre) + self._ki * self._E_all
        self._E_pre = _E_now
        force_torque = self.get_force_torque()
        s_p_f_torque = self.static_torque + force_torque
        static_torque_dict=dict(zip(self.jNamesR,self.change_order(s_p_f_torque.tolist())))
        self.rLimb.set_joint_torques(static_torque_dict)
        self.count = self.count + 1
        print self.count

    def static_force_control(self, SEAJointState):
        self._kp=rospy.get_param('/dynamic_pid_tutorials/p_param_static')
        self._ki=rospy.get_param('/dynamic_pid_tutorials/i_param_static')
        self._kd=rospy.get_param('/dynamic_pid_tutorials/d_param_static')
        print 'static kp', self._kp, 'static ki', self._ki, 'static kd', self._kd
        self.actual_velocity = matrix(SEAJointState.actual_velocity).T
        # 增量PID
        _E_now = - self.actual_velocity
        self._E_all = self._E_all + _E_now
        self.static_torque = self.static_torque + self._kp * _E_now + self._kd * (_E_now -self._E_pre) + self._ki * self._E_all
        self._E_pre = _E_now

        force_torque = self.get_force_torque()
        s_f_torque = self.static_torque + force_torque

        static_torque_dict=dict(zip(self.jNamesR,self.change_order(s_f_torque.tolist())))
        self.rLimb.set_joint_torques(static_torque_dict)

        self.count = self.count + 1
        print self.count

def main():
    # Initialize node
    rospy.init_node('torque_control_static_')
    print 'start'
    settorque=torque()
    rospy.spin()



if __name__ == "__main__":
    try:
        # pdb.set_trace()
        main()
    except:
        rospy.loginfo("example_baxter_kins_right node terminated.")

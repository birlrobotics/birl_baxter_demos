#!/usr/bin/env python
import roslib
import sys
import random
import struct
sys.path[0]="/home/karen/ros/indigo/baxter_ws/src/baxter_dmp/birl_baxter_dmp/dmp"
roslib.load_manifest('dmp')
import rospy
import numpy as np
import pandas as pd
import ipdb
import matplotlib.pyplot as plt
# import dmp srv
from dmp.srv import *
from dmp.msg import *
# imort baxter KDL
import baxter_interface
from baxter_pykdl import baxter_kinematics
from baxter_interface import CHECK_VERSION
# import msg
# from std_msgs.msg import (
#     Header,
#     Empty,
# )
# from geometry_msgs.msg import (
#     PoseStamped,
#     Pose,
#     Point,
#     Quaternion,
# )
from baxter_core_msgs.msg import JointCommand
# import baxter srv
# from baxter_core_msgs.srv import (
#     SolvePositionIK,
#     SolvePositionIKRequest,
# )




class Dmp(object):
    def __init__(self):
         self.q_s0 = None
         self.q_s1 = None
         self.q_e0 = None
         self.q_e1 = None
         self.q_w0 = None
         self.q_w1 = None
         self.q_w2 = None
  
    def callback(self,data):    
        self.q_s0 = data.command[0]
        self.q_s1 = data.command[1]
        self.q_e0 = data.command[2]
        self.q_e1 = data.command[3]
        self.q_w0 = data.command[4]
        self.q_w1 = data.command[5]
        self.q_w2 = data.command[6]
        rospy.loginfo("q_s0 %s", data.command[0])
        rospy.loginfo("q_s1 %s", data.command[1])
        rospy.loginfo("q_e0 %s", data.command[2])
        rospy.loginfo("q_e1 %s", data.command[3])
        rospy.loginfo("q_w0 %s", data.command[4])
        rospy.loginfo("q_w1 %s", data.command[5])
        rospy.loginfo("q_w2 %s", data.command[6])

#    
#    def ik_solve(self,w_x,w_y,w_z,w_qx,w_qy,w_qz,w_qw):
#        rospy.wait_for_service('baxter_tracik/ik_solver')
#        solve = rospy.ServiceProxy('baxter_tracik/ik_solver', ik_solver)
#        resp2 = solve(w_x,w_y,w_z,w_qx,w_qy,w_qz,w_qw)
#        return resp2
#          
#Learn a DMP from demonstration data
    def makeLFDRequest(self,dims, traj, dt, K_gain, D_gain, num_bases):
        demotraj = DMPTraj()
    
        for i in range(len(traj)):
            pt = DMPPoint()
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
    
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
    
        print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD done"
        return resp


#Set a DMP as active for planning
    def makeSetActiveRequest(self, dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


#Generate a plan from a DMP
    def makePlanRequest(self,x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter):
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                       seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"
        return resp

# ======================== #
def enable_Baxter():
    # Enable the robot's arms
    print("Getting robot state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state=rs.state().enabled
    print("Enabling robot...")
    rs.enable() 
    return rs


def baxterLimb_kin(baxter_limb = "right"):
    """
    This is baxter limb object
    @baxter_limb = "right" or "left"
    """
    try:
        #rLimb=baxter_interface.Limb(baxter_limb)
        rKin = baxter_kinematics(baxter_limb)
        return rKin
    except:
        print "baxterLimb_kin error!"
        return None

# def ik_request( pose, _verbose = True):
#         hdr = Header(stamp=rospy.Time.now(), frame_id='base')
#         ikreq = SolvePositionIKRequest()
#         ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
#         try:
#             resp = _iksvc(ikreq)
#         except (rospy.ServiceException, rospy.ROSException), e:
#             rospy.logerr("Service call failed: %s" % (e,))
#             return False
#         # Check if result valid, and type of seed ultimately used to get solution
#         # convert rospy's string representation of uint8[]'s to int's
#         resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
#         limb_joints = {}
#         if (resp_seeds[0] != resp.RESULT_INVALID):
#             seed_str = {
#                         ikreq.SEED_USER: 'User Provided Seed',
#                         ikreq.SEED_CURRENT: 'Current Joint Angles',
#                         ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
#                        }.get(resp_seeds[0], 'None')
#             if _verbose:
#                 print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format((seed_str)))
#             # Format solution into Limb API-compatible dictionary
#             limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#             if _verbose:
#                 print("IK Joint Solution:\n{0}".format(limb_joints))
#                 print("------------------")
#         else:
#             rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
#             return False
#         return limb_joints


def move_to_start(start_angles=None):
    #print("Moving the {0} arm to start pose...".format(_limb_name))
    # if not start_angles:
    #     start_angles = dict(zip(_joint_names, [0]*7))
    _limb.move_to_joint_positions(start_angles)
    #self.gripper_open()
    rospy.sleep(1.0)
    print("Go to start position!!!")

if __name__ == '__main__':

    starting_joint_angles = {'right_w0': -0.6699952259595108,
                        'right_w1': 1.030009435085784,
                        'right_w2': 0.4999997247485215,
                        'right_e0': -0.189968899785275,
                        'right_e1': 1.9400238130755056,
                        'right_s0': 0.08000397926829805,
                        'right_s1': -0.9999781166910306}

    ipdb.set_trace()
    rospy.init_node('dmp_baxter_r_arm_node')
    rs = enable_Baxter()
    dmp = Dmp()

    #rospy.Subscriber("end_effector_command_solution",JointCommand, dmp.callback)
    #rospy.wait_for_message("end_effector_command_solution",JointCommand)  
    #rospy.loginfo("q_s0 %s", dmp.q_s0) 
#    rospy.Subscriber("aruco_tracker/pose",PoseStamped, dmp.callback)
#    rospy.loginfo("I heard %s", dmp.w_qx)
# 
#    print('successfully get rot matrix')

    plt.close('all')
    # read the origin trajectory csv file
    train_set = pd.read_csv('/home/karen/ros/indigo/baxter_ws/src/baxter_dmp/birl_baxter_dmp/dmp/datasets/baxter_joint_input_data.csv')
    
    train_len = len(train_set)
    resample_t = np.linspace(train_set.values[0,0], train_set.values[-1,0], train_len)
    joint0_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:, 9])
    joint1_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:, 10])
    joint2_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:, 11])
    joint3_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:, 12])
    joint4_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:, 13])
    joint5_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:, 14])
    joint6_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:, 15])
    
    traj = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]* train_len
    for i in range(train_len):
        traj[i] = [joint0_data[i], joint1_data[i], joint2_data[i], joint3_data[i], joint4_data[i], joint5_data[i], joint6_data[i]]

    ## plot the trining trajectory of each joint angle
    #f1, axarr1 = plt.subplots(7, sharex=True)
    #axarr1[0].plot(resample_t, joint0_data)
    #axarr1[0].set_title('right_arm_joint_space0')
    #axarr1[1].plot(resample_t, joint1_data)
    #axarr1[2].plot(resample_t, joint2_data)
    #axarr1[3].plot(resample_t, joint3_data)
    #axarr1[4].plot(resample_t, joint4_data)
    #axarr1[5].plot(resample_t, joint5_data)
    #axarr1[6].plot(resample_t, joint6_data)
    #plt.show()


    #Create a DMP from a 7-D trajectory
    dims = 7
    dt = 0.01
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 200
    resp = dmp.makeLFDRequest(dims, traj, dt, K, D, num_bases)
    #Set it as the active DMP
    dmp.makeSetActiveRequest(resp.dmp_list)

    ###### ????
    limb = "right"
    #goal_position = list()
    _limb = baxter_interface.Limb(limb)

   # move to starting agles
    #move_to_start(starting_joint_angles)

    # goal_pose = Pose()
    # goal_pose.position.x = 0.6 
    # goal_pose.position.y = -0.2
    # goal_pose.position.z = -0.115 - 0.005

    # goal_orientation = Quaternion(
    #         x = 0.0,
    #         y = 1.0,
    #         z = 0.0,
    #         w = 6.123233995736766e-17)
    # goal_position.append(Pose(
    #     position=Point(x=goal_pose.position.x, y=goal_pose.position.y, z=goal_pose.position.z),
    #     orientation=goal_orientation))


    # ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    # _iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    # joint_angle = ik_request(goal_position[0], _verbose = True)




    goal_angle = list()
    #GoalAngle.append( rKin.inverse_kinematics([random.uniform(0.60, 0.75), random.uniform(-0.32, -0.04), -0.02], [-0.06, 1.0, -0.03,  -0.01]))

    # this is file count , which is to count the file name
    fileCount = 0
    goal_angle = [
	[ 0.91693702, -0.60515542, -0.13345633, 0.99900499, 0.05867477, 1.20033997, -0.05752428],
	[ 0.93534479, -0.69757776, -0.319068, 1.24904386, 0.19404857, 1.08068947, -0.32136898],
	[ 1.06766968, -0.57392224, -0.38857265, 0.97902982, 0.28840469, 1.21884274, -0.23655989],
	[ 0.77657777, -0.32520393, -0.05138836, 0.40803889, 0.02684466, 1.46955359, -0.1119806 ],
	[ 0.48282045, -0.73937874,  0.26537868,  1.28969435, -0.29682528, 1.05767975, -0.13192235]]
    ###
    
    # Now, generate a plan, at start joint angle.
    x_0 = [joint0_data[0], joint1_data[0], joint2_data[0], joint3_data[0], joint4_data[0], joint5_data[0], joint6_data[0]]          
    x_dot_0 = [0.4, 0.4, 0.4, 0.4, 0.4, 0.0, 0.4]
    t_0 = 1.3

    while ( fileCount < 5 and not rospy.is_shutdown() ) :
        # this is DMP goal joint angle
        #goal =[ joint0_data[-1], joint1_data[-1], joint2_data[-1], joint3_data[-1], joint4_data[-1], joint5_data[-1], joint6_data[-1]]
        
        #goal_angle.append(  ik_request(Pose(position = Point(x=random.uniform(0.60, 0.60), y=random.uniform(-0.2, -0.2), z= -0.115 - 0.005), orientation=goal_orientation),  _verbose = True) )

        goal = goal_angle[fileCount]
        goal_thresh = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
        seg_length = -1          #Plan until convergence to goal
        tau = 2 * resp.tau       #Desired plan should take twice as long as demo
        # dt = 1.0
        integrate_iter = 5       #dt is rather large, so this is > 1
        plan = dmp.makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter)

        Column0_plan = [0.0]*len(plan.plan.times)
        Column1_plan = [0.0]*len(plan.plan.times)
        Column2_plan = [0.0]*len(plan.plan.times)
        Column3_plan = [0.0]*len(plan.plan.times)
        Column4_plan = [0.0]*len(plan.plan.times)
        Column5_plan = [0.0]*len(plan.plan.times)
        Column6_plan = [0.0]*len(plan.plan.times)
        for i in range(len(plan.plan.times)):    
            Column0_plan[i] = plan.plan.points[i].positions[0]
            Column1_plan[i] = plan.plan.points[i].positions[1]
            Column2_plan[i] = plan.plan.points[i].positions[2]
            Column3_plan[i] = plan.plan.points[i].positions[3]
            Column4_plan[i] = plan.plan.points[i].positions[4]
            Column5_plan[i] = plan.plan.points[i].positions[5]
            Column6_plan[i] = plan.plan.points[i].positions[6]

        resample_t0 = np.linspace(0.01,plan.plan.times[-1], train_len)
        joint0_data_plan = np.interp(resample_t0, plan.plan.times, Column0_plan)
        joint1_data_plan = np.interp(resample_t0, plan.plan.times, Column1_plan)
        joint2_data_plan = np.interp(resample_t0, plan.plan.times, Column2_plan)
        joint3_data_plan = np.interp(resample_t0, plan.plan.times, Column3_plan)
        joint4_data_plan = np.interp(resample_t0, plan.plan.times, Column4_plan)
        joint5_data_plan = np.interp(resample_t0, plan.plan.times, Column5_plan)
        joint6_data_plan = np.interp(resample_t0, plan.plan.times, Column6_plan)

        #record the plan trajectory, and write in the output csv file
        #WriteFileDir ="/home/karen/ros/indigo/baxter_ws/src/baxter_dmp/birl_baxter_dmp/dmp/datasets/baxter_joint_output_data.csv"    
        WriteFileDir = "/home/karen/ros/indigo/baxter_ws/src/baxter_dmp/birl_baxter_dmp/dmp/datasets/baxter_joint_output_data%d.csv" %fileCount

        plan_len = len(plan.plan.times)
        f = open(WriteFileDir,'w')
        f.write('time,')
        f.write('right_s0,')
        f.write('right_s1,')
        f.write('right_e0,')
        f.write('right_e1,')
        f.write('right_w0,')
        f.write('right_w1,')
        f.write('right_w2\n')
        
        for i in range(train_len):
            f.write("%f," % (resample_t[i],))
            f.write(str(joint0_data_plan[i])+','+str(joint1_data_plan[i])+','+str(joint2_data_plan[i])+','+str(joint3_data_plan[i])+','+str(joint4_data_plan[i])+','+str(joint5_data_plan[i])+','+str(joint6_data_plan[i]) +'\n')        
        f.close()

        print "DMP planning finished!"
        fileCount += 1
        #goal_angle = list()
        print "The file count is: {}".format(fileCount - 1)
        ###########    
        # plot the DMP planning trajectory of each joint angle
        #f2, axarr2 = plt.subplots(7, sharex=True)
        #axarr2[0].plot(resample_t, joint0_data_plan)
        #axarr2[0].set_title('right_arm_joint_space1')
        #axarr2[1].plot(resample_t, joint1_data_plan)
        #axarr2[2].plot(resample_t, joint2_data_plan)
        #axarr2[3].plot(resample_t, joint3_data_plan)
        #axarr2[4].plot(resample_t, joint4_data_plan)
        #axarr2[5].plot(resample_t, joint5_data_plan)
        #axarr2[6].plot(resample_t, joint6_data_plan)
        #plt.show()

#    rospy.spin()


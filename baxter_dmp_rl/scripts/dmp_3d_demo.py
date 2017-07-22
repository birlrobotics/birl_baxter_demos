#!/usr/bin/env python
import roslib
roslib.load_manifest('dmp')
import sys
sys.path[0]="/home/karen/ros/indigo/baxter_ws/src/baxter_dmp/birl_baxter_dmp/dmp"
import rospy
import numpy as np
import ipdb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dmp.srv import *
from dmp.msg import *

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
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
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;


if __name__ == '__main__':
    # debug begin
    ipdb.set_trace()
    rospy.init_node('dmp_tutorial_node')
    #Create a DMP from a 3-D trajectory
    # set DMP parameters
    dims = 3
    dt = 1.0
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 100
    # read origin trajectory csv file
    data = "/home/karen/ros/indigo/baxter_ws/src/baxter_dmp/birl_baxter_dmp/dmp/datasets/3D_demo_data.csv"
    PointCount = len(open(data,'rU').readlines()) # count the trajectory csv file line by line
    Colum0_Traj=[0.0]*PointCount # [0.0 .....  0.0] ?
    Colum1_Traj=[0.0]*PointCount # [0.0 .....  0.0]
    Colum2_Traj=[0.0]*PointCount # [0.0 .....  0.0]
    Colum0_Traj=[float(l.split()[0]) for l in open(data)]
    Colum1_Traj=[float(l.split()[1]) for l in open(data)]
    Colum2_Traj=[float(l.split()[2]) for l in open(data)]    
    traj=[[0.0, 0.0, 0.0]]*PointCount
    for i in range(PointCount):
        traj[i]=[Colum0_Traj[i], Colum1_Traj[i], Colum2_Traj[i]]
    #DMP creat and return a dict
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)
    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)
##################################
# set DMP paramiters
    x_dot_0 = [0.0,0.0,0.0]
    t_0 = 0
    goal_thresh = [0.2,0.2,0.2]
    seg_length = -1        #Plan until convergence to goal
    tau = 2 * resp.tau    #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5    #dt is rather large, so this is > 1
##################################
    # generate a plan, goal position 0
    x_0 = [1.0000000e-02, -8.0985915e-01, -7.9287305e-01]   #start point 
    goal = [2.0000000e+00, 5.0469484e-01,-8.0178174e-02]    #different goal position
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter)
                           
    #plan data for plot
    Colum0_plan=[0.0]*len(plan.plan.times)
    Colum1_plan=[0.0]*len(plan.plan.times)
    Colum2_plan=[0.0]*len(plan.plan.times)
    for i in range(len(plan.plan.times)):    
        Colum0_plan[i]=plan.plan.points[i].positions[0]
        Colum1_plan[i]=plan.plan.points[i].positions[1]
        Colum2_plan[i]=plan.plan.points[i].positions[2]   
    
################################
    #generate a plan, goal position 1
    x1_0 = [1.0000000e-02, -8.0985915e-01, 0-7.9287305e-01]   #start point 
    goal1 = [2.0000000e+00, 5.0469484e-01,0.5+-8.0178174e-02]    #different goal position
    plan = makePlanRequest(x1_0, x_dot_0, t_0, goal1, goal_thresh, seg_length, tau, dt, integrate_iter)
                           
    #plan data for plot
    Colum0_plan1=[0.0]*len(plan.plan.times)
    Colum1_plan1=[0.0]*len(plan.plan.times)
    Colum2_plan1=[0.0]*len(plan.plan.times)
    for i in range(len(plan.plan.times)):    
        Colum0_plan1[i]=plan.plan.points[i].positions[0]
        Colum1_plan1[i]=plan.plan.points[i].positions[1]
        Colum2_plan1[i]=plan.plan.points[i].positions[2]   
################################ 
    #generate a plan, goal position 2
    x2_0 = [1.0000000e-02, -8.0985915e-01, -7.9287305e-01]   #start point  
    goal2 = [2.0000000e+00, 5.0469484e-01,-0.5+-8.0178174e-02]    #different goal position
    plan = makePlanRequest(x2_0, x_dot_0, t_0, goal2, goal_thresh, seg_length, tau, dt, integrate_iter)
                           
    #plan data for plot
    Colum0_plan2=[0.0]*len(plan.plan.times)
    Colum1_plan2=[0.0]*len(plan.plan.times)
    Colum2_plan2=[0.0]*len(plan.plan.times)
    for i in range(len(plan.plan.times)):    
        Colum0_plan2[i]=plan.plan.points[i].positions[0]
        Colum1_plan2[i]=plan.plan.points[i].positions[1]
        Colum2_plan2[i]=plan.plan.points[i].positions[2]   
################################                 
    #generate a plan, goal position 3
    x3_0 = [1.0000000e-02, -8.0985915e-01, -7.9287305e-01]   #start point 
    goal3 = [1.0+2.0000000e+00, 5.0469484e-01,-8.0178174e-02]    #different goal position
    plan = makePlanRequest(x3_0, x_dot_0, t_0, goal3, goal_thresh, seg_length, tau, dt, integrate_iter)
                           
    #plan data for plot
    Colum0_plan3=[0.0]*len(plan.plan.times)
    Colum1_plan3=[0.0]*len(plan.plan.times)
    Colum2_plan3=[0.0]*len(plan.plan.times)
    for i in range(len(plan.plan.times)):    
        Colum0_plan3[i]=plan.plan.points[i].positions[0]
        Colum1_plan3[i]=plan.plan.points[i].positions[1]
        Colum2_plan3[i]=plan.plan.points[i].positions[2]   
#################################   
    #generate a plan, goal position 4
    x4_0 = [1.0000000e-02, -8.0985915e-01, -7.9287305e-01]   #start point  
    goal4 = [-1.0+2.0000000e+00, 5.0469484e-01,-8.0178174e-02]    #different goal position
    plan = makePlanRequest(x4_0, x_dot_0, t_0, goal4, goal_thresh, seg_length, tau, dt, integrate_iter)
                           
    #plan data for plot
    Colum0_plan4=[0.0]*len(plan.plan.times)
    Colum1_plan4=[0.0]*len(plan.plan.times)
    Colum2_plan4=[0.0]*len(plan.plan.times)
    for i in range(len(plan.plan.times)):    
        Colum0_plan4[i]=plan.plan.points[i].positions[0]
        Colum1_plan4[i]=plan.plan.points[i].positions[1]
        Colum2_plan4[i]=plan.plan.points[i].positions[2]   
##################################  
       

    #creat fig
    fig=plt.figure()
    ax = Axes3D(fig)    
    plt.xlabel('X')
    plt.ylabel('Y')
    
    #plot traj fig 
    ax.plot(Colum0_Traj,Colum1_Traj,Colum2_Traj,linewidth=4,alpha=0.3)       
    #Plot plan fig    
    ax.plot(Colum0_plan,Colum1_plan,Colum2_plan,"--")
    #Plot plan fig    
    ax.plot(Colum0_plan1,Colum1_plan1,Colum2_plan1,"--")
    #Plot plan fig    
    ax.plot(Colum0_plan2,Colum1_plan2,Colum2_plan2,"--")
    #Plot plan fig    
    ax.plot(Colum0_plan3,Colum1_plan3,Colum2_plan3,"--")
    #Plot plan fig    
    ax.plot(Colum0_plan4,Colum1_plan4,Colum2_plan4,"--")
    
    #show the plot
    plt.draw()
    plt.show()

    

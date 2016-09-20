#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import Float64
import turtlesim.srv
import math
import baxter_interface
from baxter_interface import CHECK_VERSION

import operator


mirror = True
times = 20
data_set = {'left':[[0]*times for i in range(4)], 'right':[[0]*times for i in range(4)]}



#new for v8
pub_list = {}
pub_list_raw = {}

def vector_length(v):
    return math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])

def get_unit_vector(v):
    length = vector_length(v)
    return (v[0]/length, v[1]/length, v[2]/length)

def angle_query(v1, v2):
    p = v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]
    m1 = vector_length(v1)
    m2 = vector_length(v2)
    cos = p/(m1*m2)
    return math.acos(cos)


def vector_cross_product(a, b):
    r1 = a[1]*b[2]-b[1]*a[2]
    r2 = a[2]*b[0]-b[2]*a[0]
    r3 = a[0]*b[1]-b[0]*a[1]
    return (r1, r2, r3)

def tf_2_angles(shoulder, shoulder_x, elbow, hand,  torso, head, base, listener, limb_name):
    try:
        (b_2_s, trash) = listener.lookupTransform(base, shoulder, rospy.Time(0))
        (b_2_s_x, trash) = listener.lookupTransform(base, shoulder_x, rospy.Time(0))
        (b_2_e, trash) = listener.lookupTransform(base, elbow, rospy.Time(0))
        (b_2_hand, trash) = listener.lookupTransform(base, hand, rospy.Time(0))
        (b_2_t, trash) = listener.lookupTransform(base, torso, rospy.Time(0))
        (b_2_head, trash) = listener.lookupTransform(base, head, rospy.Time(0))
        
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "tf exception"
        return {}

    b_2_s = list(b_2_s)
    b_2_s_x = list(b_2_s_x)
    b_2_e = list(b_2_e)
    b_2_hand = list(b_2_hand)
    b_2_t = list(b_2_t)
    b_2_head = list(b_2_head)
    
    
    
    s_2_s_x = map(operator.sub, b_2_s_x, b_2_s)
    s_2_t = map(operator.sub, b_2_t, b_2_s)
    s_2_e = map(operator.sub, b_2_e, b_2_s)
    e_2_hand = map(operator.sub, b_2_hand, b_2_e)
    
    head_2_t = map(operator.sub, b_2_t, b_2_head)
    
    my_v2 = head_2_t
    
    #print 'v2:', my_v2
    
    my_v3 = vector_cross_product(s_2_e, my_v2)
    
    #print 'v3:', my_v3
    
    
    my_theta_1 = angle_query(my_v3, s_2_s_x)
    
    
    
    #print 'my_theta_1:',my_theta_1/math.pi*180
    if limb_name == 'right':
        radius_s0 = math.pi/4-my_theta_1
    else:
        radius_s0 = math.pi*3/4-my_theta_1
        
        
    if mirror:
        if limb_name == 'left': 
            radius_s0 = -radius_s0+math.pi/2
        else:
            radius_s0 = -radius_s0-math.pi/2

    my_theta_2 = angle_query(my_v2, s_2_e)
    radius_s1 = math.pi/2-my_theta_2
    
    my_v4 = vector_cross_product(s_2_e, e_2_hand)
    
    #print "v4:", my_v4
    
    my_theta_3 = angle_query(my_v3,my_v4)
    
    
    radius_e0 = my_theta_3
    if limb_name == 'left':
        radius_e0 = -radius_e0
    
    my_theta_4 = angle_query(s_2_e,e_2_hand)
    radius_e1 = my_theta_4
    
    
    return {"s0":radius_s0, "s1":radius_s1, "e0":radius_e0, "e1":radius_e1}
    
def track_one_arm(shoulder, shoulder_x, elbow, hand, torso, head, base, listener, limb_name):
    global pub_list_raw, pub_list


    d = tf_2_angles(shoulder, shoulder_x, elbow, hand, torso, head, base, listener, limb_name)
        
    if d == {}:
        print "tf listen failed."
        return
        
    radius_s0_raw = d["s0"]
    radius_s1_raw = d["s1"]
    radius_e0_raw = d["e0"]
    radius_e1_raw = d["e1"]

    print limb_name
    

    #pub_list_raw[limb_name+'_s0'].publish(d["s0"])
    #pub_list_raw[limb_name+'_s1'].publish(d["s1"])
    #pub_list_raw[limb_name+'_e0'].publish(d["e0"])
    #pub_list_raw[limb_name+'_e1'].publish(d["e1"])
    
    #print radius_s0_raw,radius_s1_raw,radius_e0_raw,radius_e1_raw
    
    print
    for i in data_set[limb_name]:
        i.pop(0)
     
    #print data_set[limb_name][0], data_set[limb_name][1], data_set[limb_name][2], data_set[limb_name][3] 

    data_set[limb_name][0].append(radius_s0_raw)
    data_set[limb_name][1].append(radius_s1_raw)
    data_set[limb_name][2].append(radius_e0_raw)
    data_set[limb_name][3].append(radius_e1_raw)
    
    #print data_set[limb_name][0], data_set[limb_name][1], data_set[limb_name][2], data_set[limb_name][3]
    


    command = [0.0, 0.0, 0.0, 0.0]
    for i in range(0, 4):
        sum = 0
        for j in range(0, times):
            sum = sum+data_set[limb_name][i][j]
        command[i] = sum/times
    
    radius_s0 = command[0]
    radius_s1 = command[1]
    radius_e0 = command[2]
    radius_e1 = command[3]
        
    print radius_s0, radius_s1, radius_e0, radius_e1

    print "haha"
    pub_list[limb_name+'_s0'].publish(radius_s0)
    print "hehe"

    pub_list[limb_name+'_s1'].publish(radius_s1)
    pub_list[limb_name+'_e0'].publish(radius_e0)
    pub_list[limb_name+'_e1'].publish(radius_e1)


def talker():
    global pub_list_raw, pub_list
    rospy.init_node('v8_xnode', anonymous=True)
    
    #pub part
    pub_left_s0 = rospy.Publisher('left_s0', Float64, queue_size=10)
    pub_left_s1 = rospy.Publisher('left_s1', Float64, queue_size=10)
    pub_left_e0 = rospy.Publisher('left_e0', Float64, queue_size=10)
    pub_left_e1 = rospy.Publisher('left_e1', Float64, queue_size=10)

    pub_right_s0 = rospy.Publisher('right_s0', Float64, queue_size=10)
    pub_right_s1 = rospy.Publisher('right_s1', Float64, queue_size=10)
    pub_right_e0 = rospy.Publisher('right_e0', Float64, queue_size=10)
    pub_right_e1 = rospy.Publisher('right_e1', Float64, queue_size=10)
    pub_list = {'left_s0':pub_left_s0,
                'left_s1':pub_left_s1,
                'left_e0':pub_left_e0,
                'left_e1':pub_left_e1,
                'right_s0':pub_right_s0,
                'right_s1':pub_right_s1,
                'right_e0':pub_right_e0,
                'right_e1':pub_right_e1}
    

    #pub_left_s0_raw = rospy.Publisher('left_s0_raw', Float64, queue_size=10)
    #pub_left_s1_raw = rospy.Publisher('left_s1_raw', Float64, queue_size=10)
    #pub_left_e0_raw = rospy.Publisher('left_e0_raw', Float64, queue_size=10)
    #pub_left_e1_raw = rospy.Publisher('left_e1_raw', Float64, queue_size=10)
    #pub_right_s0_raw = rospy.Publisher('rihgt_s0_raw', Float64, queue_size=10)
    #pub_right_s1_raw = rospy.Publisher('right_s1_raw', Float64, queue_size=10)
    #pub_right_e0_raw = rospy.Publisher('right_e0_raw', Float64, queue_size=10)
    #pub_right_e1_raw = rospy.Publisher('right_e1_raw', Float64, queue_size=10)
    #pub_list_raw = {'left_s0':pub_left_s0_raw,
    #            'left_s1':pub_left_s1_raw,
    #            'left_e0':pub_left_e0_raw,
    #            'left_e1':pub_left_e1_raw,
    #            'right_s0':pub_right_s0_raw,
    #            'right_s1':pub_right_s1_raw,
    #            'right_e0':pub_right_e0_raw,
    #            'right_e1':pub_right_e1_raw}


    


    #tf part
    listener = tf.TransformListener()
    
    user_index = '3'

    for_my_left_shoulder = 'right_shoulder_'+user_index
    for_my_left_shoulder_x = 'left_shoulder_'+user_index
    for_my_left_elbow = 'right_elbow_'+user_index
    for_my_left_hand = 'right_hand_'+user_index
    
    for_my_right_shoulder = 'left_shoulder_'+user_index
    for_my_right_shoulder_x = 'right_shoulder_'+user_index
    for_my_right_elbow = 'left_elbow_'+user_index
    for_my_right_hand = 'left_hand_'+user_index
    
    head = 'head_'+user_index
    torso = 'torso_'+user_index
    base = 'camera_depth_frame'


    
    rospy.sleep(5)

    rate = rospy.Rate(10) # 10hz

    
    while not rospy.is_shutdown():
        

        limb_1_name = 'left'
        limb_2_name = 'right'        
        if mirror == True:
            limb_1_name = 'right'
            limb_2_name = 'left'

        track_one_arm(for_my_left_shoulder
                        ,for_my_left_shoulder_x
                        ,for_my_left_elbow
                        ,for_my_left_hand
                        ,torso
                        ,head
                        ,base
                        ,listener
                        ,limb_1_name)
        track_one_arm(for_my_right_shoulder
                        ,for_my_right_shoulder_x
                        ,for_my_right_elbow
                        ,for_my_right_hand
                        ,torso
                        ,head
                        ,base
                        ,listener
                        ,limb_2_name)





        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

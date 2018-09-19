from __future__ import print_function
import argparse, csv, os, sys
import numpy as np
import matplotlib.pyplot as plt
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
# from baxter_trajectory import Trajectory

POSITION, VELOCITY, TORQUE = range(3)
LEFT_LIMB, RIGHT_LIMB, BOTH_LIMBS = range(3)
LEFT_LIMB_NAME = 'left'
RIGHT_LIMB_NAME = 'right'

DOF = 17
DOF_NO_TIME = 16
DOF_NO_TIME_NO_GRIPPER = 14
DOF_NO_TIME_LEFT = 8
DOF_NO_TIME_RIGHT = 8
DOF_NO_TIME_NO_GRIPPER_LEFT = 7
DOF_NO_TIME_NO_GRIPPER_RIGHT = 7

def limb_in_sphere(limb, sphere):
    """
    sphere - a sphere defined by (x, y, z, r)
    limb - a baxter limb object
    Returns True if any segment of limb is in space
    """
    states = limb.get_link_states()
    links = limb._link_names[limb.name]
    for i in range(len(links)-1):
        l1 = links[i]
        l2 = links[i+1]
        p1 = states[l1].position
        # print ("p1 = ", p1)
        p2 = states[l2].position
        # print ("p2 = ", p2)
        p1 = (p1.x, p1.y, p1.z)
        p2 = (p2.x, p2.y, p2.z)
        if (check_segment(sphere, (p1, p2))):
            return True
    return False


def check_segment(sphere, segment):
    """
    segment = ((x1,y1,z1), (x2,y2,z2))
    returns True if intersection
    """
    p1, p2 = segment
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x3, y3, z3, r = sphere
    a = (x2-x1)**2.0 + (y2-y1)**2.0 + (z2-z1)**2.0
    b = 2*( (x2-x1)*(x1-x3) + (y2-y1)*(y1-y3) + (z2-z1)*(z1-z3) )
    c = x3**2.0 + y3**2.0 + z3**2.0 + x1**2.0 + y1**2.0 + z1**2.0 - 2*(x3*x1 + y3*y1 + z3*z1) - r**2.0

    u1 = (-b + np.sqrt(b*b-4*a*c)) / 2*a
    u2 = (-b - np.sqrt(b*b-4*a*c)) / 2*a
    if ((u1 < 0 and u2 < 0) or (u1 > 1 and u2 > 1)):
        # line seg outside sphere
        return False
    if ((u1 < 0 and u2 > 1) or (u1 > 1 and u2 < 0)):
        # line segment inside sphere
        return True
    if ((abs(u1) < 1 and (u2 > 1 or u2 < 0)) or (abs(u2) < 1 and (u1 > 1 or u1 < 0))):
        # line segment intersects at one point
        return True
    if (abs(u1) < 1 and abs(u2) < 1):
        # intersects at two points
        return True
    if (abs(u1) < 1 and abs(u2) < 1 and u1 - u2 < .001):
        # tangential
        return True
    return False


def load_trajectories(dir):
    """
    Load all trajectories from dir into a list of numpy arrays
    """
    trajs = []
    keys = None
    files = os.listdir(dir)
    for f in files:
        keys, traj = load_trajectory(os.path.join(dir, f))
        trajs.append(traj)

    return trajs, keys


def load_trajectory(fname):
    """
    Load the trajectory in the file fname to a numpy array
    """
    with open(fname, 'rb') as csvfile:
        reader = csv.reader(csvfile)
        keys = reader.next()
        data = []
        for row in reader:
            row = [float(f) for f in row]
            data.append(row)
        data = np.array(data)
        return keys, data


def get_trajectories_without_time(trajs, limb=BOTH_LIMBS):
    if (limb == BOTH_LIMBS):
        trajs = [t[:,1:] for t in trajs]
        return trajs
    elif (limb == LEFT_LIMB):
        # remove time
        trajs = [t[:,1:] for t in trajs]
        trajs = [t[:,:8] for t in trajs]
        return trajs
    elif (limb == RIGHT_LIMB):
        # remove time
        trajs = [t[:,1:] for t in trajs]
        trajs = [t[:,8:] for t in trajs]
        return trajs


def get_trajectories_without_time_and_gripper(trajs, limb=BOTH_LIMBS):
    if (limb == BOTH_LIMBS):
        trajs = [np.hstack((t[:,1:8],t[:,9:16])) for t in trajs]
        return trajs
    elif (limb == LEFT_LIMB):
        trajs = [t[:,1:8] for t in trajs]
        return trajs
    elif (limb == RIGHT_LIMB):
        trajs = [t[:,9:16] for t in trajs]
        return trajs


def interpolate_time(secs, timesteps):
    time = np.linspace(0, secs, timesteps)
    return time


def run_trajectory(traj, mode=POSITION):
    if (mode == POSITION):
        run_position_trajectory(traj)
    # elif (mode == VELOCITY):
    #     run_velocity_trajectory(traj)


# def run_velocity_trajectory(traj_data):
#     traj = Trajectory()
#     traj.parse_traj(self.keys, traj_data)
#     #for safe interrupt handling
#     rospy.on_shutdown(traj.stop)
#     result = True
#     print ("Starting trajectory")
#     traj.start()
#     result = traj.wait()
#     print("Result: " + str(result) + ", Playback Complete")


def run_position_trajectory(traj, left, right, rate):
    # traj is a numpy array where each column is a DOF
    # and each row is a timestep
    time, lstart, rstart = get_cmds_from_row(traj[0])
    left.move_to_joint_positions(lstart)
    right.move_to_joint_positions(rstart)

    start_time = rospy.get_time()
    for t in range(traj.shape[0]): # for each row
        sys.stdout.write("\r Record %d of %d" %
                         (t, traj.shape[0]))
        sys.stdout.flush()
        time, lcmd, rcmd = get_cmds_from_row(traj[t])
        # send these commands until the next frame
        while (rospy.get_time() - start_time) < time:
            if rospy.is_shutdown():
                print ("ROS shutdown, aborting")
                return
            left.set_joint_positions(lcmd)
            right.set_joint_positions(rcmd)
            # TODO: future gripper handling?
            rate.sleep()


def get_cmds_from_row(row, keys):
    limb_dof = 7
    row_list = row.tolist()
    row_dict = dict(zip(keys, row_list))

    # create a dictionary of joint to value for left limb
    # skip element 0 because it is the time param
    # skip elements 8, 16 because they are gripper params

    ldict = {k: row_dict[k] for k in keys[1:limb_dof+1]}
    rdict = {k: row_dict[k] for k in keys[limb_dof+2:-1]}

    return row[0], ldict, rdict


def get_limb_coordinate(left, right):
    return left.endpoint_pose(), right.endpoint_pose()


def plot_trajectory(traj):
    """
    Plot the given trajectory for each dof
    """
    for i in range(traj.shape[1]): # num dofs
        plt.subplot(3,6,i+1)
        plt.plot(traj[:,i], lw=2)
        plt.title("Traj for DOF="+str(i))
        plt.xlabel('timesteps')
        plt.ylabel('joint position')
    plt.show()



#
# if __name__ == "__main__":
# parser = argparse.ArgumentParser(description='Baxter controller')
# parser.add_argument('-d', '--dir', dest='dir', required=True)
# args = parser.parse_args()
#
# bc = BaxterUtils()
# bc.load_trajectories(args.dir)
# bc.run_trajectory()
#bc.plot_trajectory()

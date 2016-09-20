#!/usr/bin/env python

import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


def rotate(l):
    """
    Rotates a list left.

    @param l: the list
    """
    if len(l):
        v = l[0]
        l[:-1] = l[1:]
        l[-1] = v


def set_j(cmd, limb, joints, index, delta):
    """
    Set the selected joint to current pos + delta.

    @param cmd: the joint command dictionary
    @param limb: the limb to get the pos from
    @param joints: a list of joint names
    @param index: the index in the list of names
    @param delta: delta to update the joint by

    joint/index is to make this work in the bindings.
    """
    joint = joints[index]
    cmd[joint] = delta + limb.joint_angle(joint)


def map_joystick(joystick):
    """
    Maps joystick input to joint position commands.

    @param joystick: an instance of a Joystick
    """
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lcmd = {}
    rcmd = {}

    #available joints
    lj = left.joint_names()
    rj = right.joint_names()

    #abbreviations
    jhi = lambda s: joystick.stick_value(s) > 0
    jlo = lambda s: joystick.stick_value(s) < 0
    bdn = joystick.button_down
    bup = joystick.button_up

    def print_help(bindings_list):
        print("Press Ctrl-C to quit.")
        for bindings in bindings_list:
            for (test, _cmd, doc) in bindings:
                if callable(doc):
                    doc = doc()
                print("%s: %s" % (str(test[1][0]), doc))

    bindings_list = []
    bindings = (
        ((bdn, ['rightTrigger']),
         (grip_left.close,  []), "left gripper close"),
        ((bup, ['rightTrigger']),
         (grip_left.open,   []), "left gripper open"),
        ((bdn, ['leftTrigger']),
         (grip_right.close, []), "right gripper close"),
        ((bup, ['leftTrigger']),
         (grip_right.open,  []), "right gripper open"),
        ((jlo, ['leftStickHorz']),
         (set_j, [rcmd, right, rj, 0,  0.1]), lambda: "right inc " + rj[0]),
        ((jhi, ['leftStickHorz']),
         (set_j, [rcmd, right, rj, 0, -0.1]), lambda: "right dec " + rj[0]),
        ((jlo, ['rightStickHorz']),
         (set_j, [lcmd, left,  lj, 0,  0.1]), lambda: "left inc " + lj[0]),
        ((jhi, ['rightStickHorz']),
         (set_j, [lcmd, left,  lj, 0, -0.1]), lambda: "left dec " + lj[0]),
        ((jlo, ['leftStickVert']),
         (set_j, [rcmd, right, rj, 1,  0.1]), lambda: "right inc " + rj[1]),
        ((jhi, ['leftStickVert']),
         (set_j, [rcmd, right, rj, 1, -0.1]), lambda: "right dec " + rj[1]),
        ((jlo, ['rightStickVert']),
         (set_j, [lcmd, left,  lj, 1,  0.1]), lambda: "left inc " + lj[1]),
        ((jhi, ['rightStickVert']),
         (set_j, [lcmd, left,  lj, 1, -0.1]), lambda: "left dec " + lj[1]),
        ((bdn, ['rightBumper']),
         (rotate, [lj]), "left: cycle joint"),
        ((bdn, ['leftBumper']),
         (rotate, [rj]), "right: cycle joint"),
        ((bdn, ['btnRight']),
         (grip_left.calibrate, []), "left calibrate"),
        ((bdn, ['btnLeft']),
         (grip_right.calibrate, []), "right calibrate"),
        ((bdn, ['function1']),
         (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']),
         (print_help, [bindings_list]), "help"),
        )
    bindings_list.append(bindings)

    rate = rospy.Rate(100)
    print_help(bindings_list)
    print("Press Ctrl-C to stop. ")
    while not rospy.is_shutdown():
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                cmd[0](*cmd[1])
                if callable(doc):
                    print(doc())
                else:
                    print(doc)
        if len(lcmd):
            left.set_joint_positions(lcmd)
            lcmd.clear()
        if len(rcmd):
            right.set_joint_positions(rcmd)
            rcmd.clear()
        rate.sleep()
    return False


def main():
    joystick = baxter_external_devices.joystick.PS3Controller()

    print("Initializing node joystick_control... ")
    rospy.init_node("joystick_control")

    map_joystick(joystick)
    print("Done.")


if __name__ == '__main__':
    main()

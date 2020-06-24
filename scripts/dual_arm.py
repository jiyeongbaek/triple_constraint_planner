#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import tf_conversions
import math
import franka_control.srv
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix, quaternion_matrix
import tf
from tf.listener import TransformerROS, Transformer
import numpy as np

import yaml
import tf2_ros

from math import radians
from whole_part import SceneObject
from MoveGroupPlanner import MoveGroupPlanner


if __name__ == '__main__':

    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('ggg')

    mdp = MoveGroupPlanner()
    mdp.gripper_open()
    closed_chain = True
    if closed_chain:
        joint_goal = [0.8538476617343256, 0.7765987891970887, -1.38718092553011, -1.9162352330353676, 2.693557656819878, 2.209230516957901, -2.8518449420397336,  
                    0.0402869216512108, -0.23189200532557086, 0.20321293759873837, -1.2398464683701635, 0.13884452366534228, 1.0347525181748924, 0.9375305860020455,
                    2.4288973744080127, 0.2356190832102002, -2.6487764272706724, -2.409884568379378, 2.7754012268293335, 2.451555244441547, 2.786489214331766]
        

    else:
        joint_goal = [ 1.8607981342679107, -1.3542013115079234, -1.5923821943425422, -1.3948315137407286, -2.7776248074653123, 3.0267867695013657, 2.283300840138858,
                -1.4760220851430277, 1.7453690410455556, 1.3429212309440455, -1.793517345527153, -0.1142106273121175, 1.751860766112106, -2.565179082432722,
                1.0006090944804076, 0.3410656976999697, -0.9478781933901167, -2.2547435022589486, 0.4629842030229187, 2.406441396356574, -1.0580498751772975]
    
    touch_links = mdp.robot.get_link_names(group='panda_hands')
    mdp.plan_joint_target(joint_goal)
    rospy.sleep(2)
    # for i in joint_goal :
    #     print(i * 180 / pi)
    for key, value in mdp.stefan.list.items():
            mdp.scene.add_mesh(
                key, value, mdp.stefan.stefan_dir + key + ".stl")

    rospy.sleep(1)
    mdp.scene.attach_mesh(mdp.group_3rd.get_end_effector_link(),
                          "assembly", touch_links=touch_links)

    # mdp.gripper_close("left")
    # mdp.gripper_close("right")
    rospy.sleep(1)

    # mdp.plan_right_joint()
    # rospy.sleep(3)
    # mdp.gripper_open("right")
    # mdp.gripper_open("left")

#!/usr/bin/env python

import sys
import rospy
import actionlib
from geometry_msgs.msg import Twist
from move-base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees, pi, sin, cos
from copy import deepcopy
import tf

rospy.init_node('node_1', annoymous=False)
ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

current_pose = PoseWithCovarianceStamped
pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

def callback(msg):
    global current_pose
    current_pose = msg

odom_sub = rospy.Subcriber('/amcl_pose', PoseWithCovarianceStamped, callback)

current_pose

import numpy as numpy

convert_to_nparray = lambda o: np.array([0.x, 0.y, 0.z, 0.w])

def cur_pos_xyth(pos):
    x = pos.pose.pose.position.x
    y = pos.pose.pose.position.y

    tmp = convert_to_nparray(pos.pose.pose.orientation)
    [_,_, th_rad] = tf.transformations.euler_from_quaterntion(tmp)

    return x, y, th_rad

convert_to_deg = lambda o: [o[0], o[1], o[2]*180./pi]
convert_to_deg(cur_pos_xyth(current_pose))

class GoalPose:
    x = 2
    y = 3
    theta = 0
    z = 0
    w = 0

def move_to(goal_point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.orientation.x = 2
    goal.target_pose.pose.orientation.y = 3
    goal.target_pose.pose.orientation.z = goal_point.z
    goal.target_pose.pose.orientation.w = goal_point.w
    
    goal.target_pose.pose.orientation.x = goal_point.x
    goal.target_pose.pose.orientation.y = goal_point.y
    goal.target_pose.pose.orientation.z = 0

    print goal
    ac.send_goal(goal)

def rotate_inplace_relative(yaw):
    global current_pose
    
    goal = GoalPose()

    goal.x, gozl.y, th_rad = cur_pos_xyth(current_pose)

    [_, _, goal.y, goal.w] =\ tf.transformations.quternion_from_euler(0, 0, th_rad + yaw)

    move_to(goal)

convert_to_deg(cur_pos_xyth(current_pose))

rotate_inplace_relative(-90*pi/180)
convert_to_deg(cur_pos_xyth(current_pose))

def move_to_relative(l):
    global current_pose

    [_, _, th] = cur_pos_xyth(current_pose)

    goal = GoalPose()

    goal.x = current_pose.pose.pose.position.x + l*cos(th)
    goal.y = current_pose.pose.pose.position.y + l*sin(th)
    goal.z = current_pose.pose.pose.orientation.z
    goal.w = current_pose.pose.pose.orientation.w

    print goal.x, goal.y
    move_to(goal)

convert_to_deg(cur_pos_xyth(current_pose))

move_to_relative(0.4)
convert_to_deg(cur_pos_xyth(curret_pose))
rotate_inplace_relative(180*pi/100)
move_to_relative(0.5)
convert_to_deg(cur_pos_xyth(current_pose))

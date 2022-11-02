#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

rospy.init_node('init_pose')
pub_init = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)

init_pose = PoseWithCovarianceStamped()
init_pose.header.frame_id = "map"
init_pose.header.stamp = rospy.Time.now()
init_pose.pose.pose.position.x = 0.0
init_pose.pose.pose.position.y = 0.0
init_pose.pose.pose.orientation.w = 1.0

def update_init_pose(x, y, theta):
    init_pose.header.stamp = rospy.Time.now()
    init_pose.pose.pose.position.x = x
    init_pose.pose.pose.position.y = y
    init_pose.pose.pose.orientation.w = 1.0
    q = quaternion_from_euler(0.0, 0.0, theta)
    init_pose.pose.pose.orientation.x = q[0]
    init_pose.pose.pose.orientation.y = q[1]
    init_pose.pose.pose.orientation.z = q[2]
    init_pose.pose.pose.orientation.w = q[3]

update_init_pose(0.0, 0.0, 0.0)

pub_init.publish(init_pose)

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"

def send_goal(x,y,theta):
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0.0, 0.0, theta)
    goal.target_pose.pose.orientation.x=q[0]
    goal.target_pose.pose.orientation.y=q[1]
    goal.target_pose.pose.orientation.z=q[2]
    goal.target_pose.pose.orientation.w=q[3]
    client.send_goal(goal)
    

send_goal(2.0,3.0,0.0)
wait = client.wait_for_result()
if not wait:
    print('Error')
else:
    print(client.get_result())


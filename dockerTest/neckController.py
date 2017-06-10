#!/usr/bin/env python
#Contains Neck Controller Code
import sys
import rospy
from Task1 import ROBOT_NAME
import copy
from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
import time

neck_publisher = rospy.Publisher("/ihmc_ros/{0}/control/neck_trajectory".format(ROBOT_NAME), NeckTrajectoryRosMessage, queue_size=1)

def appendTrajectoryPoint(neck_trajectory, time, positions):
    if not neck_trajectory.joint_trajectory_messages:
        neck_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        neck_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return neck_trajectory

def adjustNeck(position, time):
	testmsg = NeckTrajectoryRosMessage()
	testmsg.unique_id = -1
	testmsg = appendTrajectoryPoint(testmsg, time, position)
	neck_publisher.publish(testmsg)

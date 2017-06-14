#!/usr/bin/env python
#Contains Pelvis Orientation Controller Code
import sys
import rospy
from Task1 import ROBOT_NAME
import copy
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
import tf
import tf2_geometry_msgs
import time as t
from ihmc_msgs.msg import PelvisOrientationTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
import PointCloudTask1 as PC

PelvisOrientTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/pelvis_orientation_trajectory".format(ROBOT_NAME), PelvisOrientationTrajectoryRosMessage, queue_size=1)

def appendTrajectoryPoint(pelvis_orientation_trajectory, time, rollPitchYaw):
	roll = rollPitchYaw[0]
	pitch = rollPitchYaw[1]
	yaw = rollPitchYaw[2]
	quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	point = copy.deepcopy(SO3TrajectoryPointRosMessage())
	point.time = time
	point.orientation = copy.deepcopy(Quaternion())
	point.orientation.x = quat[0]
	point.orientation.y = quat[1]
	point.orientation.z = quat[2]
	point.orientation.w = quat[3]
	point.angular_velocity = copy.deepcopy(Vector3())
	point.angular_velocity.x = 0
	point.angular_velocity.y = 0
	point.angular_velocity.z = 0
	pelvis_orientation_trajectory.taskspace_trajectory_points.append(point)
	return pelvis_orientation_trajectory

def adjustPelvisOrientation(position, time): 
	msg = PelvisOrientationTrajectoryRosMessage()
	msg.unique_id = -1
	msg.execution_mode = 0
	msg.previous_message_id = 0
	msg = appendTrajectoryPoint(msg, time, position) #Side to Side(+-20), 
	t.sleep(0.50)
	PelvisOrientTrajectoryPublisher.publish(msg)



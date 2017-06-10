#!/usr/bin/env python
#Contains Head Controller Code
import sys
import rospy
from ihmc_msgs.msg import HeadTrajectoryRosMessage
from Task1 import ROBOT_NAME
import copy
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
import tf
import time as t
from ihmc_msgs.msg import HeadTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import tf2_ros

headTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/head_trajectory".format(ROBOT_NAME), HeadTrajectoryRosMessage, queue_size=1)	
transform = None	
def transUpdater(trans):
	global transform
	transform = copy.deepcopy(trans)
def appendTrajectoryPoint(head_trajectory, time, rollPitchYaw):

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
	print transform
	print point.orientation
	head_trajectory.taskspace_trajectory_points.append(point)
	return head_trajectory

def adjustHead(position, time):
	#position = [South indian head, head nod, 
	msg = HeadTrajectoryRosMessage()
	msg.unique_id = -1
	msg = appendTrajectoryPoint(msg, time, position)
	t.sleep(0.50)
	headTrajectoryPublisher.publish(msg)

#JUNK HEAD CODE, CHOSEN CONTROLLER: NECK
def headLower():
	global transform
	transform = tf_buffer.lookup_transform("world", "head", rospy.Time(0), rospy.Duration(1.0))
	hC.transUpdater(transform)
	print O.color.BOLD + O.color.GREEN + "Adjusting Head to Default" + O.color.END
	hC.adjustHead([0.0, 0.0, 0.0], 1.0) #Jostles the Head in case stuck
	#t.sleep(5)
	hC.adjustHead([-0.5, 0.0, 0.0], 1.0)
	t.sleep(25)
	print O.color.BOLD + O.color.GREEN + "Adjusting Head to Fully Lowered" + O.color.END
	hC.adjustHead([-0.5, 1.1, 0.0], 1.0)
	t.sleep(10)


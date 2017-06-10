#!/usr/bin/env python
#Contains Arm Control Code
import sys
import rospy
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage
from Task1 import ROBOT_NAME
import copy



armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)
hand_publisher = rospy.Publisher("/ihmc_ros/{0}/control/hand_trajectory".format(ROBOT_NAME), HandTrajectoryRosMessage, queue_size=1)

#Preselected Values?
#[-1.0, 1.4, 1.1, 0.1, 0.8, 0.1, -0.0]
#[-1.0, 1.2, 1.1, 0.4, 0.8, 0.0, 0.2]

def appendTrajectoryPoint(arm_trajectory, time, positions):
	if not arm_trajectory.joint_trajectory_messages:
		arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
	for i, pos in enumerate(positions):
		point = TrajectoryPoint1DRosMessage()
		point.time = time
		point.position = pos
		point.velocity = 0
		arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
	return arm_trajectory

def ArmMsgMaker(time, side, POSITION):
	msg = ArmTrajectoryRosMessage()
	msg.execution_mode = 0
	msg.robot_side = side
	msg.unique_id = -1
	msg = appendTrajectoryPoint(msg, time, POSITION)
	armTrajectoryPublisher.publish(msg)

def HandMsgMaker(time, side, POSITION_COORD, ORIENTATION_COORD):

	testHand = HandTrajectoryRosMessage()
	testHand.robot_side = side
	testHand.unique_id = -1
	testHand.previous_message_id = 0
	testHand.execution_mode = 0
	testHand.base_for_control = HandTrajectoryRosMessage.WORLD #Control with respect to world	
	testHand.taskspace_trajectory_points = [copy.deepcopy(SE3TrajectoryPointRosMessage()) for i in range(1)]
	testHand.taskspace_trajectory_points[i].time = time
	testHand.taskspace_trajectory_points[i].position.x = POSITION_COORD[0]
	testHand.taskspace_trajectory_points[i].position.y = POSITION_COORD[1]
	testHand.taskspace_trajectory_points[i].position.z = POSITION_COORD[2] 
	testHand.taskspace_trajectory_points[i].orientation.x = ORIENTATION_COORD[0] 
	testHand.taskspace_trajectory_points[i].orientation.y = ORIENTATION_COORD[1] 
	testHand.taskspace_trajectory_points[i].orientation.z = ORIENTATION_COORD[2] 
	testHand.taskspace_trajectory_points[i].orientation.w = ORIENTATION_COORD[3] 
	hand_publisher.publish(testHand);


"""
def appendHandTrajectoryPoint(hand_trajectory, time, positions):
	if not hand_trajectory.taskspace_trajectory_points:
			hand_trajectory.taskspace_trajectory_points = [copy.deepcopy(SE3TrajectoryPointRosMessage()) for i in range(len(positions))]
	for i, pos in enumerate(positions):
			point = SE3TrajectoryPointRosMessage()
			point.time = time
			point.position = pos
			point.unique_id = -1
			hand_trajectory.taskspace_trajectory_points.append(point)
def _append_trajectory_point_so3(msg, time, joint_values):
	roll, pitch, yaw = joint_values
	quat = quaternion_from_euler(roll, pitch, yaw)
	point = SO3TrajectoryPointRosMessage()
	point.time = time
	point.orientation = Quaternion()
	point.orientation.x = quat[0]
	point.orientation.y = quat[1]
	point.orientation.z = quat[2]
	point.orientation.w = quat[3]
	point.angular_velocity = Vector3()
	point.angular_velocity.x = 0
	point.angular_velocity.y = 0
	point.angular_velocity.z = 0
	msg.taskspace_trajectory_points.append(point)
"""



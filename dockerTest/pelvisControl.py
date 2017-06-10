#!/usr/bin/env python
#Contains Pelvis Control Code
import sys
import rospy
from ihmc_msgs.msg import PelvisHeightTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from Task1 import ROBOT_NAME


PelvisHeightTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/pelvis_height_trajectory".format(ROBOT_NAME), PelvisHeightTrajectoryRosMessage, queue_size=1)

def adjustPelvis(height, time):
	testmsg = PelvisHeightTrajectoryRosMessage()
	testmsg.trajectory_points = []
	testmsg.execution_mode = 0
	testmsg.unique_id = -1
	#while LEFT ==0:
	testmsg = appendPelvisPoint(testmsg, time, height)
	PelvisHeightTrajectoryPublisher.publish(testmsg)
	
	PelvisHeightTrajectoryPublisher.publish(testmsg)
def appendPelvisPoint(pelvis_msg, time, position):
	if not pelvis_msg.trajectory_points:
		pelvis_msg.trajectory_points = [copy.deepcopy(TrajectoryPoint1DRosMessage()) for i in range(len(pelvis_msg.trajectory_points))]
	point = TrajectoryPoint1DRosMessage()
	point.time = time
	point.position = position
	point.velocity = 0
	pelvis_msg.trajectory_points.append(point)
	return pelvis_msg

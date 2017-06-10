#!/usr/bin/env python
import sys
import copy
import time as t
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf
import numpy
from numpy import append
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
import math as m
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import HandTrajectoryRosMessage
from geometry_msgs.msg import PoseStamped
ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
import armControl as aC
import pelvisControl as pC
import headController as hC
import neckController as nC
import handControl as hhC
import other as O
import PointCloudTask1 as PCT1
import torsoControl as tC
import pelvisOrient as pO


PointPUB = rospy.Publisher("/WORLDPOINT", PointStamped, queue_size = 0)
transform = transformP = "Stuff"


#Arm Trajectories
DEFAULT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
LEFT_ARM_DEFAULT = [-1.3, -0.6, 1.0, -1.3, 0.6, -0.6, -0.36]
RIGHT_ARM_DEFAULT = [-1.3, 0.6, 1.0, 1.3, 0.6, 0.6, 0.36]
def getSat(data):
	data.targetPitch
def getPoint(data, other):
	#other structure: other[0] = side, other[1] = time
	x = data.pose.position.x
	y = data.pose.position.y
	z = data.pose.position.z
	xO = data.pose.orientation.x
	yO = data.pose.orientation.y
	zO = data.pose.orientation.z

	MoveHand(x, y, z, xO, yO, zO, other[0], other[1])
	print O.color.CYAN + "Going to Point" + O.color.END
		
def MoveHand(x, y, z, xO, yO, zO, side, time):
	try:
		global transform, transformP
		global tf_buffer
		transform = tf_buffer.lookup_transform("world", "head", rospy.Time(0), rospy.Duration(1.0))
		headPoint = PoseStamped()
		headPoint.header.frame_id = "head"
		headPoint.pose.position.x = x
		headPoint.pose.position.y = y
		headPoint.pose.position.z = z

		worldPoint = tf2_geometry_msgs.do_transform_pose(headPoint, transform)
		POSITION = [worldPoint.pose.position.x, worldPoint.pose.position.y , worldPoint.pose.position.z ]

		transformP = tf_buffer.lookup_transform("world", "pelvis", rospy.Time(0), rospy.Duration(1.0))
		temp = tf.transformations.euler_from_quaternion((transformP.transform.rotation.x, transformP.transform.rotation.y, transformP.transform.rotation.z, transformP.transform.rotation.w))
		ORIENTATION = [xO+temp[0], yO+temp[1], zO+temp[2]]
		quat = tf.transformations.quaternion_from_euler(ORIENTATION[0], ORIENTATION[1], ORIENTATION[2])
		aC.HandMsgMaker(time, side, POSITION, quat)
		print O.color.GREEN + "Published Hand Trajectory" + O.color.END
		POINT = PointStamped()
		POINT.point.x = POSITION[0]
		POINT.point.y = POSITION[1]
		POINT.point.z = POSITION[2]

		POINT.header.frame_id ="world"
		PointPUB.publish(POINT)
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		print "HELP"
		rate.sleep()
def Task1Main():
	#CONTROL PANEL
	global transformP
	hhC.adjustRightHand([0.0, 0.0, 0.0, 0.0, 0.0])#Open Hand
	hhC.adjustLeftHand([0.0, 0.0, 0.0, 0.0, 0.0])
	

	Position = True
	#pO.adjustPelvisOrientation([0.0, 0.0, 0.0], 2.0)

	if Position:
		transformP = tf_buffer.lookup_transform("world", "pelvis", rospy.Time(0), rospy.Duration(1.0))
		print O.color.GREEN + "Arms to Default" + O.color.END
		aC.ArmMsgMaker(2.0, ArmTrajectoryRosMessage.RIGHT, RIGHT_ARM_DEFAULT)
		t.sleep(1)
		aC.ArmMsgMaker(2.0, ArmTrajectoryRosMessage.LEFT, LEFT_ARM_DEFAULT)
		t.sleep(5)
		print  O.color.GREEN + "Pelvis to 0.86" + O.color.END
		pC.adjustPelvis(0.87, 2.0)
		print O.color.GREEN + "Head to Lowered" + O.color.END
		nC.adjustNeck([0.5, 0.0, 0.0], 2.0)
		print O.color.GREEN + "Breaking Back" + O.color.END
		tC.pelvisTF([m.radians(0.00), m.radians(30.00), m.radians(0.00)], 1.0, transformP)
	else:
		print O.color.GREEN + "Pelvis to 1.0" + O.color.END
		pC.adjustPelvis(1.0, 2.0)
		print O.color.GREEN + "Head to Default" + O.color.END
		nC.adjustNeck([0.0, 0.0, 0.0], 1.0)

	print O.color.BOLD + O.color.CYAN + "Ready for Points" + O.color.END
	
	rospy.spin()

if __name__ == '__main__':
	try:
		global tf_buffer
		rospy.init_node('Task1', anonymous=True)
		tf_buffer = tf2_ros.Buffer(rospy.Duration(1000.0))
		tf_listener = tf2_ros.TransformListener(tf_buffer)
		RightHandSub = rospy.Subscriber("/SogiPoseR", PoseStamped, getPoint, (HandTrajectoryRosMessage.RIGHT, 0.50), queue_size = 1)
		LeftHandSub = rospy.Subscriber("/SogiPoseL", PoseStamped, getPoint, (HandTrajectoryRosMessage.LEFT, 0.50), queue_size = 1)

		rate = rospy.Rate(10) # 10hz
		t.sleep(1)
		if not O.PubCheck(True):
			sys.exit(1)
		# make sure the simulation is running otherwise wait

		if not rospy.is_shutdown():
			Task1Main()
			#main method ^^
			t.sleep(1)
	

	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python
################################################################################################################################################################################
#To use this code:
#Pass in 5 arguments (You must pass in all the arguments):
#first argument: enter 1 to execute the arm command
#second argument: enter 1 to execute the pelvis command
#third argument: enter 1 to execute the head command
#fourth argument: enter 1 to execute the back command
#fifth argument: enter 1 to set the whole body back to default
#EXample:: python Task1.py 1 1 1 1 0
#IF YOU DONT WANT TO USE ARGUMENTS, IRGNORE THIS
################################################################################################################################################################################

import sys
sys.path.insert(0,'..')
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
import argparse
import HelperCode as Cont
from std_msgs.msg import String

PointPUB = rospy.Publisher("/WORLDPOINT", PointStamped, queue_size = 0)
transform = transformP = "Stuff"
conn = rospy.Publisher("/Sendback", String, queue_size = 1)

#Arm Trajectories
DEFAULT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
LEFT_ARM_DEFAULT = [-1.3, -0.6, 1.0, -1.3, 0.6, -0.6, -0.36]
RIGHT_ARM_DEFAULT = [-1.3, 0.6, 1.0, 1.3, 0.6, 0.6, 0.36]


def getPoint(data, other):
	#other structure: other[0] = side, other[1] = time
	x = data.pose.position.x
	y = data.pose.position.y
	z = data.pose.position.z
	xO = data.pose.orientation.x
	yO = data.pose.orientation.y
	zO = data.pose.orientation.z

	MoveHand(x, y, z, xO, yO, zO, other[0], other[1])
	print Cont.O.color.CYAN + "Going to Point" + Cont.O.color.END
	stuff = String()
	stuff.data = "Going to Point"
	conn.publish(stuff)
		
def MoveHand(x, y, z, xO, yO, zO, side, time):
	if m.isnan(x):
		print Cont.O.BOLD + Cont.O.UNDERLINE + Cont.O.RED + "NAN Value received, Not Moving Arms" + Cont.O.END
		return False
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
		Cont.aC.HandMsgMaker(time, side, POSITION, quat)
		print Cont.O.color.GREEN + "Published Hand Trajectory" + Cont.O.color.END
		stuff = String()
		stuff.data = "Published Hand Trajectory"
		conn.publish(stuff)

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
	Cont.hhC.adjustRightHand([0.0, 0.0, 0.0, 0.0, 0.0])#Open Hand
	Cont.hhC.adjustLeftHand([0.0, 0.0, 0.0, 0.0, 0.0])

	Position = True
	#pO.adjustPelvisOrientation([0.0, 0.0, 0.0], 2.0)
	if Position:
		transformP = tf_buffer.lookup_transform("world", "pelvis", rospy.Time(0), rospy.Duration(1.0))
		print Cont.O.color.GREEN + "Arms to Default" + Cont.O.color.END
		Cont.aC.ArmMsgMaker(2.0, ArmTrajectoryRosMessage.RIGHT, RIGHT_ARM_DEFAULT)
		t.sleep(1)
		Cont.aC.ArmMsgMaker(2.0, ArmTrajectoryRosMessage.LEFT, LEFT_ARM_DEFAULT)
		t.sleep(5)
		print  Cont.O.color.GREEN + "Pelvis to 0.86" + Cont.O.color.END
		Cont.pC.adjustPelvis(0.87, 2.0)
		print Cont.O.color.GREEN + "Head to Lowered" + Cont.O.color.END
		Cont.nC.adjustNeck([0.5, 0.0, 0.0], 2.0)
		print Cont.O.color.GREEN + "Breaking Back" + Cont.O.color.END
		Cont.tC.pelvisTF([m.radians(0.00), m.radians(30.00), m.radians(0.00)], 1.0, transformP)
		stuff = String()
		stuff.data = "In POSITION"
		conn.publish(stuff)
	else:
		transformP = tf_buffer.lookup_transform("world", "pelvis", rospy.Time(0), rospy.Duration(1.0))
		print Cont.O.color.GREEN + "Pelvis to 1.0" + Cont.O.color.END
		Cont.pC.adjustPelvis(1.0, 2.0)
		print Cont.O.color.GREEN + "Head to Default" + Cont.O.color.END
		Cont.nC.adjustNeck([0.0, 0.0, 0.0], 1.0)
		print Cont.O.color.GREEN + "Un-Breaking Back" + Cont.O.color.END
		Cont.tC.pelvisTF([m.radians(0.00), m.radians(0.00), m.radians(0.00)], 1.0, transformP)

	print Cont.O.color.BOLD + Cont.O.color.CYAN + "Ready for Points" + Cont.O.color.END
	
	rospy.spin()

def Task1MainO(arm, pelvis, head, back, default):
	#CONTROL PANEL
	global transformP
	Cont.hhC.adjustRightHand([0.0, 0.0, 0.0, 0.0, 0.0])#Open Hand
	Cont.hhC.adjustLeftHand([0.0, 0.0, 0.0, 0.0, 0.0])
	print arm, pelvis, head, back, default

	#Position = True
	#pO.adjustPelvisOrientation([0.0, 0.0, 0.0], 2.0)
	
	#if Position:
	if arm == '1' :
		print Cont.O.color.GREEN + "Arms to Default" + Cont.O.color.END
		Cont.aC.ArmMsgMaker(2.0, ArmTrajectoryRosMessage.RIGHT, RIGHT_ARM_DEFAULT)
		t.sleep(1)
		Cont.aC.ArmMsgMaker(2.0, ArmTrajectoryRosMessage.LEFT, LEFT_ARM_DEFAULT)
		t.sleep(5)
	if pelvis == '1' :
		print  Cont.O.color.GREEN + "Pelvis to 0.86" + Cont.O.color.END
		Cont.pC.adjustPelvis(0.87, 2.0)
	if head == '1' :
		print Cont.O.color.GREEN + "Head to Lowered" + Cont.O.color.END
		Cont.nC.adjustNeck([0.5, 0.0, 0.0], 2.0)
	if back == '1' :
		transformP = tf_buffer.lookup_transform("world", "pelvis", rospy.Time(0), rospy.Duration(1.0))
		print Cont.O.color.GREEN + "Breaking Back" + Cont.O.color.END
		Cont.tC.pelvisTF([m.radians(0.00), m.radians(30.00), m.radians(0.00)], 1.0, transformP)
	#else:
	if default=='1' :
		print Cont.O.color.GREEN + "Pelvis to 1.0" + Cont.O.color.END
		Cont.pC.adjustPelvis(1.0, 2.0)
		print Cont.O.color.GREEN + "Head to Default" + Cont.O.color.END
		Cont.nC.adjustNeck([0.0, 0.0, 0.0], 1.0)
		transformP = tf_buffer.lookup_transform("world", "pelvis", rospy.Time(0), rospy.Duration(1.0))
		print Cont.O.color.GREEN + "Un-Breaking Back" + Cont.O.color.END
		Cont.tC.pelvisTF([m.radians(0.00), m.radians(0.00), m.radians(0.00)], 1.0, transformP)
		

	print Cont.O.color.BOLD + Cont.O.color.CYAN + "Ready for Points" + Cont.O.color.END
	
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
		if not Cont.O.PubCheck(True):
			sys.exit(1)
		# make sure the simulation is running otherwise wait

		if not rospy.is_shutdown():
			if len(sys.argv) > 1 :
				arm  = sys.argv[1]
				pelvis = sys.argv[2]
				head = sys.argv[3]
				back = sys.argv[4]
				default = sys.argv[5]
				Task1MainO(arm, pelvis, head, back, default)
				#main method ^^
			else :
				Task1Main()
			t.sleep(1)
	

	except rospy.ROSInterruptException:
		pass

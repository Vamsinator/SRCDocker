#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import Task1 as T1
import other as O
import numpy as np
import math as m
import PointCloudTask1 as PC
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import time as t
from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from srcsim.msg import Satellite
import handControl as hhC
import makeMarker as mC
import torsoControl as tC

knobsFound = 0
Calibrated = False
Knob = [[],[]]

Xaverage = []
Yaverage = []
Zaverage = []
CPoints = []
Xval = 0.0
Yval = 0.0
Zval = 0.0

#Greencolor = ([0, 200, 0] , [50,255,50])
Bluecolor =  ([0, 0, 100], [50, 50, 255])
#Bluecolor =  ([0, 0, 0], [255, 255, 255])
Redcolor = ([70, 0, 0], [200, 10, 10])
Greycolor = ([20, 20, 20], [170, 170, 170]) #?

def PointPubber(S, SO, side):

	if side == 0 :
		
		LPOINT = PointStamped()
		LPOINT.point.y, LPOINT.point.z, LPOINT.point.x = S
		LPOINT.header.frame_id ="head"
		PC.LPointPUB.publish(LPOINT)

		LPOSE = PoseStamped()
		LPOSE.header.frame_id = "head"
		LPOSE.pose.position.x = LPOINT.point.x
		LPOSE.pose.position.y = LPOINT.point.y
		LPOSE.pose.position.z = LPOINT.point.z
		LPOSE.pose.orientation.x = SO[0]
		LPOSE.pose.orientation.y = SO[1]
		LPOSE.pose.orientation.z = SO[2]
		
		PC.LComm.publish(LPOSE)

	else :

		RPOINT = PointStamped()
		RPOINT.point.y, RPOINT.point.z, RPOINT.point.x = S
		RPOINT.header.frame_id ="head"
		PC.RPointPUB.publish(RPOINT)

		RPOSE = PoseStamped()
		RPOSE.header.frame_id = "head"
		RPOSE.pose.position.x = RPOINT.point.x
		RPOSE.pose.position.y = RPOINT.point.y
		RPOSE.pose.position.z = RPOINT.point.z
		RPOSE.pose.orientation.x = SO[0]
		RPOSE.pose.orientation.y = SO[1]
		RPOSE.pose.orientation.z = SO[2]

		PC.RComm.publish(RPOSE)

def PointCloudGen(d, side):
	GenPointCloud = PointCloud()
	GenPointCloud.header.frame_id = "head"
	for i in range(len(d)):
		point = Point32()
		point.x = d[i][2]
		point.y = -d[i][0]+0.03 #Needed for aligntment (idk why)
		point.z = -d[i][1]
		GenPointCloud.points.append(point)
	t.sleep(0.05)
	PC.Choosen.publish(GenPointCloud)
def ResetCalcValues():
	global Xaverage, Yaverage, Zaverage, CPoints, Xval, Yval, Zval
	Xaverage = [[], []]
	Yaverage = [[], []]
	Zaverage = [[], []]
	Xval = 0.0
	Yval = 0.0
	Zval = 0.0
	CPoints = []

def FindKnobs(PointList):
	global Xaverage, Yaverage, Zaverage, CPoints, Xval, Yval, Zval
	ResetCalcValues()
	#Finding Handles
	Trimmed = []
	for x in PointList:
		distance = pow(x[0]**2+x[1]**2+x[2]**2, 0.5)
		if distance < 1.0:
			Trimmed.append(x)
	if False: #For Verbosity
		print O.color.GREEN + "Point Trimmed:", len(Trimmed),  O.color.END
		print O.color.GREEN + "Point Count:  ", len(PointList) ,O.color.END
	
	for x in Trimmed:
		temp1 = np.greater_equal(x[3], Redcolor[0])
		temp2 = np.less_equal(x[3], Redcolor[1])
		temp3 = np.array_equal([True, True, True], temp1) and np.array_equal([True, True, True], temp2)
		if temp3:
			Zaverage[0].append(x[2])
			#Yaverage[0].append(x[1])
			#Xaverage[0].append(x[0])

		temp1 = np.greater_equal(x[3], Bluecolor[0])
		temp2 = np.less_equal(x[3], Bluecolor[1])
		temp3 = np.array_equal([True, True, True], temp1) and np.array_equal([True, True, True], temp2)
		if temp3:
			Zaverage[1].append(x[2])
			#Yaverage[1].append(x[1])
			#Xaverage[1].append(x[0])
	ZLTemp= np.average(Zaverage[0]) #Finding the average distance from Head to Wheel and using that to filter
	ZRTemp= np.average(Zaverage[1]) #Done for Left and Right, so if robot is not entirely aligned it should work
	#Xval = XTemp= np.average(Xaverage[0])
	#Yval = YTemp= np.average(Yaverage[0])
	ResetCalcValues()
	
	for x in Trimmed:
		temp1 = np.greater_equal(x[3], Greycolor[0])
		temp2 = np.less_equal(x[3], Greycolor[1])
		temp3 = np.array_equal([True, True, True], temp1) and np.array_equal([True, True, True], temp2)

		if x[3][0] == x[3][1] and x[3][2] == x[3][1] and temp3: ##Is grey and isn't white
			if x[0] >=0: #Left or right
				if x[2]<=(ZLTemp-0.095)*1.05 and x[2] >=(ZRTemp-0.095)*0.95: #Is closer than the average red/blue part and not too close (+-5%)
					Xaverage[0].append(x[0])
					Yaverage[0].append(x[1])
					Zaverage[0].append(x[2])
					CPoints.append(x)
			else:
				if x[2]<=(ZRTemp-0.095)*1.05 and x[2] >=(ZRTemp-0.095)*0.95:
					Xaverage[1].append(x[0])
					Yaverage[1].append(x[1])
					Zaverage[1].append(x[2])
					CPoints.append(x)
	if len(CPoints) < 200:
		print O.color.BOLD + O.color.RED + "FATAL PROBLEM" + O.color.END #ADD EXCEPTION
		#sys.exit(2)
	PointCloudGen(CPoints, 1)
	LeftPoint = [-np.average(Xaverage[0])+.03, -np.average(Yaverage[0]) , np.average(Zaverage[0])] # Left(-) or right(+), up down, forward backwards
	RightPoint = [-np.average(Xaverage[1])+0.03, -np.average(Yaverage[1]) , np.average(Zaverage[1])]
	ResetCalcValues()
	return LeftPoint, RightPoint
def FindOutside(PointList):
	global Xaverage, Yaverage, Zaverage, CPoints, Xval, Yval, Zval
	ResetCalcValues()
	#Finding Handles
	Trimmed = []
	for x in PointList:
		distance = pow(x[0]**2+x[1]**2+x[2]**2, 0.5)
		if distance < 1.0:
			Trimmed.append(x)
	if False: #For Verbosity
		print O.color.GREEN + "Point Trimmed:", len(Trimmed),  O.color.END
		print O.color.GREEN + "Point Count:  ", len(PointList), O.color.END
		
	for x in Trimmed:
		temp1 = np.greater_equal(x[3], Redcolor[0])
		temp2 = np.less_equal(x[3], Redcolor[1])
		temp3 = np.array_equal([True, True, True], temp1) and np.array_equal([True, True, True], temp2)
		if temp3:
			Zaverage[0].append(x[2])
			Yaverage[0].append(x[1])
			Xaverage[0].append(x[0])

		temp1 = np.greater_equal(x[3], Bluecolor[0])
		temp2 = np.less_equal(x[3], Bluecolor[1])
		temp3 = np.array_equal([True, True, True], temp1) and np.array_equal([True, True, True], temp2)
		if temp3:
			Zaverage[1].append(x[2])
			Yaverage[1].append(x[1])
			Xaverage[1].append(x[0])
	L = [np.average(Xaverage[0]), np.average(Yaverage[0]), np.average(Zaverage[0])] #Finds Center of Red Wheel
	R = [np.average(Xaverage[1]), np.average(Yaverage[1]), np.average(Zaverage[1])] #Finding Center of Blue Wheel
	L = [-np.average(Xaverage[0])+0.03-0.15, -np.average(Yaverage[0]) , np.average(Zaverage[0])] # Left(-) or right(+), up down, forward backwards
	R = [-np.average(Xaverage[1])+0.03+0.15, -np.average(Yaverage[1]) , np.average(Zaverage[1])]

	ResetCalcValues()
	print "L: ", L, "R: ", R
	return L, R
def ResetArms():

		t.sleep(0.5)
		LEFT_ARM_DEFAULT = [-1.3, -0.6, 1.0, -1.3, 0.6, -0.6, -0.36]
		RIGHT_ARM_DEFAULT = [-1.3, 0.6, 1.0, 1.3, 0.6, 0.6, 0.36]
		T1.aC.ArmMsgMaker(2.0, ArmTrajectoryRosMessage.RIGHT, RIGHT_ARM_DEFAULT)
		t.sleep(0.5)
		T1.aC.ArmMsgMaker(2.0, ArmTrajectoryRosMessage.LEFT, LEFT_ARM_DEFAULT)
		hhC.adjustRightHand([0.0, 0.0, 0.0, 0.0, 0.0])
		hhC.adjustLeftHand([0.0, 0.0, 0.0, 0.0, 0.0])
def getSat(data):
	return

def LClockwise(L1, L2, L3, L4, LO):
	PointPubber(L1, LO, 0)
	t.sleep(3)
	print O.color.CYAN + "Side Touching" , O.color.END
	PointPubber(L2, LO, 0)
	t.sleep(4)
	print O.color.CYAN + "Moving Down" , O.color.END
	PointPubber(L3, LO, 0)
	t.sleep(2)
	print O.color.CYAN + "Moving Out" , O.color.END
	PointPubber(L4, LO, 0)
	print O.color.CYAN + "Moving Start" , O.color.END

def RClockwise(R1, R2, R3, R4, RO):
	PointPubber(R1, RO, 1)
	t.sleep(3)
	print O.color.CYAN + "Side Touching" , O.color.END
	PointPubber(R2, RO, 1)
	t.sleep(4)
	print O.color.CYAN + "Moving Down" , O.color.END
	PointPubber(R3, RO, 1)
	t.sleep(2)
	print O.color.CYAN + "Moving Out" , O.color.END
	PointPubber(R4, RO, 1)
	print O.color.CYAN + "Moving Start" , O.color.END

def LCounter(L1, L2, L3, L4, LO):
	PointPubber(L1, LO, 0)
	t.sleep(3)
	print O.color.CYAN + "Side Touching" , O.color.END
	PointPubber(L2, LO, 0)
	t.sleep(4)
	print O.color.CYAN + "Moving Down" , O.color.END
	PointPubber(L3, LO, 0)
	t.sleep(2)
	print O.color.CYAN + "Moving Out" , O.color.END
	PointPubber(L4, LO, 0)
	print O.color.CYAN + "Moving Start" , O.color.END

def RCounter(R1, R2, R3, R4, RO):
	PointPubber(R1, RO, 1)
	t.sleep(3)
	print O.color.CYAN + "Side Touching" , O.color.END
	PointPubber(R2, RO, 1)
	t.sleep(4)
	print O.color.CYAN + "Moving Down" , O.color.END
	PointPubber(R3, RO, 1)
	t.sleep(2)
	print O.color.CYAN + "Moving Out" , O.color.END
	PointPubber(R4, RO, 1)
	print O.color.CYAN + "Moving Start" , O.color.END

def Task1Processor(PointList):
	global knobsFound, Calibrated
	side = 0; #change to make different side (0==LEFT, 1==RIGHT)
	LeftPoint, RightPoint = FindOutside(PointList)
	mC.addMarker(LeftPoint, 0)
	t.sleep(0.5)
	mC.addMarker(RightPoint, 1)

	#CALIBRATION CODE Runs until Calibrated
	Calibrated = True
	if Calibrated:
		LO = [m.radians(-20), m.radians(-20), m.radians(-130)] #
		RO = [m.radians(20), m.radians(-20), m.radians(130)] # RED, GREEN, BLUE based on Palm
		
		LPOS1 = [LeftPoint[0], LeftPoint[1]-0.15, LeftPoint[2]+0.01]
		RPOS1 = [RightPoint[0], RightPoint[1]-0.15, RightPoint[2]+0.01]
		LPOS2 = [LeftPoint[0]+0.05, LeftPoint[1]+0.15, LeftPoint[2]+0.02]
		RPOS2 = [RightPoint[0]-0.05, RightPoint[1]+0.15, RightPoint[2]+0.02]
		LPOS3 = [LeftPoint[0]-0.15, LeftPoint[1]+0.15, LeftPoint[2]]
		RPOS3 = [RightPoint[0]+0.15, RightPoint[1]+0.15, RightPoint[2]]
		LPOS4 = [LeftPoint[0]-0.15, LeftPoint[1]-0.15, LeftPoint[2]-.01]
		RPOS4 = [RightPoint[0]+0.15, RightPoint[1]-0.15, RightPoint[2]-.01]
		
		LeftPoint, RightPoint = FindOutside(PointList)
		mC.addMarker(LeftPoint, 0)
		t.sleep(0.5)
		mC.addMarker(RightPoint, 1)

		LCounter(LPOS1, LPOS2, LPOS3, LPOS4, LO)
		RClockwise(RPOS1, RPOS2, RPOS3, RPOS4, RO)
		# Left or right, up down, forward backwards

		"""
		print O.color.CYAN + "Side Grabbing Prep", O.color.END
		hhC.adjustRightHand([0.0, 0.150, 0.150, 0.150, 0.150])
		t.sleep(0.1)
		hhC.adjustLeftHand([0.0, -0.150, -0.150, -0.150, -0.150])
		t.sleep(2)
		"""

		knobsFound = True
	
	#PointPubber([-0.34016937380940521, 0.019932347534164653, 0.52386487879592547] , LO, [0.32556296228522719, 0.025278871880395491, 0.52586259550472902], RO)

	
		



	





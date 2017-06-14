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
import neckController as nC

firstTime = True
RO = [m.radians(00), m.radians(90), m.radians(90)]
LO = [m.radians(00), m.radians(90), m.radians(-90)]

yC = yT = pC = pT = 0

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

def PointCloudGen(d):
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
			variance = 0.05
			if x[0] >=0: #Left or right
				if x[2]<=(ZLTemp-0.07)*(1+variance) and x[2] >=(ZRTemp-0.090)*(1-variance): #Is closer than the average red/blue part and not too close (+-5%)
					Xaverage[0].append(x[0])
					Yaverage[0].append(x[1])
					Zaverage[0].append(x[2])
					CPoints.append(x)
			else:
				if x[2]<=(ZRTemp-0.07)*(1+variance) and x[2] >=(ZRTemp-0.090)*(1-variance):
					Xaverage[1].append(x[0])
					Yaverage[1].append(x[1])
					Zaverage[1].append(x[2])
					CPoints.append(x)
	if len(CPoints) < 200:
		print O.color.BOLD + O.color.RED + "FATAL PROBLEM" + O.color.END #ADD EXCEPTION
		#sys.exit(2)
	PointCloudGen(CPoints)
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
	#PointCloudGen(CPoints)
	L = [np.average(Xaverage[0]), np.average(Yaverage[0]), np.average(Zaverage[0])] #Finds Center of Red Wheel
	R = [np.average(Xaverage[1]), np.average(Yaverage[1]), np.average(Zaverage[1])] #Finding Center of Blue Wheel
	L = [-np.average(Xaverage[0])+0.03-0.15, -np.average(Yaverage[0]) , np.average(Zaverage[0])] # Left(-) or right(+), up down, forward backwards
	R = [-np.average(Xaverage[1])+0.03+0.15, -np.average(Yaverage[1]) , np.average(Zaverage[1])]

	ResetCalcValues()
	#print "L: ", L, "R: ", R
	return L, R
def ResetArms():
	#Resets Arms
	global RO, LO
	PointPubber([0.2, -0.2, 0.32] , RO, 1) # RED, GREEN, BLUE based on Palm
	hhC.adjustRightHand([0.0, 0.0, 0.0, 0.0, 0.0])
	t.sleep(0.1)
	PointPubber([-0.2, -0.2, 0.32] , LO, 0)
	hhC.adjustLeftHand([0.0, 0.0, 0.0, 0.0, 0.0])

def getSat(data):
	global yC, yT, pC, pT
	yC = data.current_yaw
	yT = data.target_yaw
	pC = data.current_pitch
	pT = data.target_pitch
	#print "hello"
	return

def RCC(RightPoint):
	global RO, LO
	print "RightCounter Started" #OUTSIDE
	PointPubber([RightPoint[0]-0.05, RightPoint[1]+0.3, RightPoint[2]+0.02], RO, 1)
	t.sleep(15)
	PointPubber([RightPoint[0]-0.05, RightPoint[1]-0.1, RightPoint[2]+0.02], RO, 1)
	t.sleep(10)
	PointPubber([RightPoint[0]-0.05, RightPoint[1]-0.15, RightPoint[2]-0.2], RO, 1)
	t.sleep(10)
	PointPubber([RightPoint[0]-0.05, RightPoint[1]+0.3, RightPoint[2]-0.2], RO, 1)
	t.sleep(5)
	print "RightCounter Ended"

def RC(RightPoint):
	global RO, LO
	print "RightClockwise Started" #INSIDE
	PointPubber([RightPoint[0]-0.3, RightPoint[1]+0.2, RightPoint[2]+0.02], RO, 1)
	t.sleep(10)
	PointPubber([RightPoint[0]-0.3, RightPoint[1]-0.2, RightPoint[2]+0.02], RO, 1)
	t.sleep(10)
	PointPubber([RightPoint[0]-0.3, RightPoint[1]-0.3, RightPoint[2]-0.15], RO, 1)
	t.sleep(10)
	PointPubber([RightPoint[0]-0.3, RightPoint[1]+0.2, RightPoint[2]-0.15], RO, 1)
	t.sleep(5)
	print "RightClockwise Ended"
def LCC(LeftPoint):
	global RO, LO
	print "LeftCounter Started"
	PointPubber([LeftPoint[0]+0.3, LeftPoint[1]+0.2, LeftPoint[2]+0.02], LO, 0)
	t.sleep(10)
	PointPubber([LeftPoint[0]+0.3, LeftPoint[1]-0.2, LeftPoint[2]+0.02], LO, 0)
	t.sleep(10)
	PointPubber([LeftPoint[0]+0.3, LeftPoint[1]-0.3, LeftPoint[2]-0.15], LO, 0)
	t.sleep(10)
	PointPubber([LeftPoint[0]+0.3, LeftPoint[1]+0.2, LeftPoint[2]-0.15], LO, 0)
	t.sleep(5)
	print "LeftCounter Ended"
def LC(LeftPoint):
	global RO, LO
	print "LeftClockwise Started"
	PointPubber([LeftPoint[0]+0.05, LeftPoint[1]+0.3, LeftPoint[2]+0.02], LO, 0)
	t.sleep(15)
	PointPubber([LeftPoint[0]+0.05, LeftPoint[1]-0.1, LeftPoint[2]+0.02], LO, 0)
	t.sleep(10)
	PointPubber([LeftPoint[0]+0.05, LeftPoint[1]-0.15, LeftPoint[2]-0.2], LO, 0)
	t.sleep(10)
	PointPubber([LeftPoint[0]+0.05, LeftPoint[1]+0.3, LeftPoint[2]-0.2], LO, 0)
	t.sleep(5)
	print "LeftClockwise Ended"

def Task1Processor(PointList):
	global firstTime
	global yC, yT, pC, pT
	#Angles of Left and Right Arm	
	#Prints out Current Angles, or Success
	if m.fabs(pC - pT) > m.radians(5):
		print O.color.BOLD + O.color.UNDERLINE + O.color.DARKCYAN
		print "Pitch (Blue) Delta: ", m.fabs(pC - pT)
		print O.color.END
	else:
		print O.color.GREEN + O.color.BOLD + "PITCH GOOD!" + O.color.END
	if m.fabs(yC - yT) > m.radians(5):
		print O.color.BOLD + O.color.UNDERLINE + O.color.DARKCYAN
		print "Yaw (Red) Delta: ", m.fabs(yC - yT)
		print O.color.END
	else:
		print O.color.GREEN + O.color.BOLD + "YAW GOOD!" + O.color.END

	#Finds Point, and Publishes
	LeftPoint, RightPoint = FindOutside(PointList)

	mC.addMarker(RightPoint, 1)
	mC.addMarker(LeftPoint, 0)

	#Moves arms from Task1.py Reset Values to Proper reset values
	if firstTime:
		print LeftPoint, RightPoint
		ResetArms()
		print "First Time only"
		t.sleep(20)
		firstTime = False

	#Adjusts hands to clenched
	hhC.adjustRightHand([0.0, 1.0, 1.0, 1.0, 1.0])
	hhC.adjustLeftHand([0.0, -1.0, -1.0, -1.0, -1.0])
	
	#Moves arms in right directions
	if m.fabs(pC - pT) > m.radians(5): #BLUE
		print O.color.CYAN
		if pC > pT:
			RCC(RightPoint)
		else:
			RC(RightPoint)
	if m.fabs(yC - yT) > m.radians(5): #RED
		if yC < yT:
			LC(LeftPoint)
		else:
			LCC(LeftPoint)
		print O.color.END








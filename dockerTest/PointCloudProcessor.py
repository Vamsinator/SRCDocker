#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.insert(0,'..')
import numpy as np
import math as m
import rospy
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
import HelperCode as Cont
from std_msgs.msg import String


LComm = rospy.Publisher("/SogiPoseL", PoseStamped, queue_size = 1)
RComm = rospy.Publisher("/SogiPoseR", PoseStamped, queue_size = 1)
LPointPUB = rospy.Publisher("/LeftHandle", PointStamped, queue_size = 0)
RPointPUB = rospy.Publisher("/RightHandle", PointStamped, queue_size = 0)

conn = rospy.Publisher("/Sendback", String, queue_size = 1)

Choosen = rospy.Publisher("/SogiPoints", PointCloud, queue_size = 1)

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
	Choosen.publish(GenPointCloud)
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
		stuff = String()
		stuff.data = "PointList Less than 200!"
		print stuff.data
		conn.publish(stuff)
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
	if len(Xaverage[0]) == 0 or len(Xaverage[1]) == 0:
		print Cont.O.color.RED + Cont.O.color.BOLD + "CAN'T SEE RED OR BLUE WHEEL" + Cont.O.color.END
		stuff = String()
		stuff.data = "CAN'T SEE RED OR BLUE WHEEL"
		print stuff.data
		conn.publish(stuff)
		return 0, 0
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
	Cont.mC.PointPubber([0.2, -0.2, 0.32] , RO, 1) # RED, GREEN, BLUE based on Palm
	Cont.hhC.adjustRightHand([0.0, 0.0, 0.0, 0.0, 0.0])
	t.sleep(0.1)
	Cont.mC.PointPubber([-0.2, -0.2, 0.32] , LO, 0)
	Cont.hhC.adjustLeftHand([0.0, 0.0, 0.0, 0.0, 0.0])

def getSat(data):
	global yC, yT, pC, pT
	yC = data.current_yaw
	yT = data.target_yaw
	pC = data.current_pitch
	pT = data.target_pitch
	#print "hello"
	return

def RCC(RightPoint, z):
	global RO, LO
	print "RightCounter Started" #OUTSIDE
	Cont.mC.PointPubber([RightPoint[0]-0.05, RightPoint[1]+0.3, RightPoint[2]+0.02], RO, 1)
	t.sleep(15)
	Cont.mC.PointPubber([RightPoint[0]-0.05, RightPoint[1]-0.1, RightPoint[2]+0.02], RO, 1)
	t.sleep(10)
	Cont.mC.PointPubber([RightPoint[0]-0.05, RightPoint[1]-0.15, RightPoint[2]-0.2], RO, 1)
	t.sleep(10)
	Cont.mC.PointPubber([RightPoint[0]-0.05, RightPoint[1]+0.3, RightPoint[2]-0.2], RO, 1)
	t.sleep(5)
	print "RightCounter Ended"

def RC(RightPoint, z):
	global RO, LO
	print "RightClockwise Started" #INSIDE
	Cont.mC.PointPubber([RightPoint[0]-0.3, RightPoint[1]+0.2, RightPoint[2]+0.02], RO, 1)
	t.sleep(10)
	Cont.mC.PointPubber([RightPoint[0]-0.3, RightPoint[1]-0.2, RightPoint[2]+0.02], RO, 1)
	t.sleep(10)
	Cont.mC.PointPubber([RightPoint[0]-0.3, RightPoint[1]-0.3, RightPoint[2]-0.15], RO, 1)
	t.sleep(10)
	Cont.mC.PointPubber([RightPoint[0]-0.3, RightPoint[1]+0.2, RightPoint[2]-0.15], RO, 1)
	t.sleep(5)
	print "RightClockwise Ended"
def LCC(LeftPoint, z):
	global RO, LO
	print "LeftCounter Started"
	Cont.mC.PointPubber([LeftPoint[0]+0.3, LeftPoint[1]+0.2, LeftPoint[2]+0.02], LO, 0)
	t.sleep(10)
	Cont.mC.PointPubber([LeftPoint[0]+0.3, LeftPoint[1]-0.2, LeftPoint[2]+0.02], LO, 0)
	t.sleep(10)
	Cont.mC.PointPubber([LeftPoint[0]+0.3, LeftPoint[1]-0.3, LeftPoint[2]-0.15], LO, 0)
	t.sleep(10)
	Cont.mC.PointPubber([LeftPoint[0]+0.3, LeftPoint[1]+0.2, LeftPoint[2]-0.15], LO, 0)
	t.sleep(5)
	print "LeftCounter Ended"
def LC(LeftPoint, z):
	global RO, LO
	print "LeftClockwise Started"
	Cont.mC.PointPubber([LeftPoint[0]+0.05, LeftPoint[1]+0.3, LeftPoint[2]+0.02], LO, 0)
	t.sleep(15)
	Cont.mC.PointPubber([LeftPoint[0]+0.05, LeftPoint[1]-0.1, LeftPoint[2]+0.02], LO, 0)
	t.sleep(10)
	Cont.mC.PointPubber([LeftPoint[0]+0.05, LeftPoint[1]-0.15, LeftPoint[2]-0.2], LO, 0)
	t.sleep(10)
	Cont.mC.PointPubber([LeftPoint[0]+0.05, LeftPoint[1]+0.3, LeftPoint[2]-0.2], LO, 0)
	t.sleep(5)
	print "LeftClockwise Ended"

def makeAdjustments(LeftPoint, RightPoint, LLC, LLCC, RRC, RRCC):
	if LLC == '1' :
		LC(LeftPoint)
	if LLCC == '1' :
		LCC(LeftPoint)
	if RRC == '1' :
		RC(RightPoint)
	if RRCC == '1' :
		RCC(RightPoint)

def Task1Processor(PointList):
	global firstTime
	global yC, yT, pC, pT
	#Angles of Left and Right Arm	
	#Prints out Current Angles, or Success
	if m.fabs(pC - pT) > m.radians(5):
		print Cont.O.color.BOLD + Cont.O.color.UNDERLINE + Cont.O.color.DARKCYAN
		print "Pitch (Blue) Delta: ", m.fabs(pC - pT)
		stuff = String()
		stuff.data = "Pitch (Blue) Delta: ", m.fabs(pC - pT)
		conn.publish(stuff)
		print Cont.O.color.END
	else:
		print Cont.O.color.GREEN + Cont.O.color.BOLD + "PITCH GOOD!" + Cont.O.color.END
		stuff = String()
		stuff.data = "PITCH GOOD!"
		conn.publish(stuff)
	if m.fabs(yC - yT) > m.radians(5):
		print Cont.O.color.BOLD + Cont.O.color.UNDERLINE + Cont.O.color.DARKCYAN
		print "Yaw (Red) Delta: ", m.fabs(yC - yT)
		stuff = String()
		stuff.data = "Yaw (Red) Delta: ", m.fabs(yC - yT)
		conn.publish(stuff)
		print Cont.O.color.END
	else:
		print Cont.O.color.GREEN + Cont.O.color.BOLD + "YAW GOOD!" + Cont.O.color.END
		stuff = String()
		stuff.data = "YAW GOOD"
		conn.publish(stuff)

	#Finds Point, and Publishes
	LeftPoint, RightPoint = FindOutside(PointList)
	stuff = String()
	stuff.data = "LEFT POINT: " + str(LeftPoint) + " RIGHT POINT: " + str(RightPoint)
	conn.publish(stuff)
	if LeftPoint == 0:
		return False
		#Stops the method from continuing

	Cont.mC.addMarker(RightPoint, 1)
	Cont.mC.addMarker(LeftPoint, 0)

	#Moves arms from Task1.py Reset Values to Proper reset values
	if firstTime:
		print LeftPoint, RightPoint
		ResetArms()
		print "First Time only"
		stuff = String()
		stuff.data = "First Time Only"
		conn.publish(stuff)
		t.sleep(20)
		firstTime = False

	#Adjusts hands to clenched
	Cont.hhC.adjustRightHand([0.0, 1.0, 1.0, 1.0, 1.0])
	Cont.hhC.adjustLeftHand([0.0, -1.0, -1.0, -1.0, -1.0])
	
	#Moves arms in right directions
	if m.fabs(pC - pT) > m.radians(5): #BLUE
		print Cont.O.color.CYAN
		if pC > pT:
			RCC(RightPoint, 0)
		else:
			RC(RightPoint, 0)
	if m.fabs(yC - yT) > m.radians(5): #RED
		if yC < yT:
			LC(LeftPoint, 0)
		else:
			LCC(LeftPoint, 0)
		print Cont.O.color.END

def Task1ProcessorO(PointList, LC, LCC, RC, RCC, zAdjust, which):
	global firstTime
	global yC, yT, pC, pT
	#Angles of Left and Right Arm	
	#Prints out Current Angles, or Success
	if m.fabs(pC - pT) > m.radians(5):
		print Cont.O.color.BOLD + Cont.O.color.UNDERLINE + Cont.O.color.DARKCYAN
		print "Pitch (Blue) Delta: ", m.fabs(pC - pT)
		stuff = String()
		stuff.data = "Pitch (Blue) Delta: ", m.fabs(pC - pT)
		conn.publish(stuff)
		print Cont.O.color.END
	else:
		print Cont.O.color.GREEN + Cont.O.color.BOLD + "PITCH GOOD!" + Cont.O.color.END
		stuff = String()
		stuff.data = "PITCH GOOD!"
		conn.publish(stuff)
	if m.fabs(yC - yT) > m.radians(5):
		print Cont.O.color.BOLD + Cont.O.color.UNDERLINE + Cont.O.color.DARKCYAN
		print "Yaw (Red) Delta: ", m.fabs(yC - yT)
		stuff = String()
		stuff.data = "Yaw (Red) Delta: ", m.fabs(yC - yT)
		conn.publish(stuff)
		print Cont.O.color.END
	else:
		print Cont.O.color.GREEN + Cont.O.color.BOLD + "YAW GOOD!" + Cont.O.color.END
		stuff = String()
		stuff.data = "YAW GOOD!"
		conn.publish(stuff)

	#Finds Point, and Publishes
	LeftPoint, RightPoint = FindOutside(PointList)

	Cont.mC.addMarker(RightPoint, 1)
	Cont.mC.addMarker(LeftPoint, 0)

	#Moves arms from Task1.py Reset Values to Proper reset values
	if firstTime:
		print LeftPoint, RightPoint
		ResetArms()
		print "First Time only"
		stuff = String()
		stuff.data = "First Time only"
		conn.publish(stuff)
		t.sleep(20)
		firstTime = False

	#Adjusts hands to clenched
	Cont.hhC.adjustRightHand([0.0, 1.0, 1.0, 1.0, 1.0])
	Cont.hhC.adjustLeftHand([0.0, -1.0, -1.0, -1.0, -1.0])
	
	#Moves arms in right directions
	print Cont.O.color.CYAN
	if m.fabs(pC - pT) > m.radians(5): #BLUE
		if pC > pT:
			RCC(RightPoint, 0)
		else:
			RC(RightPoint, 0)
	if m.fabs(yC - yT) > m.radians(5): #RED
		if yC < yT:
			LC(LeftPoint, 0)
		else:
			LCC(LeftPoint, 0)
	print Cont.O.color.END

	makeAdjustments(LeftPoint, RightPoint, LC, LCC, RC, RCC)

	if zAdjust != '0' :
		if which == '1' :
			LC(LeftPoint, zAdjust)
		
		if which == '2' :
			LCC(LeftPoint, zAdjust)
		
		if which == '3' :
			RC(RightPoint, zAdjust)
		
		if which == '4' :	
			RCC(RightPoint, zAdjust)








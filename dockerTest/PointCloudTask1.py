#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import PointCloudProcessor as PCP
import numpy as np
import rospy
from timeit import default_timer
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from sensor_msgs import point_cloud2 as pc2
import ctypes
import struct
from srcsim.msg import Console
from scipy import ndimage
import progressbar
import time as t
import tf2_ros
import tf2_geometry_msgs
import tf
import neckController as nC
import message_filters
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from ihmc_msgs.msg import ChestTrajectoryRosMessage
import random
import visualProcessor as VP
from srcsim.msg import Satellite
import other as O
gatekeeper = 0
c = 0

LPointPUB = rospy.Publisher("/LeftHandle", PointStamped, queue_size = 0)
RPointPUB = rospy.Publisher("/RightHandle", PointStamped, queue_size = 0)
Choosen = rospy.Publisher("/SogiPoints", PointCloud, queue_size = 1)

LComm = rospy.Publisher("/SogiPoseL", PoseStamped, queue_size = 1)
RComm = rospy.Publisher("/SogiPoseR", PoseStamped, queue_size = 1)

chestTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/chest_trajectory".format(rospy.get_param('/ihmc_ros/robot_name')), ChestTrajectoryRosMessage, queue_size=1)	

def CloudPreProcessor(pointcloud):
	global Pub, gatekeeper, PointsList, tf_buffer, c
	if gatekeeper % 4 == 0:
		start = default_timer()
		print O.color.CYAN + "PointCloud Aquired" , O.color.END
		gen = pc2.read_points(pointcloud, skip_nans=True)
		count = 0
		if False:
			XVAL = 0.0
			YVAL = 0.0
			ZVAL = 0.0
			DISTANCE = 1000	
			while(DISTANCE > 0.60):
				XVAL = (random.random()*0.6+0.2) #Side to Side [0.2, 0.6]
				YVAL = (-random.random()*0.9+0.6) #LEFT and RIGHT [ -0.6, 0.3]
				ZVAL = random.random()*0.6+0.1 #UP AND DOWN [ 0.1, 0.5]
				DISTANCE = pow(XVAL**2+YVAL**2+ZVAL**2, 0.5)
		if True:
			PointsList = []
			for point in list(gen): #For a single point
				test = point[3]
				s = struct.pack('>f', test)
				i = struct.unpack('>l', s) [0]
				pack = ctypes.c_uint32(i).value
				x = point[0]
				y = point[1]
				z = point[2]
				r = (pack & 0x00FF0000)>> 16
				g = (pack & 0x0000FF00)>> 8
				b = (pack & 0x000000FF)
				color = np.array([r, g, b])
				distance = pow(x**2+y**2+z**2, 0.5)
				PointsList.append([x, y, z, color])
				count=count + 1
			PCP.Task1Processor(PointsList)
		#Point are reversed since head is upside down
		print "Time Taken: " + str(default_timer() - start)
	gatekeeper += 1

def Final1():
	#Initiates Node
	global tf_buffer
	rospy.init_node('Final1', anonymous=True)
	tf_buffer = tf2_ros.Buffer(rospy.Duration(1000.0))
	tf_listener = tf2_ros.TransformListener(tf_buffer)
	#Subscription setup
	rospy.Subscriber("/multisense/camera/points2", PointCloud2, CloudPreProcessor, queue_size=1, buff_size= 2**8)
	rospy.Subscriber("/task1/checkpoint2/satellite", Satellite, PCP.getSat, queue_size = 2)

	rate = rospy.Rate(10) #hz
	rospy.spin()

	
if __name__ == '__main__':
	try:
		Final1()
	except rospy.ROSInterruptException:
		pass

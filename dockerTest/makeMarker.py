#!/usr/bin/env python
#Contains the creation of a marker Code
import sys
import rospy
import copy
import tf
from visualization_msgs.msg import Marker
import PointCloudTask1 as PC

RmarkerPUB = rospy.Publisher('/visualization_markerR', Marker, queue_size = 1)
LmarkerPUB = rospy.Publisher('/visualization_markerL', Marker, queue_size = 1)

def addMarker(position, side):

	mark = Marker()
	mark.header.frame_id = "head"

	mark.header.stamp = rospy.Time.now()
	mark.type = Marker.CYLINDER
	#Change Orientation
	mark.pose.position.x = position[2]
	mark.pose.position.z = position[1]
	mark.pose.orientation.x = 0.0
	mark.pose.orientation.y = 1.0
	mark.pose.orientation.z = 0.0
	mark.pose.orientation.w = 1.0
	#mark.scale.x = 0.05
	#mark.scale.y = 0.05
	#mark.scale.z = 0.12
	mark.scale.x = .04
	mark.scale.y = .04
	mark.scale.z = .1
	mark.color.a = 0.5
	if side ==0 :
		mark.color.r = 1.0
		mark.pose.position.y = position[0]
		LmarkerPUB.publish(mark)
	else:
		mark.color.b = 1.0
		mark.pose.position.y = position[0]
		RmarkerPUB.publish(mark)

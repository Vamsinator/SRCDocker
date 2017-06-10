#!/usr/bin/env python

#import roslib; roslib.load_manifest('ellipse_marker')
from visualization_msgs.msg import Marker
import rospy
import time

rospy.init_node('ellipse', anonymous=True)
publisher = rospy.Publisher("ellipse", Marker, queue_size = 1)
count = 0

while not rospy.is_shutdown():
    ellipse = Marker()
    ellipse.header.frame_id = "odom"
    ellipse.header.stamp = rospy.Time.now()
    ellipse.type = Marker.CYLINDER
    ellipse.action = Marker.ADD
    ellipse.pose.position.x = .5
    ellipse.pose.position.y = .75
    ellipse.pose.position.z = 1.0
    ellipse.pose.orientation.x = 0.0
    ellipse.pose.orientation.y = 0.0
    ellipse.pose.orientation.z = 0.0
    ellipse.pose.orientation.w = 1.0
    ellipse.scale.x = 2*count
    ellipse.scale.y = 2*count
    ellipse.scale.z = 1
    ellipse.color.a = 1.0
    ellipse.color.r = 1.0
    ellipse.color.g = 1.0
    ellipse.color.b = 1.0
    count += 0.1

    # Publish the MarkerArray
    publisher.publish(ellipse)

    time.sleep(1.0)

"""
import copy
import time
import rospy
import tf
import tf2_ros
import numpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def addPalmMarker():
	POSITION = [.5, .75, 1.0]
	ORIENTATION = [0.10, 0.40, 0.0, 0.0]
	WorldPoint = Marker()
	WorldPoint.header.frame_id = "1"
	WorldPoint.header.stamp.secs = rospy.get_rostime().secs
	WorldPoint.ns = "world"
	WorldPoint.id = 0
	WorldPoint.type = 3
	WorldPoint.action = 0
	WorldPoint.pose.position.x = POSITION[0]
	WorldPoint.pose.position.y = POSITION[1]
	WorldPoint.pose.position.z = POSITION[2]
	WorldPoint.pose.orientation.x = ORIENTATION[0]
	WorldPoint.pose.orientation.y = ORIENTATION[1]
	WorldPoint.pose.orientation.z = ORIENTATION[2]
	WorldPoint.pose.orientation.w = ORIENTATION[3]
	WorldPoint.scale.x = WorldPoint.scale.y = 0.5
	WorldPoint.scale.z = 1
	WorldPoint.color.r = WorldPoint.color.g = WorldPoint.color.b = 1
	WorldPoint.color.a = 0.5
	WorldPoint.lifetime.secs = 2
	#Point.x = Point.y = Point.z = 10
	
	pub.publish(WorldPoint)


if __name__ == '__main__':

	try:
		rospy.init_node('register', anonymous=True)
		pub = rospy.Publisher("visualization_marker", Marker, queue_size = 1)

		if not rospy.is_shutdown():
			addPalmMarker()
			time.sleep(1)
	

	except rospy.ROSInterruptException:
		pass
"""
	





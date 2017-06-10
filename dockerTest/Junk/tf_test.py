#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs

if __name__ == '__main__':
	rospy.init_node('tf_turtle')
	rate = rospy.Rate(10.0)
	tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
	tf_listener = tf2_ros.TransformListener(tf_buffer)
	try:
		transform = tf_buffer.lookup_transform("world", "head", rospy.Time(0), rospy.Duration(1.0))
		xOff = transform.transform.translation.x
		yOff = transform.transform.translation.y
		zOff = transform.transform.translation.z
		print xOff, yOff, zOff
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		print "HELP"
		rate.sleep()

	print "Hello"

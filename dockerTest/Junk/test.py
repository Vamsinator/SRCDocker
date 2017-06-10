#!/usr/bin/env python
import rospy
x = 10
def test1():
	global x
	print x

print "HELLO"


if __name__ == '__main__':
	rospy.init_node('Task1', anonymous=True)
	test1()

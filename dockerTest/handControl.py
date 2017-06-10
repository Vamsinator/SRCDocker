#!/usr/bin/env python
#Contains Hand Controller Code
import sys
import rospy
#from Task1 import ROBOT_NAME
from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage
import time
import copy
import numpy
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
left_hand_publisher = rospy.Publisher('/left_hand_position_controller/command', Float64MultiArray, queue_size=1)
right_hand_publisher = rospy.Publisher('/right_hand_position_controller/command', Float64MultiArray, queue_size=1)
def adjustRightHand(position):
        Rmsg = Float64MultiArray()

        dim = MultiArrayDimension()
        dim.label = 'fingers'
        dim.size = 5
        dim.stride = 5
        Rmsg.layout.dim = [dim]
        Rmsg.layout.data_offset = 0
        Rmsg.data = position

	right_hand_publisher.publish(Rmsg)

def adjustLeftHand(position):
	Lmsg = Float64MultiArray()

        dim = MultiArrayDimension()
        dim.label = 'fingers'
        dim.size = 5
        dim.stride = 5
        Lmsg.layout.dim = [dim]
        Lmsg.layout.data_offset = 0
        Lmsg.data = position

        left_hand_publisher.publish(Lmsg)

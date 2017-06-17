#!/usr/bin/env python
import sys
sys.path.insert(0,'..')
import numpy as np
import copy
import rospy
import io as logreader
#import os
import time
import math as m
from sensor_msgs.msg import Image
from std_msgs.msg import Float64 
#import mpmath as mp
from srcsim.msg import Console
from scipy import ndimage, stats
import socket
#from walkObj import walk
import HelperCode as Cont
from ihmc_msgs.msg import ArmTrajectoryRosMessage
import tf2_ros
import tf2_geometry_msgs
import tf
from srcsim.msg import Leak
from geometry_msgs.msg import Point
from std_msgs.msg import String

Limage = 0
Rimage = 0
TCP_IP = '192.168.2.10'
TCP_PORT = 8009 #ENTER PORT STUFF HERE
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((TCP_IP, TCP_PORT))
sock.listen(True)
conn, addr = sock.accept()
#Walker = walk()
tf_buffer = "Bla Bla"

Oldvalue = Newvalue = 0
Ox = Oy = Oz = Nx = Ny = Nz = 0
def Comm():
	global tf_buffer
	try:
		while True:
			try:
				data = conn.recv(1024)
				dataArray = data.split(" ")
				sock.close()
				#LEFTARM 1.0 0 0 0 0 0 0 0
				if dataArray[0] == "LEFTARM":
					if len(dataArray) != 9:
						print Cont.O.color.RED + "INVALID # OF ARGS FOR LEFT ARM (REQUIRES 9)" + Cont.O.color.END
					else:
						Position = [float(dataArray[2]), float(dataArray[3]), float(dataArray[4]), float(dataArray[5]), float(dataArray[6]), float(dataArray[7]), float(dataArray[8])]
						Cont.aC.ArmMsgMaker(float(dataArray[1]), ArmTrajectoryRosMessage.LEFT, Position)
						stuff = String()
						stuff.data = "LEFT ARM PUBLISHED"
						print stuff.data
						conne.publish(stuff)
				#RIGHTARM 1.0 0 0 0 0 0 0 0
				if dataArray[0] == "RIGHTARM":
					if len(dataArray) != 9:
						print Cont.O.color.RED + "INVALID # OF ARGS FOR RIGHT ARM (REQUIRES 9)" + str(len(dataArray)) + Cont.O.color.END
					else:
						Position = [float(dataArray[2]), float(dataArray[3]), float(dataArray[4]), float(dataArray[5]), float(dataArray[6]), float(dataArray[7]), float(dataArray[8])]
						Cont.aC.ArmMsgMaker(float(dataArray[1]), ArmTrajectoryRosMessage.RIGHT, Position)
						stuff = String()
						stuff.data = "RIGHT ARM PUBLISHED"
						print stuff.data
						conne.publish(stuff)
				#LEFTHAND 1.0 0 0 0 0 0
				if dataArray[0] == "LEFTHAND":
					if len(dataArray) != 6:
						print Cont.O.color.RED + "INVALID # OF ARGS FOR LEFT HAND (REQUIRES 6)" + Cont.O.color.END
					else:
						Position = [float(dataArray[1]), float(dataArray[2]), float(dataArray[3]), float(dataArray[4]), float(dataArray[5])]

						Cont.hhC.adjustLeftHand(Position)

						stuff = String()
						stuff.data = "LEFT HAND PUBLISHED"
						print stuff.data
						conne.publish(stuff)
				#RIGHTHAND 1.0 0 0 0 0 0 
				if dataArray[0] == "RIGHTHAND":
					if len(dataArray) != 6:
						print Cont.O.color.RED + "INVALID # OF ARGS FOR RIGHT HAND (REQUIRES 6)" + Cont.O.color.END
					else:
						Position = [float(dataArray[1]), float(dataArray[2]), float(dataArray[3]), float(dataArray[4]), float(dataArray[5])]
					
						Cont.hhC.adjustRightHand(Position)

						stuff = String()
						stuff.data = "RIGHT HAND PUBLISHED"
						print stuff.data
						conne.publish(stuff)
				#NECK 2.0 0 0 0
				if dataArray[0] == "NECK":
					if len(dataArray) != 5:
						print Cont.O.color.RED + "INVALID # OF ARGS FOR NECK (REQUIRES 5)" + Cont.O.color.END
					else:
						Position =  [float(dataArray[2]), float(dataArray[3]), float(dataArray[4])]
						
						Cont.nC.adjustNeck(Position, float(dataArray[1]))

						stuff = String()
						stuff.data = "NECK PUBLISHED"
						print stuff.data
						conne.publish(stuff)
				#TORSO 1.0 0.00 30.00 0.00
				if dataArray[0] == "TORSO":
					if len(dataArray) != 5:
						print Cont.O.color.RED + "INVALID # OF ARGS FOR TORSO (REQUIRES 5)" + Cont.O.color.END
					else:
						Position =  [m.radians(float(dataArray[2])), m.radians(float(dataArray[3])), m.radians(float(dataArray[4]))]
						
						transformP = tf_buffer.lookup_transform("world", "pelvis", rospy.Time(0), rospy.Duration(1.0))
						Cont.tC.pelvisTF(Position, float(dataArray[1]), tf_buffer.lookup_transform("world", "pelvis", rospy.Time(0), rospy.Duration(1.0)))

						stuff = String()
						stuff.data = "TORSO PUBLISHED"
						print stuff.data
						conne.publish(stuff)
				#PELVIS 1.0 0	
				if dataArray[0] == "PELVIS":
					if len(dataArray) !=3:
						print Cont.O.color.RED + "INVALID # OF ARGS FOR PELVIS (REQUIRES 3)" + Cont.O.color.END
					else:
						POSITION = float(dataArray[2])
						
						Cont.pC.adjustPelvis(POSITION, float(dataArray[1]))
						stuff = String()
						stuff.data = "PELVIS PUBLISHED"
						print stuff.data
						conne.publish(stuff)
				#RIGHTMOVE 1.0 0 0 0 0 0 0 
				if dataArray[0] == "RIGHTMOVE":
					if len(dataArray) != 8:
						print Cont.O.color.RED + "INVALID # OF ARGS FOR RIGHTMOVE (REQUIRES 8)" + Cont.O.color.END
					else:
						POSITION = [float(dataArray[2]), float(dataArray[3]), float(dataArray[4])]
						ORIENTATION = [float(dataArray[5]), float(dataArray[6]), float(dataArray[7])]
						
						Cont.aC.MoveHand(float(dataArray[2]), float(dataArray[3]), float(dataArray[4]), float(dataArray[5]), float(dataArray[6]), float(dataArray[7]), ArmTrajectoryRosMessage.RIGHT, float(dataArray[1]), tf_buffer)
						stuff = String()
						stuff.data = "RIGHTMOVE PUBLISHED"
						print stuff.data
						conne.publish(stuff)
				#LEFTMOVE 1.0 0 0 0 0 0 0 
				if dataArray[0] == "LEFTMOVE":
					if len(dataArray) != 8:
						print Cont.O.color.RED + "INVALID # OF ARGS FOR LEFTMOVE (REQUIRES 8)" + Cont.O.color.END
					else:
						POSITION = [float(dataArray[2]), float(dataArray[3]), float(dataArray[4])]
						ORIENTATION = [float(dataArray[5]), float(dataArray[6]), float(dataArray[7])]
						
						Cont.aC.MoveHand(float(dataArray[2]), float(dataArray[3]), float(dataArray[4]), float(dataArray[5]), float(dataArray[6]), float(dataArray[7]), ArmTrajectoryRosMessage.LEFT, float(dataArray[1]), tf_buffer)

						stuff = String()
						stuff.data = "LEFTMOVE PUBLISHED"
						print stuff.data
						conne.publish(stuff)
				#Walker.walkTest(float(dataArray[0]), float(dataArray[1]), float(dataArray[2]))

			except socket.error:
				continue
	except KeyboardInterrupt:
		print "ENDED"
		sock.close()
		sys.exit(4)

def getLeak(data):
	if data.value>0.01:
		print "Leak detected"
		stuff = String()
		stuff.data = "Leak detected"
		conne.publish(stuff)
if __name__ == '__main__':
	try:
		global tf_buffer
		rospy.init_node("Controller", anonymous=True)
		rate = rospy.Rate(10)
		tf_buffer = tf2_ros.Buffer(rospy.Duration(1000.0))
		tf_listener = tf2_ros.TransformListener(tf_buffer)
		rospy.Subscriber("/task3/checkpoint5/leak", Leak, getLeak, queue_size = 2)
		conne = rospy.Publisher("/Sendback", String, queue_size = 1)
		#Calls back to a "smart method"
		Comm()
		sock.close()
	except rospy.ROSInterruptException:
		pass

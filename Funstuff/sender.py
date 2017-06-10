import numpy as np
import copy
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import io as logreader
import os
import time
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Float64 
#import mpmath as mp
from srcsim.msg import Console
from scipy import ndimage, stats
import socket
bridge = CvBridge()

Limage = 0
Rimage = 0
TCP_IP = 'localhost'
TCP_PORT = 32000 #ENTER PORT STUFF HERE
sock = socket.socket()
sock.connect((TCP_IP, TCP_PORT))

"""
Uses Black and white filter to find the path
"""

#Greencolor = ([0, 100, 0] , [50,255,50])
Bluecolor =  ([0, 0, 70], [50, 50, 255])
Redcolor = ([95, 0, 0], [255, 50, 50])
Whitecolor = ([170, 170, 170], [255, 255, 255])
#Graycolor = ([51, 51, 51],[128, 128, 128])
Blackcolor = ([0, 0, 0], [60, 60, 60])
nomAngle = [math.pi/2.0, math.pi/2.0]
LEFT = 0
RIGHT = 1
def left(data):
	global LEFT
	Limage = preprocessor(data)
	Limage = processor(Limage, LEFT)
	
def right(data):
	global RIGHT
	Rimage = preprocessor(data)
	Rimage = processor(Rimage, RIGHT)
	
def Final1():
	#Initiates Node
	#Subscription setup
	rospy.Subscriber("/multisense/camera/left/image_color", Image, left, queue_size=1, buff_size = 2**30)
	#rospy.Subscriber("/multisense/camera/right/image_color", Image, right, queue_size=1, buff_size = 2**30)
	#Publisher Setup
	rate = rospy.Rate(10) #hz
	rospy.spin()
        global socket
        #socket.close()

def preprocessor(image):
	#Converts from ROS
	tempimage = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
	#Rotates image 180
	tempimage = ndimage.rotate(tempimage, 180)
	#Returns image
	return tempimage
buff = []
def addToBuffer(angle):
    #places angle in a buffer and then publishes the median angle
    buff.append(angle)
    while len(buff)>9:
	buff.pop(0)
    publish = copy.deepcopy(buff)
    publish.sort()
    angPub.publish(publish[len(publish)/2])
def processor(image, side):
	Threshold = 60
	MetThresh = 0
	colordetected = False
	mask = 0
	#crop image to focus more on the path in front
	crop_img = image
        Blackmask = image
	#cv2.imshow("image crop", crop_img)
	#Blackmask = cv2.inRange(image, np.array(Blackcolor[0], dtype="uint8"),  np.array(Blackcolor[1], dtype="uint8"))
	mus = 0.0
	#return the sum of the weight function over the point
	firstCol = 0.0
	lastCol = 0.0
	if not np.array_equal(np.zeros_like(Blackmask), Blackmask):
	    global sock
            encode_param=[int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, imgencode = cv2.imencode('.jpg', Blackmask, encode_param)
            data = np.array(imgencode)
            stringData = data.tostring()
            sock.send( str(len(stringData)).ljust(16))
            sock.send(stringData);
	return mask

if __name__ == '__main__':
	try:
		rospy.init_node('Pathfind', anonymous=True)
		angPub = rospy.Publisher('bwAngle', Float64, queue_size=10)
		angPub.publish(360.0)
		Final1()
	except rospy.ROSInterruptException:
		pass



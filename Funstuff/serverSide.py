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
from walkObj import walk

Limage = 0
Rimage = 0
TCP_IP = '192.168.2.10'
TCP_PORT = 32000 #ENTER PORT STUFF HERE
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((TCP_IP, TCP_PORT))
sock.listen(True)
conn, addr = sock.accept()
Walker = walk()
try:
    while True:
        try:
            data = conn.recv(1024)
        
            dataArray = data.split(" ")
            Walker.walkTest(float(dataArray[0]), float(dataArray[1]), float(dataArray[2]))
        except socket.error:
            continue
except KeyboardInterrupt:
    pass

s.close()

#decimg = cv2.imdecode(data, 1)
#cv2.imshow('ROBOT', decimg)

import numpy as np
import copy
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import io as logreader
import os
import time
import math
import socket
flag = 0
Limage = 0
Rimage = 0
TCP_IP = 'localhost'
TCP_PORT = 32000 #ENTER PORT STUFF HERE
sock = socket.socket()
sock.connect((TCP_IP, TCP_PORT))
def appendPoints(dataList):
    global data, flag,sock
    data = str(dataList)
    flag = 1
    try:
        sock.send(data)
    except socket.error:
        return
def closeSocket():
    global sock
    sock.close()
#decimg = cv2.imdecode(data, 1)
#cv2.imshow('ROBOT', decimg)

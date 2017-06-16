import numpy as np
import copy
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import io as logreader
import os
import time
import math
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Float64 
#import mpmath as mp
from srcsim.msg import Console
from scipy import ndimage, stats
import socket
bridge = CvBridge()
from walkObj import walk
Walker = walk()
Limage = 0
Rimage = 0
TCP_IP = '172.17.0.1' 
TCP_PORT = 32000 #ENTER PORT STUFF HERE
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((TCP_IP, TCP_PORT))
sock.listen(True)
conn, addr = sock.accept()
#Walker = walk()
"""
import armControl as ac
import handControl as hC
import neckController as nC
import pelvisControl as pC
import torsoControl as tC
import pelvisOrient as pO
import headControl as hhC
"""
import subprocess
try:
    while True:
        try:
            q = None
            data = conn.recv(1024)
            if data == "START":
                q = Subprocess.pOpen(["python", "Task1.py"])
            if data == "STOP":
                if q == None:
                    print "Null error"
                    continue
                os.killpg(os.getpg(q.pgid), signal.SIGTERM)
            dataArray = data.split(" ")
            strF = dataArray[0]
            Walker.walkTest(float(dataArray[0]), float(dataArray[1]), float(dataArray[2]))
            
            """
            if strF == "hl":
                hC.adjustLeftHand()
            if strF == "hr":
                hC.adjustRightHand()
            if strF == "n":
                nC.adjustNeck()
            if strF == "pC":
                pC.adjustPelvis()
            if strF == "tC":
                tC.adjustChest()
            if strF == "pO":
                pO.adjustPelvisOrientation()
            if strF == "hhC":
                hhC.adjustHead()
            """
        except socket.error:
            continue
except KeyboardInterrupt:
    sock.close()
    pass

sock.close()

#decimg = cv2.imdecode(data, 1)
#cv2.imshow('ROBOT', decimg)

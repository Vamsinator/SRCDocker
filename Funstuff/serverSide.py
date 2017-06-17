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
from std_msgs.msg import Float64, String 
#import mpmath as mp
from srcsim.msg import Console
from scipy import ndimage, stats
import socket
bridge = CvBridge()
from walkObj import walk
import robotCaller as rCall
Walker = walk()
Limage = 0
Rimage = 0
TCP_IP = '192.168.2.10' 
TCP_PORT = 33000 #ENTER PORT STUFF HERE
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((TCP_IP, TCP_PORT))
sock.listen(True)
conn, addr = sock.accept()
#from nbstreamreader import NonBlockingStreamReader as NBSR
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
pCloud = None
q = None
try:
    while True:
        try:
            data = conn.recv(1024)
            dataArray = data.split(" ")
            if dataArray[0] == "STARTT1":
                if len(dataArray)==1:
                    q = subprocess.Popen(["python", "../dockerTest/Task.py"])
                    continue
                q = subprocess.Popen(["python", "../dockerTest/Task1.py", dataArray[1], dataArray[2], dataArray[3], dataArray[4], dataArray[5]], stdout=subprocess.PIPE)
                nbT1 = NBSR(q.stdout)
                continue
            if dataArray[0] == "STOPT1":
                if q == None:
                    print "Null error"
                    continue
                os.killpg(os.getpgid(q.pid), signal.SIGTERM)
                q = None
                continue
            if dataArray[0] == "STARTP1":
                if len(dataArray)==1:
                    pCloud = subprocess.Popen(["python", "../dockerTest/PointCloudProcessor.py"])
                    continue
                pCloud = subprocess.Popen(["python", "../dockerTest/PointCloudProcessor.py", dataArray[1], dataArray[2], dataArray[3], dataArray[4], dataArray[5], dataArray[6]], stdout=subprocess.PIPE)
                continue
            if dataArray[0] == "STOPP1":
                if pCloud == None:
                    print "null error"
                    continue
            
            strF = dataArray[0]
            Walker.walkTest(float(dataArray[0]), float(dataArray[1]), float(dataArray[2]))
            rospy.Subscriber("/Sendback", String, callback) 
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
def callback(msg):
    global rCall
    if msg.data!= '':
        rCall.appendPoints(msg.data)
    return
#decimg = cv2.imdecode(data, 1)
#cv2.imshow('ROBOT', decimg)

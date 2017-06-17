#!/usr/bin/env python
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


def revcall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf+= newbuf
        count -= len(newbuf)
    return buf
Limage = 0
Rimage = 0
TCP_IP = '192.168.0.2'
TCP_PORT = 31000 #ENTER PORT STUFF HERE
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((TCP_IP, TCP_PORT))
sock.listen(True)
conn, addr = sock.accept()
try:
    while 1:
        length = revcall(conn, 16)
        stringData = revcall(conn, int(length))
        data = np.fromstring(stringData, dtype='uint8')
        decimg = cv2.imdecode(data, 1)
        cv2.line(decimg, (145, 0), (145, 544), (0, 0, 225), 5)
        cv2.imwrite('ROBOT.jpg', decimg)
except KeyboardInterrupt:
    sock.close()
    pass 
sock.close()


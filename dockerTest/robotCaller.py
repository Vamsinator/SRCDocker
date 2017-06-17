#!/usr/bin/env python
import numpy as np
import copy
import rospy
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
import Tkinter as tk

TCP_IP = '192.168.2.8'
TCP_PORT = 8009 #ENTER PORT STUFF HERE
sock = socket.socket()
sock.connect((TCP_IP, TCP_PORT))

armLabels = ["Rotates shoulder arm forward(+)/backward(-)", "Moves entire arm up(+)/down(-)", "rotates elbow/above forward(+)/backward(-)", "moves elbow joint forward(-)/backward(+)", "Rotates forearm forward(+)/backward(-)", "Moves hand up(+)/down(-)", "Moves had forward(-)/backward(+)"]
handLabels = ["Move thumb forward", "Curl/Uncurl thumb", "Curl/uncurl 2nd finger", "Curl/uncurl 3rd finger", "Curl/uncurl 4th finger"]
neckLabels = ["Bend down(only +)", "rotate left(+)/rotate right(-)", "Bend up(only -)"]
torsoLabels = ["Bend sideways, right(+)/left(-)", "Bend down/bend up", "rotate right(+)/rotate left(-)"]
pelvisLabel = "Down/Up"

top = tk.Tk()
LarmFrame = tk.Frame(top)
RarmFrame = tk.Frame(top)
LhandFrame = tk.Frame(top)
RhandFrame = tk.Frame(top)
NeckFrame = tk.Frame(top)
TorsoFrame = tk.Frame(top)
PelvisFrame = tk.Frame(top)
RmoveFrame = tk.Frame(top)
LmoveFrame = tk.Frame(top)

LarmFrame.pack(side=tk.LEFT)
RarmFrame.pack(side=tk.RIGHT)
LhandFrame.pack(side=tk.LEFT)
RhandFrame.pack(side=tk.RIGHT)
LmoveFrame.pack(side=tk.LEFT)
RmoveFrame.pack(side=tk.RIGHT)
NeckFrame.pack(side=tk.LEFT)
TorsoFrame.pack(side=tk.RIGHT)
PelvisFrame.pack(side=tk.RIGHT)

#Left, Right
DefaultArm = [[1.176, -0.932, -1.519, -0.007, -0.83, 0.0, 0.0, 0.0],[1.176, -0.932, 1.519, -0.007, 0.83, 0.0, 0.0, 0.0]]  

MoveVals = [[],[]]
MoveScales = [[],[]]
#Left, Right
MoveLimits = [[ [-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0] ] ,
[ [-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0]]]
MoveTime = [tk.DoubleVar(), tk.DoubleVar()]

ArmVals = [[],[]]
ArmScales = [[],[]]
#Left, Right
ArmLimits = [[ [-2.85, 2.0], [-1.519, 1.266], [-3.1, 2.18], [-2.174, 0.12], [-2.0, 3.1], [-0.6, 0.6], [-0.49, 0.36] ] ,
[ [-2.85, 2.0], [-1.266,1.519], [-3.1, 2.18], [-0.12,2.174], [-2.0, 3.1], [-0.6, 0.6], [-0.49, 0.36]]]
ArmTime = [tk.DoubleVar(), tk.DoubleVar()]

HandVals = [[],[]]
HandScales = [[],[]]
#left, right
HandLimits = [[ [0.0, 1.8], [0.0, -0.60], [0.0,-1.1], [0.0, -0.9], [0.0, -1.0] ] ,
[ [0.0, 1.8], [0.0, 0.6], [0.0, 1.1], [0.0, 0.9], [0.0, 1.0]]]

NeckVals = []
NeckScales = []
NeckLimits = [[0.0, 1.162], [1.0, -1.0], [-0.5, 0.0]]
NeckTime = tk.DoubleVar()

TorsoVals = []
TorsoScales = []
TorsoLimits = [[-20.00, 20.00], [0.00, 50.00], [90.00, -90.00]]
TorsoTime = tk.DoubleVar()

PelvisVal = tk.DoubleVar()
PelvisScales = tk.Scale()
PelvisLimits = [0.75, 1.20]
PelvisTime = 5.0 #HardCoded

def ResetVal(Vals, resetVal):
	for x in Vals:
		x.set(resetVal)

def LArmPubber():
	try:
		temp = "LEFTARM" + " " + str(ArmTime[0].get()) +" "+ str(ArmVals[0][0].get()) +" "+ str(ArmVals[0][1].get()) +" "+ str(ArmVals[0][2].get()) +" "+ str(ArmVals[0][3].get()) +" "+ str(ArmVals[0][4].get()) +" "+ str(ArmVals[0][5].get()) +" "+ str(ArmVals[0][6].get())
		print ''.join(temp)
		sock.send(''.join(temp))
	except socket.error:
		print "HELP"

def RArmPubber():
	try:
		temp = "RIGHTARM" + " " + str(ArmTime[1].get()) +" "+ str(ArmVals[1][0].get()) +" "+ str(ArmVals[1][1].get()) +" "+ str(ArmVals[1][2].get()) +" "+ str(ArmVals[1][3].get()) +" "+ str(ArmVals[1][4].get()) +" "+ str(ArmVals[1][5].get()) +" "+ str(ArmVals[1][6].get())
		print ''.join(temp)
		sock.send(''.join(temp))
	except socket.error:
		print "HELP"

def LHandPubber():
	try:
		temp = "LEFTHAND" + " " + str(HandVals[0][0].get()) + " " + str(HandVals[0][1].get()) + " " + str(HandVals[0][2].get()) + " " + str(HandVals[0][3].get()) + " " + str(HandVals[0][4].get())
		print ''.join(temp)
		sock.send(''.join(temp))
	except socket.error:
		print "HELP"

def RHandPubber():
	try:
		temp = "RIGHTHAND" + " " + str(HandVals[1][0].get()) + " " + str(HandVals[1][1].get()) + " " + str(HandVals[1][2].get()) + " " + str(HandVals[1][3].get()) + " " + str(HandVals[1][4].get())
		print ''.join(temp)
		sock.send(''.join(temp))
	except socket.error:
		print "HELP"

def NeckPubber():
	try:
		temp = "NECK" + " " + str(NeckTime.get()) + " " + str(NeckVals[0].get()) + " " + str(NeckVals[1].get()) + " " + str(NeckVals[2].get()) 
		print ''.join(temp)
		sock.send(''.join(temp))
	except socket.error:
		print "HELP"

def TorsoPubber():
	try:
		temp = "TORSO" + " " + str(TorsoTime.get()) + " " + str(TorsoVals[0].get()) + " " + str(TorsoVals[1].get()) + " " + str(TorsoVals[2].get())
		print ''.join(temp)
		sock.send(''.join(temp))
	except socket.error:
		print "HELP"

def PelvisPubber():
	try:
		temp = "PELVIS" + " " + str(PelvisTime) + " " + str(PelvisVal.get())
		print ''.join(temp)
		sock.send(''.join(temp))
	except socket.error:
		print "HELP"

def LMovePubber():
	try:
		temp = "LEFTMOVE" + " " + str(MoveTime[0].get()) +" "+ str(MoveVals[0][0].get()) +" "+ str(MoveVals[0][1].get()) +" "+ str(MoveVals[0][2].get()) +" "+ str(MoveVals[0][3].get()) +" "+ str(MoveVals[0][4].get()) +" "+ str(MoveVals[0][5].get())
		print ''.join(temp)
		sock.send(''.join(temp))
	except socket.error:
		print "HELP"

def RMovePubber():
	try:
		temp = "RIGHTMOVE" + " " + str(MoveTime[1].get()) +" "+ str(MoveVals[1][0].get()) +" "+ str(MoveVals[1][1].get()) +" "+ str(MoveVals[1][2].get()) +" "+ str(MoveVals[1][3].get()) +" "+ str(MoveVals[1][4].get()) +" "+ str(MoveVals[1][5].get())
		print ''.join(temp)
		sock.send(''.join(temp))
	except socket.error:
		print "HELP"

LeftacLabel = tk.Label (LarmFrame, text = "LEFT ARM")
LeftacLabel.pack()

RightacLabel = tk.Label (RarmFrame, text = "RIGHT ARM")
RightacLabel.pack()

RighthccLabel = tk.Label (RhandFrame, text = "RIGHT HAND")
RighthccLabel.pack()

LefthccLabel = tk.Label (LhandFrame, text = "LEFT HAND")
LefthccLabel.pack()

NeckncLabel = tk.Label (NeckFrame, text = "NECK")
NeckncLabel.pack()

TorsotcLabel = tk.Label (TorsoFrame, text = "TORSO")
TorsotcLabel.pack()

PelvispcLabel = tk.Label(PelvisFrame, text = "PELVIS")
PelvispcLabel.pack()

LMoveacLabel = tk.Label (LmoveFrame, text = "LEFT MOVE")
LMoveacLabel.pack()

RMoveacLabel = tk.Label (RmoveFrame, text = "RIGHT MOVE")
RMoveacLabel.pack()


RightacButton = tk.Button(RarmFrame, text="Send Right Arm Values", command = RArmPubber)
RESETRightacButton = tk.Button(RarmFrame, text="RESET RIGHT ARM", command = lambda: ResetVal(ArmVals[1], 0.0))
#DefaultRightButton = tk.Button(RarmFrame, text="Starting Right Arm", command = lambda: ResetVals(DefaultArm[1]))

LeftacButton = tk.Button(LarmFrame, text="Send Left Arm Values", command = LArmPubber)
RESETLeftacButton = tk.Button(LarmFrame, text="RESET LEFT ARM", command = lambda: ResetVal(ArmVals[0], 0.0))
DefaultLeftButton = tk.Button(RarmFrame, text="Default", command = lambda: ResetVals(DefaultArm[1]))

RighthccButton = tk.Button(RhandFrame, text="Send Right hand Values", command = RHandPubber)
RESETRighthccButton = tk.Button(RhandFrame, text="RESET RIGHT HAND", command = lambda: ResetVal(HandVals[1], 0.0))

LefthccButton = tk.Button(LhandFrame, text="Send Left hand Values", command = LHandPubber)
RESETLefthccButton = tk.Button(LhandFrame, text="RESET LEFT HAND", command = lambda: ResetVal(HandVals[0], 0.0))

NeckncButton = tk.Button(NeckFrame, text = "Send Neck Values", command = NeckPubber)
RESETNeckncButton = tk.Button(NeckFrame, text = "RESET NECK", command = lambda: ResetVal(NeckVals, 0.0))

TorsotcButton = tk.Button(TorsoFrame, text = "Send Torso Values", command = TorsoPubber)
RESETTorsotcButton = tk.Button(TorsoFrame, text = "RESET TORSO", command = lambda: ResetVal(TorsoVals, 0.0))

PelvispcButton = tk.Button(PelvisFrame, text = "Send Pelvis Values", command = PelvisPubber)
RESETPelvispcButton = tk.Button(PelvisFrame, text = "RESET PELVIS", command = lambda: PelvisVal.set(0.960))

RightMoveacButton = tk.Button(RmoveFrame, text="Send Right Move Values", command = RMovePubber)
RESETRightMoveacButton = tk.Button(RmoveFrame, text="RESET RIGHT MOVE", command = lambda: ResetVal(MoveVals[1], 0.0))

LeftMoveacButton = tk.Button(LmoveFrame, text="Send Left Move Values", command = LMovePubber)
RESETLeftMoveacButton = tk.Button(LmoveFrame, text="RESET LEFT MOVE", command = lambda: ResetVal(MoveVals[0], 0.0))
lengthSlider = 200


for j in range(2):
	for i in range(7):
		ArmVals[j].append(tk.DoubleVar())

		if j == 0:
			ArmScales[j].append(tk.Scale(LarmFrame, variable = ArmVals[j][i], label = armLabels[i], length = lengthSlider*(1.5), from_ = ArmLimits[j][i][0], to = ArmLimits[j][i][1], orient=tk.HORIZONTAL, resolution=0.001))
			ArmScales[j][i].pack()
		
		if j == 1:
			ArmScales[j].append(tk.Scale(RarmFrame, variable = ArmVals[j][i], label = armLabels[i], length = lengthSlider*(1.5), from_ = ArmLimits[j][i][0], to = ArmLimits[j][i][1], orient=tk.HORIZONTAL, resolution=0.001))
			ArmScales[j][i].pack()
		
for j in range(2):
	for i in range(5):
		HandVals[j].append(tk.DoubleVar())
		
		if j == 0:
			HandScales[j].append(tk.Scale(LhandFrame, variable = HandVals[j][i], label = handLabels[i], length = lengthSlider, from_ = HandLimits[j][i][0], to = HandLimits[j][i][1], orient=tk.HORIZONTAL, resolution=0.001))
			HandScales[j][i].pack()
		if j == 1:
			HandScales[j].append(tk.Scale(RhandFrame, variable = HandVals[j][i], label = handLabels[i], length = lengthSlider, from_ = HandLimits[j][i][0], to = HandLimits[j][i][1], orient=tk.HORIZONTAL, resolution=0.001))
			HandScales[j][i].pack()

for i in range(3):
		NeckVals.append(tk.DoubleVar())
		NeckScales.append(tk.Scale(NeckFrame, variable = NeckVals[i], label = neckLabels[i], length = lengthSlider, from_ = NeckLimits[i][0], to = NeckLimits[i][1], orient=tk.HORIZONTAL, resolution=0.001))
		NeckScales[i].pack()

for i in range(3):
		TorsoVals.append(tk.DoubleVar())
		TorsoScales.append(tk.Scale(TorsoFrame, variable = TorsoVals[i], label = torsoLabels[i], length = lengthSlider, from_ = TorsoLimits[i][0], to = TorsoLimits[i][1], orient=tk.HORIZONTAL, resolution=0.001))
		TorsoScales[i].pack()

#Already set in definition
PelvisScales = tk.Scale(PelvisFrame, variable = PelvisVal, label = pelvisLabel, from_ = PelvisLimits[0], length = lengthSlider, to = PelvisLimits[1], orient=tk.HORIZONTAL, resolution=0.001)
PelvisVal.set(0.96)
PelvisScales.pack()

for j in range(2):
	for i in range(6):
		MoveVals[j].append(tk.DoubleVar())

		if j == 0:
			MoveScales[j].append(tk.Scale(LmoveFrame, variable = MoveVals[j][i], label = i+1, length = lengthSlider, from_ = MoveLimits[j][i][0], to = MoveLimits[j][i][1], orient=tk.HORIZONTAL, resolution=0.001))
			MoveScales[j][i].pack()
		
		if j == 1:
			MoveScales[j].append(tk.Scale(RmoveFrame, variable = MoveVals[j][i], label = i+1, length = lengthSlider, from_ = MoveLimits[j][i][0], to = MoveLimits[j][i][1], orient=tk.HORIZONTAL, resolution=0.001))
			MoveScales[j][i].pack()

LATime = tk.Scale(LarmFrame, variable=ArmTime[0], label= "LeftArmTime", orient=tk.HORIZONTAL,to = 10.0, resolution = 0.001)
RATime = tk.Scale(RarmFrame, variable=ArmTime[1], label= "RightArmTime", orient=tk.HORIZONTAL, to = 10.0, resolution = 0.001)
LATime.pack()
RATime.pack()


TTime = tk.Scale(TorsoFrame, variable=TorsoTime, label= "TorsoTime", orient=tk.HORIZONTAL,to = 10.0, resolution = 0.001)
TTime.pack()

LMTime = tk.Scale(LmoveFrame, variable=MoveTime[0], label= "LeftMoveTime", orient=tk.HORIZONTAL,to = 10.0, resolution = 0.001)
RMTime = tk.Scale(RmoveFrame, variable=MoveTime[1], label= "RightMoveTime", orient=tk.HORIZONTAL, to = 10.0, resolution = 0.001)
LMTime.pack()
RMTime.pack()


RightacButton.pack()
LeftacButton.pack()
LefthccButton.pack()
RighthccButton.pack()
NeckncButton.pack()
TorsotcButton.pack()
PelvispcButton.pack()
LeftMoveacButton.pack()
RightMoveacButton.pack()
RESETLeftacButton.pack()
RESETRightacButton.pack()
RESETLefthccButton.pack()
RESETRighthccButton.pack()
RESETNeckncButton.pack()
RESETTorsotcButton.pack()
RESETPelvispcButton.pack()
RESETLeftMoveacButton.pack()
RESETRightMoveacButton.pack()

top.mainloop()
sock.close()

"""
try:
	while True:
	try:
		data = raw_input()
		if not data == "":
		sock.send(data)

	except socket.error:
		print "HELP"
		continue
except KeyboardInterrupt:
	pass
"""


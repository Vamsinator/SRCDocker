import time
import rospy
import tf
import tf2_ros
import numpy
import math
from collections import OrderedDict
from math import cos, radians, sin, sqrt
import select
import sys
import termios
import tty
from Task1 import ROBOT_NAME
from geometry_msgs.msg import Quaternion, Transform, Vector3


from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import copy
import sys
from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import tf2_ros
from visualization_msgs.msg import Marker
import rospy
import copy
from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
import time
oldAngle =0.0
#stepCounter = 0
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None
tfBuffer = None
tfListener = None
footStepListPublisher =None
neck_publisher = None

right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)
RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)
		     
footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, recievedFootStepStatus)
footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)
tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)
neck_publisher = rospy.Publisher("/ihmc_ros/{0}/control/neck_trajectory".format(ROBOT_NAME), NeckTrajectoryRosMessage, queue_size=1)
rate = rospy.Rate(10) # 10hz
time.sleep(1)

# make sure the simulation is running otherwise wait
		   
msg = FootstepDataListRosMessage()
msg.default_transfer_time = 0.8
msg.default_swing_time = 0.8
msg.execution_mode = 0
msg.unique_id = -1
msgCopy = copy.deepcopy(msg)
def addMarker( orientation):
	global pub
	mark = Marker()
	mark.header.frame_id = "torso"
        #self.mark.header.ns = 'newFrame'

	mark.header.stamp = rospy.Time.now()
	mark.type = Marker.ARROW
	mark.pose.position.x =1
	mark.pose.position.y = 1
	mark.pose.position.z = 1
	mark.pose.orientation.x=0
	mark.pose.orientation.y = 0
	mark.pose.orientation.z = orientation
	mark.pose.orientation.w = 1.0
	mark.scale.x = 5.0
	mark.scale.y = 5.0
	mark.scale.z = 5.0
	mark.color.a = 1.0
	mark.color.r = 1.0
	#self.mark.color.g = 1.0
	#self.mark.color.b = #1.0
	pub.publish(mark)    
def createRotationFootStepList( yaw):
        left_footstep = FootstepDataRosMessage()
        left_footstep.robot_side = FootstepDataRosMessage.LEFT
        right_footstep = FootstepDataRosMessage()
        right_footstep.robot_side = FootstepDataRosMessage.RIGHT

        left_foot_world = tfBuffer.lookup_transform(
            'world', LEFT_FOOT_FRAME_NAME, rospy.Time())
        right_foot_world = tfBuffer.lookup_transform(
            'world', RIGHT_FOOT_FRAME_NAME, rospy.Time())
        intermediate_transform = Transform()
        # create a virtual fram between the feet, this will be the center of the rotation
        intermediate_transform.translation.x = (
            left_foot_world.transform.translation.x + right_foot_world.transform.translation.x)/2.
        intermediate_transform.translation.y = (
            left_foot_world.transform.translation.y + right_foot_world.transform.translation.y)/2.
        intermediate_transform.translation.z = (
            left_foot_world.transform.translation.z + right_foot_world.transform.translation.z)/2.
        # here we assume that feet have the same orientation so we can pick arbitrary left or right
        intermediate_transform.rotation = left_foot_world.transform.rotation

        left_footstep.location = left_foot_world.transform.translation
        right_footstep.location = right_foot_world.transform.translation

        # define the turning radius
        radius = sqrt(
            (
                right_foot_world.transform.translation.x -
                left_foot_world.transform.translation.x
            )**2 + (
                right_foot_world.transform.translation.y -
                left_foot_world.transform.translation.y
            )**2) / 2.

        left_offset = [-radius*sin(yaw), radius*(1-cos(yaw)), 0]
        right_offset = [radius*sin(yaw), -radius*(1-cos(yaw)), 0]
        if yaw > 0:
            return [left_offset, right_offset]
        else:
            return [right_offset, left_offset]
def walkTest(move, direc, fir):
	global oldAngle, msg, msgCopy
	msg = copy.deepcopy(msgCopy)
	LEFT = 0
	RIGHT = 1
	START =2
	TURN = 3
	SHIFTLEFT = 4
	SHIFTRIGHT = 5
	SHIFTUP = 6
	RSTART = 7
	RTURN = 8
        SHIFTR = 9
        SHIFTL = 10
        ADJUSTR = 11
        ADJUSTL = 12
        REVERSE = 13
        if move == REVERSE:
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, [-.1, 0 , 0], 0))
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, [-.1, 0, 0], 0))
            footStepListPublisher.publish(msg)
            waitForFootsteps(2)
            return
        if move == ADJUSTR:
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, [0, 0, 0], math.pi/36))
            footStepListPublisher.publish(msg)
            waitForFootsteps(1)
        if move == ADJUSTL:
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, [0, 0, 0], -math.pi/36))
            footStepListPublisher.publish(msg)
            waitForFootsteps(1)
        if move == SHIFTR:
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, [.1, 0, 0], 0))
            footStepListPublisher.publish(msg)
            waitForFootsteps(1)
        if move == SHIFTL:
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, [.1, 0, 0], 0))
            footStepListPublisher.publish(msg)
            waitForFootsteps(1)
	if move == SHIFTUP:
	    msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, [.1, 0, 0], 0))
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, [.1, 0, 0], 0))
            footStepListPublisher.publish(msg)
            waitForFootsteps(2)
	    return
	if SHIFTRIGHT == move:
	    msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, [0, -0.1, 0], 0))
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, [0, -.1, 0], 0))
	    footStepListPublisher.publish(msg)
	    waitForFootsteps(2)
	    return
	if move == SHIFTLEFT:
	    msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, [0, 0.1, 0], 0))
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, [0, .1, 0], 0))
	    footStepListPublisher.publish(msg)
	    waitForFootsteps(2)
	    return
	direction = float(direc)
        if move== START:
	    msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, [.05*math.sin(oldAngle), .05*math.cos(oldAngle), 0.0], 0))
	    msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, [-.05*math.sin(oldAngle), -.05*math.cos(oldAngle), 0.0], 0))
	    footStepListPublisher.publish(msg)
	    waitForFootsteps(len(msg.footstep_data_list))
	    return
        if move == RSTART:
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, [-.05, 0, 0], 0))
            msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, [.05, 0, 0], 0))
            footStepListPublisher.publish(msg)
            waitForFootsteps(2)
            return
    #print "Hello"
        curYpos = 0
        LEFT_FOOT = [0.0, 0.00, 0.0]
        RIGHT_FOOT = [0.0, -0.00, 0.0]
        #direction = float(direc)
        curXpos = 0.0
        side = LEFT
        curDistTraveled = 0.0
	if move == TURN:
	    rospy.loginfo('Turning')
            if direction >= 0:
	        curXpos = abs(.316*(math.sin(direction) + math.sin(oldAngle)))
	        curYpos = .316*(math.cos(oldAngle)-math.cos(direction))
	    if direction< 0:
		curXpos =  abs(-.316*(math.sin(direction) + math.sin(oldAngle)))
		curYpos = .316*(-1.0*math.cos(oldAngle)+math.cos(direction))
	    #oldAngle = direction
            if direction == 0:
                curYpos = 0
            if direction >=0:
		#curYpos -= .05
    	        #retVal = self.createRotationFootStepList(direction)
		RIGHT_FOOT[0] = curXpos
		RIGHT_FOOT[1] = (curYpos) 
	        side = LEFT
                msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, LEFT_FOOT, direction/(math.pi/2.0)))
       
                msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, RIGHT_FOOT, direction/(math.pi/2.0)))
            if direction <0:
		#curYpos += .05
	        LEFT_FOOT[0] = curXpos
	        LEFT_FOOT[1] = (curYpos)
	        side = RIGHT
		retVal = createRotationFootStepList(direction)
	        msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, RIGHT_FOOT,direction/(math.pi/2.0)))
	        msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, LEFT_FOOT, direction/(math.pi/2.0)))
	    footStepListPublisher.publish(msg)
	    waitForFootsteps(len(msg.footstep_data_list))
	    return
	curDistTraveled = 0.0
	if move == RTURN:
	    rospy.loginfo('RTurning')
            if direction >= 0:
	        curXpos = abs(.316*(math.sin(direction) + math.sin(oldAngle)))
	        curYpos = .316*(math.cos(oldAngle)-math.cos(direction))
	    if direction< 0:
		curXpos =  abs(-.316*(math.sin(direction) + math.sin(oldAngle)))
		curYpos = .316*(-1.0*math.cos(oldAngle)+math.cos(direction))
	    #oldAngle = direction
            if direction == 0:
                curYpos = 0
            if direction <0:
		#curYpos -= .05
    	        #retVal = self.createRotationFootStepList(direction)
		LEFT_FOOT[0] = curXpos
		LEFT_FOOT[1] = (curYpos) 
	        side = LEFT
                msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, RIGHT_FOOT, direction/(math.pi/2.0)))
       
                msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, LEFT_FOOT, direction/(math.pi/2.0)))
            if direction >=0:
		#curYpos += .05
	        RIGHT_FOOT[0] = curXpos
	        RIGHT_FOOT[1] = (curYpos)
	        side = RIGHT
		retVal = createRotationFootStepList(direction)
	        msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, LEFT_FOOT,direction/(math.pi/2.0)))
	        msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, RIGHT_FOOT, direction/(math.pi/2.0)))
	    footStepListPublisher.publish(msg)
	    waitForFootsteps(len(msg.footstep_data_list))
	    return
   
    #generate list of steps based on magnitude and direction
    #first rotate opposite foot and then start walking
        side = int(move)
        first = int(fir)
        dist = float(direction)
        currentDist = 0
        while currentDist<dist:
            if side == LEFT:
	        if first == 1:
		    print "first left"
	            LEFT_FOOT[0] =.4
	            LEFT_FOOT[1] = 0.0
		    #print LEFT_FOOT
	            first = 0
                    currentDist += .4
	        if first == 0:
	            LEFT_FOOT[0] = 0.8
	            LEFT_FOOT[1] = 0.0	    
                    currentDist += 0.8
	        msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.LEFT, LEFT_FOOT, 0.0))
            if side == RIGHT:
	        if first == 1: 
		    print "first right"
                    RIGHT_FOOT[0] = .4
                    RIGHT_FOOT[1] = 0.0
		    #print RIGHT_FOOT
                    first = 0
                    currentDist += 0.4
                if first == 0:
                    RIGHT_FOOT[0] = .8
                    RIGHT_FOOT[1] = 0.0
                    currentDist += .8

                msg.footstep_data_list.append(createFootStepOffset(FootstepDataRosMessage.RIGHT, RIGHT_FOOT, 0.0))
            side ^=1
        footStepListPublisher.publish(msg)
        rospy.loginfo('walk forward...')
        waitForFootsteps(len(msg.footstep_data_list))
        return 

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace( stepSide):
	global LEFT_FOOT_FRAME_NAME, RIGHT_FOOT_FRAME_NAME, tfBuffer
        footstep = FootstepDataRosMessage()
        footstep.robot_side = stepSide

        if stepSide == FootstepDataRosMessage.LEFT:
            foot_frame = LEFT_FOOT_FRAME_NAME
        else:
            foot_frame = RIGHT_FOOT_FRAME_NAME

        footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
        footstep.orientation = footWorld.transform.rotation
        footstep.location = footWorld.transform.translation
        return footstep

# Creates footstep offset from the current foot position. The offset is in foot frame.
    
def createFootStepOffset(stepSide, offset, zrot):
        footstep = createFootStepInPlace(stepSide)
        
    # transform the offset to world frame
        quat = footstep.orientation
        rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        transformedOffset = numpy.dot(rot[0:3, 0:3], offset)
	addMarker(zrot)
        footstep.location.x += transformedOffset[0]
        footstep.location.y += transformedOffset[1]
        footstep.location.z += transformedOffset[2]

        footstep.orientation.z += zrot

        return footstep
def waitForFootsteps(numberOfSteps):
        global stepCounter
        stepCounter = 0
        while stepCounter < numberOfSteps:
            time.sleep(1)
        rospy.loginfo('finished set of steps')

def recievedFootStepStatus(msg):
        global stepCounter
        if msg.status == 1:
            stepCounter += 1
def appendTrajectoryPoint( neck_trajectory, time, positions):
        if not neck_trajectory.joint_trajectory_messages:
            neck_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
        for i, pos in enumerate(positions):
            point = TrajectoryPoint1DRosMessage()
            point.time = time
            point.position = pos
            point.velocity = 0
            neck_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
        return neck_trajectory

def adjustNeck(position, time):
	testmsg = NeckTrajectoryRosMessage()
	testmsg.unique_id = -1
	testmsg = appendTrajectoryPoint(testmsg, time, position)
        neck_publisher.publish(testmsg)


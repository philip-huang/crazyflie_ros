#!/usr/bin/env python

import rospy
import numpy as np
from crazyflie_driver.srv import *
from std_msgs.msg import Float32MultiArray
import tf
from crazyflie_driver.msg import TrajectoryPolynomialPiece, Position

def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])

class Crazyflie:
    def __init__(self, prefix, cf_frame):
        self.prefix = prefix
        self.cf_frame = cf_frame
        self.listener = tf.TransformListener()

        rospy.loginfo("Waiting for Services")
        rospy.wait_for_service(prefix + "/set_group_mask")
        self.setGroupMaskService = rospy.ServiceProxy(prefix + "/set_group_mask", SetGroupMask)
        rospy.wait_for_service(prefix + "/takeoff")
        self.takeoffService = rospy.ServiceProxy(prefix + "/takeoff", Takeoff)
        rospy.wait_for_service(prefix + "/land")
        self.landService = rospy.ServiceProxy(prefix + "/land", Land)
        rospy.wait_for_service(prefix + "/stop")
        self.stopService = rospy.ServiceProxy(prefix + "/stop", Stop)
        rospy.wait_for_service(prefix + "/go_to")
        self.goToService = rospy.ServiceProxy(prefix + "/go_to", GoTo)
        rospy.wait_for_service(prefix + "/upload_trajectory")
        self.uploadTrajectoryService = rospy.ServiceProxy(prefix + "/upload_trajectory", UploadTrajectory)
        rospy.wait_for_service(prefix + "/start_trajectory")
        self.startTrajectoryService = rospy.ServiceProxy(prefix + "/start_trajectory", StartTrajectory)
        rospy.wait_for_service(prefix + "/update_params")
        self.updateParamsService = rospy.ServiceProxy(prefix + "/update_params", UpdateParams)
        rospy.loginfo("Found All Required Services for " + self.prefix) 
        rospy.Subscriber((prefix + "/target_position"), Float32MultiArray, self.subTarget) 

        # Position Waypoint
        self.target = None

    def setGroupMask(self, groupMask):
        self.setGroupMaskService(groupMask)

    def takeoff(self, targetHeight, duration, groupMask = 0):
        self.takeoffService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration, groupMask = 0):
        self.landService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def stop(self, groupMask = 0):
        self.stopService(groupMask)

    def goTo(self, goal, yaw, duration, relative = False, groupMask = 0):
        gp = arrayToGeometryPoint(goal)
        self.goToService(groupMask, relative, gp, yaw, rospy.Duration.from_sec(duration))

    def uploadTrajectory(self, trajectoryId, pieceOffset, trajectory):
        pieces = []
        for poly in trajectory.polynomials:
            piece = TrajectoryPolynomialPiece()
            piece.duration = rospy.Duration.from_sec(poly.duration)
            piece.poly_x   = poly.px.p
            piece.poly_y   = poly.py.p
            piece.poly_z   = poly.pz.p
            piece.poly_yaw = poly.pyaw.p
            pieces.append(piece)
        self.uploadTrajectoryService(trajectoryId, pieceOffset, pieces)

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        self.startTrajectoryService(groupMask, trajectoryId, timescale, reverse, relative)

    def position(self):  
        try:           
            t = self.listener.getLatestCommonTime("/world", self.cf_frame) 
            trans, roat = self.listener.lookupTransform("/world", self.cf_frame, t) 
            return np.array(trans)
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("External Position Feedback of the Crazyflie is unavailable")

        return np.zeros(3)

    def getParam(self, name):
        return rospy.get_param(self.prefix + "/" + name)

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService([name])

    def setParams(self, params):
        for name, value in params.iteritems():
            rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService(params.keys())

    def reset_ekf(self):
        #self.setParam("kalman/initialX", 0)
        #self.setParam("kalman/initialY", 0.5)
        #self.setParam("kalman/initialZ", 0)
        self.setParam("kalman/resetEstimation", 1)
        rospy.sleep(0.3)         
        self.setParam("kalman/resetEstimation", 0) 
        rospy.sleep(2)  
     
    def subTarget(self, msg):
        self.num_target = len(msg.data)//3
        self.target = [[msg.data[3 * i + j] for j in range(3)] for i in range(self.num_target)]
    
    def ready_to_auto_land(self):
        diff = self.position()-self.target
        if np.abs(diff[0]) < 0.15 and np.abs(diff[1]) < 0.15 and diff[2] < 0.15:
            rospy.loginfo("Arrived Target! Signal rotor to shut down")
            return True
        else:
            return False

    def setTargets(self, targets):
        self.all_targets = targets
        self.next_target_number = 0

    def goTarget(self, next = False):
        if self.next_target_number >= len(self.all_targets):
            rospy.loginfo("No more targets!")
            return 0
        
        next_target = self.all_targets[self.next_target_number]
        
        if next_target == 'CamON':
            self.setParam("locSrv/useExtPos", 1)
            rospy.loginfo("Locoalization from Camera AUTO ON")
            self.next_target_number += 1
            next_target = self.all_targets[self.next_target_number]
        elif next_target == 'CamOFF':
            self.setParam("locSrv/useExtPos", 0)
            rospy.loginfo("Locoalization from Camera AUTO OFF")
            self.next_target_number += 1
            next_target = self.all_targets[self.next_target_number]


        if next_target[0] == 'Stop':
            rospy.loginfo("Signal rotor to shut down")
            self.stop()
            duration = next_target[1]
        
        else:
            duration = next_target[3]
            if self.all_targets[self.next_target_number-1][0] == 'Stop': 
                # need to re take off (go to service wont work)
                self.takeoff(next_target[2], duration)
            else:
                #   .....  Position  .......... Duration
                self.goTo(next_target[:3], 0, duration, relative = False)
            rospy.loginfo("Pursing Next Target at x:{}, y:{}, z:{}, duration:{}s".format(next_target[0], \
                            next_target[1], next_target[2], duration))
        self.next_target_number += 1

        return duration
    
#!/usr/bin/env python

import rospy
import numpy as np
from crazyflie_driver.srv import *
import tf
from crazyflie_driver.msg import TrajectoryPolynomialPiece 

def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])

class Crazyflie:
    def __init__(self, prefix, cf_frame):
        self.prefix = prefix
        self.cf_frame = cf_frame

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
        listener = tf.TransformListener()
        while not listener.frameExists(self.cf_frame):  
            continue
        t = listener.getLatestCommonTime("/world", self.cf_frame)
        trans, roat = listener.lookupTransform("/world", self.cf_frame, t) 
        return np.array(trans)

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
        self.setParam("kalman/resetEstimation", 1)
        rospy.sleep(0.3)
        self.setParam("kalman/resetEstimation", 0) 
        rospy.sleep(2) 
        rospy.set_param("kalman/initialX", 0)
        rospy.set_param("kalman/initialY", 0)
        rospy.set_param("kalman/initialZ", 0)
        self.updateParamsService(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

        rospy.set_param("kalman/resetEstimation", 1)
        # rospy.set_param("locSrv/extPosStdDev", 1e-4)
        self.updateParamsService(["kalman/resetEstimation"]) #, "locSrv/extPosStdDev"])
        rospy.sleep(1.0)
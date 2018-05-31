#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PointStamped
from crazyflie_driver.srv import Land, Stop, UpdateParams

if __name__ == "__main__":
    rospy.init_node("estimator")
    # Frames
    prefix = "crazyflie"
    rospy.loginfo("Estimator initiated for " + prefix)

    cf_frame = prefix + "/base_link" 
    src_frame = "/world"
    
    # External Position Publisher
    pub = rospy.Publisher("/" +prefix + "/external_position", PointStamped, queue_size=1)
    msg = PointStamped()
    flight = rospy.get_param("~flight", False)
    # Land Service  
    if flight:
        rospy.wait_for_service("/" + prefix + "/update_params")
        rospy.loginfo("Found update params services")
        update_params = rospy.ServiceProxy("/" + prefix + '/update_params', UpdateParams)
        #rospy.wait_for_service(prefix + "/land")
        land = rospy.ServiceProxy("/" + prefix + "/land", Land)
        #rospy.wait_for_service(prefix + "/stop")
        stop = rospy.ServiceProxy("/" + prefix + "/stop", Stop)
    
    # Transform Listener
    listener = tf.TransformListener()

    # Update Time
    t_now = rospy.get_rostime()
    last_update_time = t_now.secs + t_now.nsecs * (10 ** -9) 
    firstTransform = True 
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():      
        if listener.frameExists(cf_frame):  
            if firstTransform and flight: 
                #rospy.set_param("/" + prefix + "kalman/initialX", 0)
                #rospy.set_param("/" + prefix + "kalman/initialY", 0)
                #rospy.set_param("/" + prefix + "kalman/initialZ", 0.02)
                #update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

                #rospy.set_param("kalman/resetEstimation", 1)
                #rospy.set_param("locSrv/extPosStdDev", 1e-4)
                #update_params(["kalman/resetEstimation"]) #, "locSrv/extPosStdDev"])
                pass
                
            t = listener.getLatestCommonTime(src_frame, cf_frame)
            trans, roat = listener.lookupTransform(src_frame, cf_frame, t) 
            msg.header.frame_id = src_frame
            msg.header.stamp = rospy.Time.now()
            msg.header.seq += 1
            msg.point.x = trans[0]
            msg.point.y = trans[1]
            msg.point.z = trans[2] 
            # pub.publish(msg)
            now_time = msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** -9) 
            
            if (not firstTransform) and (now_time - last_update_time) > 1:
                rospy.logerr("Missing external position for one second. Requesting landing..")
               
                land(0, targetHeight = 0.0, duration = 1.0)
                rospy.sleep(1.0)
                stop(0)
                raise rospy.exceptions.ROSInterruptException()
            
            firstTransform = False
            last_update_time = now_time
        rate.sleep()
        
    rospy.spin()
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
    pub = rospy.Publisher("/" +prefix + "/external_position_", PointStamped, queue_size=1)
    msg = PointStamped()
    flight = rospy.get_param("~flight", False)
    # Land Service 
    if flight:
        rospy.wait_for_service("/" + prefix + "/update_params")
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
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
            t = listener.getLatestCommonTime(src_frame, cf_frame)
            trans, roat = listener.lookupTransform(src_frame, cf_frame, t) 
            msg.header.frame_id = src_frame
            msg.header.stamp = rospy.Time.now()
            msg.header.seq += 1
            msg.point.x = trans[0]
            msg.point.y = trans[1]
            msg.point.z = trans[2] 
            pub.publish(msg) 
            now_time = msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** -9) 
            if firstTransform: 
                firstTransform = False 
            elif (now_time - last_update_time) > 1:
                rospy.logerr("Missing external position for one second. Requesting landing..")
               
                land(0, targetHeight = 0.0, duration = 1.0)
                rospy.sleep(1.0)
                stop(0)
                raise rospy.exceptions.ROSInterruptException()
            last_update_time = now_time
        rate.sleep()
        
    rospy.spin()
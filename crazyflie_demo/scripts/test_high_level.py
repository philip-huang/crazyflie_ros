#!/usr/bin/env python

import rospy
import crazyflie
import estimator
import time
import uav_trajectory


if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf = crazyflie.Crazyflie("/crazyflie", "/crazyflie/base_link")  
    cf.setParam("commander/enHighLevel", 1)
    cf.reset_ekf() 
  
    # cf.goTo([0.5, 0, height], 0, duration = 1.0)
    # time.sleep(5.0)

    # cf.goTo([0.5, 0.5, height], 0, duration = 1.0)
    # time.sleep(5.0)

    # cf.goTo([0, 0.5, height], 0, duration = 1.0)
    # time.sleep(5.0)

    # cf.goTo([0, 0, height], 0, duration = 1.0)
    # time.sleep(5.0)

    #cf.land(targetHeight = 0.0, duration = 2.0)
    # time.sleep(2.0)

    # cf.land(targetHeight = 0.0, duration = 2.0)

    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("takeoff.csv")

    traj2 = uav_trajectory.Trajectory()
    traj2.loadcsv("figure8.csv")
 
    cf.uploadTrajectory(0, 0, traj1)
    cf.startTrajectory(0, timescale=1.0)
    time.sleep(traj1.duration * 2.0)

    t_upload_start = time.time()
    cf.uploadTrajectory(1, len(traj1.polynomials), traj2)
    print("upload time taken: {}s".format(time.time()-t_upload_start))
    cf.startTrajectory(1, timescale=2.0)
    time.sleep(traj2.duration * 1.0)

    cf.startTrajectory(0, timescale=1.0, reverse=True)
    time.sleep(traj1.duration * 1.0)

    cf.stop()

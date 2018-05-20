#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

print("Hello World!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf = crazyflie.Crazyflie("/crazyflie", "/vicon/crazyflie/crazyflie") 
    cf.setParam("commander/enHighLevel", 1)
    cf.reset_ekf() 

    cf.takeoff(targetHeight = 0.5, duration = 3.0)
    time.sleep(5.0)

    cf.land(targetHeight = 0.0, duration = 3.0)
    time.sleep(3.0)

    cf.land(targetHeight = 0.0, duration = 2.0)

    # traj1 = uav_trajectory.Trajectory()
    # traj1.loadcsv("takeoff.csv")

    # traj2 = uav_trajectory.Trajectory()
    # traj2.loadcsv("figure8.csv")

    # print(traj1.duration)

    # cf.uploadTrajectory(0, 0, traj1)
    # cf.uploadTrajectory(1, len(traj1.polynomials), traj2)

    # cf.startTrajectory(0, timescale=1.0)
    # time.sleep(traj1.duration * 2.0)

    # cf.startTrajectory(1, timescale=2.0)
    # time.sleep(traj2.duration * 2.0)

    # cf.startTrajectory(0, timescale=1.0, reverse=True)
    # time.sleep(traj1.duration * 1.0)

    cf.stop()

#!/usr/bin/env python

import rospy
import crazyflie
import time
from sensor_msgs.msg import Joy

last_joy_msg = None
current_action_duration = 0.0
action_start_time = 0.0
drone_position = [0, 0, 0]    

def get_time():
    t = rospy.get_rostime()
    return t.secs + t.nsecs * (10 ** -9)

def joy_cb(msg):
    global last_joy_msg
    last_joy_msg = msg

def load_params():
    config = {}
    config["target_height"] = rospy.get_param("~target_height", 0.2)
    config["takeoff_speed"] = rospy.get_param("~takeoff_speed", 1)
    config["landing_speed"] = rospy.get_param("~landing_speed", 0.5)
    config["Goto_speed"] = rospy.get_param("~goto_speed", 1)
    return config

class DroneState:
    # Like enum in c ++ 
    Idle = 0
    Taking_Off = 1
    Hover = 2
    Landing = 3
    Goto = 4
    Emergency = 5

    @staticmethod
    def default_nextstate(state):
        if state == DroneState.Idle:
            return DroneState.Idle
        if state == DroneState.Taking_Off:
            if (action_start_time + current_action_duration) > get_time(): 
                return DroneState.Taking_Off
            else: 
                return DroneState.Hover
        if state == DroneState.Landing:
            if (action_start_time + current_action_duration) > get_time():
                return DroneState.Landing
            else: 
                return DroneState.Idle
        if state == DroneState.Goto:
            if (action_start_time + current_action_duration) > get_time():
                return DroneState.Goto
            else: 
                return DroneState.Hover
        if state == DroneState.Emergency:
            return DroneState.Emergency
        if state == DroneState.Hover:
            return DroneState.Hover
        
    @staticmethod
    def joy_nextstate(state, msg):
        global drone_position
        if msg.buttons[2] == 1:
            rospy.logfatal("Emergency Requested! Prepare to land....")
            return DroneState.Emergency
        elif msg.buttons[3] == 1 and state == DroneState.Goto:
            rospy.loginfo("Hover Requested")
            return DroneState.Hover
        elif msg.buttons[0] == 1 and state == DroneState.Idle:
            rospy.loginfo("Taking Off Requested")
            return DroneState.Taking_Off
        elif msg.buttons[1] == 1 and (state == DroneState.Taking_Off or state == DroneState.Hover \
        or state == DroneState.Landing):
            rospy.loginfo("Landing Requested")
            return DroneState.Landing 
        elif (msg.axes[6] == -1.0 or msg.axes[6] == 1.0) and state == DroneState.Hover:
            rospy.loginfo("Go {} m in x direction".format(msg.axes[6] * 0.1))
            drone_position = [msg.axes[6] * 0.1, 0, 0]
            return DroneState.Goto
        elif (msg.axes[7] == -1.0 or msg.axes[7] == 1.0) and state == DroneState.Hover:
            rospy.loginfo("Go {} m in y direction".format(msg.axes[7] * 0.1))
            drone_position = [0, msg.axes[7] * 0.1, 0]
            return DroneState.Goto
        else:
            # Void Joystick command
            return -1

if __name__ == '__main__':
    rospy.init_node("joystick_controller")
    rospy.loginfo("Initializing Joystick Controller...")
    config = load_params()
    rospy.Subscriber("/joy", Joy, joy_cb)

    # Connecting to Crazyflie
    cf = crazyflie.Crazyflie("/crazyflie", "/crazyflie/base_link")  
    cf.setParam("commander/enHighLevel", 1)
    cf.reset_ekf() 
    rospy.loginfo("Finish Setting up Crazyflie. Ready to take off...")
    state = DroneState.Idle
    

    while(not rospy.is_shutdown()):
        if last_joy_msg != None:
            new_state = DroneState.joy_nextstate(state, last_joy_msg) 
            if new_state == DroneState.Taking_Off:
                current_action_duration = config["target_height"] * config["takeoff_speed"]
                cf.takeoff(targetHeight = config["target_height"], 
                            duration = current_action_duration)

            elif new_state == DroneState.Landing:
                current_action_duration = config["landing_speed"] * config["target_height"]
                cf.land(targetHeight = 0.0, duration = current_action_duration) 
            elif new_state == DroneState.Emergency:
                cf.land(targetHeight = 0.0, duration = 0.5)
                rospy.sleep(0.5)
                cf.stop()
            elif new_state == DroneState.Hover:
                cf.goTo([0, 0, 0], 0, 0.1, relative = True)
                rospy.sleep(0.1)
            elif new_state == DroneState.Goto:
                current_action_duration = 0.2
                cf.goTo(drone_position, 0, current_action_duration, relative = True) 
                

            last_joy_msg = None

            if new_state != -1:
                state = new_state
            action_start_time = get_time()
 
        else:
            state = DroneState.default_nextstate(state) 
    
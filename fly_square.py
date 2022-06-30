#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy
import math, time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL

from rich.console import Console
console = Console()

# Global parameters
x_cor = 10
y_cor = 0
yaw_in_radians = 0
yaw_rate = 0
flag = 0

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def _init_(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoff_altitude = 3
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
            takeoffService(altitude = takeoff_altitude)
            console.print("[EXECUTED] Takeoff {} m".format(takeoff_altitude), style="green bold")
        except rospy.ServiceException as e:
            console.print("[FAILED] Takeoff Aborted", style="red bold")
            console.print("ERROR : {}".format(e), style="red")

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
            console.print("[EXECUTED] ARMED", style="green bold")
        except rospy.ServiceException as e:
            console.print("[FAILED] Arming Aborted", style="red bold")
            console.print("ERROR : {}".format(e), style="red")

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(False)
            console.print("[EXECUTED] DISARMED", style="green bold")
        except rospy.ServiceException as e:
            console.print("[FAILED] Disarming Aborted", style="red bold")
            console.print("ERROR : {}".format(e), style="red")

    def setguidedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='GUIDED')
            console.print("[EXECUTED] MODE GUIDED", style="green bold")
        except rospy.ServiceException as e:
            console.print("[FAILED] Guided Mode Could not be set", style="red bold")
            console.print("ERROR : {}".format(e), style="red")


    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='LAND')
            console.print("[EXECUTED] MODE LAND", style="green bold")
        except rospy.ServiceException as e:
               console.print("[FAILED] Land Mode Could not be set", style="red bold")
               console.print("ERROR : {}".format(e), style="red")

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask =  PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                             + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                             + PositionTarget.IGNORE_YAW_RATE
        
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 1.0
        
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP

        # A Message for the current local position of the drone
        self.local_pos = Point()

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.
        self.theta = np.linspace(0, 2*np.pi, 5)

        # the radius of the circle
        self.r = 10

        # compute x1 and x2
        self.x1 = self.r*np.cos(self.theta)
        self.x2 = self.r*np.sin(self.theta)
        self.sp.position.x = 0
        self.sp.position.y = 0
        self.abc = 0

    ## Update setpoint message
    def updateSp(self):
        global x_cor,y_cor,yaw_rate,flag
        try:
            d = 0.5
            if -d <= self.threshold(self.local_pos.x,self.sp.position.x) <= d and -d <= self.threshold(self.local_pos.y,self.sp.position.y) <= d:
                self.sp.position.x = self.x1[self.abc]
                self.sp.position.y = self.x2[self.abc]
                print(self.abc)
                self.abc += 1

            # Second Coordinate of Square
            #if self.abc == 4 and -d <= self.threshold(self.local_pos.x,self.sp.position.x) <= d and -d <= self.threshold(self.local_pos.y,self.sp.position.y) <= d:
             #   modes.setAutoLandMode()
            
        except:
            pass
    
    """ CUSTOM FUNCTION """
    
    def threshold(self,x,y):
        return abs(x-y)
    
    """ DEFINED, but UNUSED """
    
    def x_dir(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y

    def neg_x_dir(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y

    def y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y + 5

    def neg_y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y - 5

    """ CALLBACK """

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg
    
n1 = 0
n2 = 0
n3 = 0

# Main function
def main():
    global n1, n2
    # initiate node
    rospy.init_node('square', anonymous=True)
    # ROS loop rate
    rate = rospy.Rate(20.0)
    
    """ OBJECTS """
    # flight mode object
    modes = fcuModes()
    # controller object
    cnt = Controller()

    """ RECEIVE & TRANSMIT DATA """
    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    """ TAKEOFF """
    # Make sure the drone is armed
    modes.setguidedMode()

    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    time.sleep(5)

    # set in takeoff mode and takeoff to default altitude (3 m)
    if cnt.state.armed:
        modes.setTakeoff()
        time.sleep(9)
    # We need to send few setpoint messages, then activate guided mode, to take effec
    # activate guided mode
    
    """ SQUARE MOVEMENT """
    # ROS main loop
    while not rospy.is_shutdown():
        cnt.updateSp()
        sp_pub.publish(cnt.sp)
        rate.sleep()
        

def run():
    try:
        main()
    except rospy.ROSInterruptException:
        pass


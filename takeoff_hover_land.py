#!/usr/bin/env python

from rich.console import Console
console = Console()

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import time

def run():
    rospy.init_node('drone_controller', anonymous=True)
    rate = rospy.Rate(10)
    
    # Set Mode
    console.print("[INFO] Setting the Mode...", style="blue bold")
    rospy.wait_for_service('/mavros/set_mode')
    try:
        change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = change_mode(custom_mode="GUIDED")
        rospy.loginfo(response)
        console.print("[EXECUTED] SET MODE to 'GUIDED'", style="green bold")
    except rospy.ServiceException as e:
        console.print("[FAILED] SET MODE to 'GUIDED'", style="red bold")
        console.print("ERROR : {}".format(e), style="red")
    
    # Arm
    console.print("[INFO] Starting to ARMING...", style="blue bold")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arming_cl(value = True)
        rospy.loginfo(response)
        console.print("[EXECUTED] ARMING", style="green bold")
    except rospy.ServiceException as e:
        console.print("[FAILED] ARMING", style="red bold")
        console.print("ERROR : {}".format(e), style="red")
    
    # Takeoff
    console.print("[INFO] Starting to TAKEOFF...", style="blue bold")
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        response = takeoff_cl(altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo(response)
        console.print("[EXECUTED] TAKEOFF", style="green bold")
    except rospy.ServiceException as e:
        console.print("[FAILED] TAKEOFF'", style="red bold")
        console.print("ERROR : {}".format(e), style="red")
    
    console.print("[INFO] HOVERING...", style="blue bold")
    time.sleep(10)
    
    # Land
    console.print("[INFO] Starting to LANDING...", style="blue bold")
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        response = takeoff_cl(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo(response)
        console.print("[EXECUTED] LANDING", style="green bold")
    except rospy.ServiceException as e:
        console.print("[FAILED] LANDING'", style="red bold")
        console.print("ERROR : {}".format(e), style="red")
    
    console.print("[INFO] Wait to Land...", style="blue bold")
    time.sleep(10)
    
    # Disarm
    console.print("[INFO] Starting to DISARMING...", style="blue bold")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arming_cl(value = False)
        rospy.loginfo(response)
        console.print("[EXECUTED] DISARM", style="green bold")
    except rospy.ServiceException as e:
        console.print("[FAILED] DISARM'", style="red bold")
        console.print("ERROR : {}".format(e), style="red")

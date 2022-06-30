#!/usr/bin/env python

from rich.console import Console
console = Console()

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import time

rospy.init_node('cps_landing')
rate = rospy.Rate(10)

def run():
    # Land
    print("\nLanding")
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        response = takeoff_cl(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo(response)
        console.print("[EXECUTED] LANDING", style="green bold")
    except rospy.ServiceException as e:
        console.print("[FAILED] LANDING", style="red bold")
        console.print("ERROR : {}".format(e), style="red")
    
    print("\nWait to Land")
    time.sleep(10)
    
    # Disarm
    print ("\nDisarming")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arming_cl(value = False)
        rospy.loginfo(response)
        console.print("[EXECUTED] DISARMED", style="green bold")
    except rospy.ServiceException as e:
        console.print("[FAILED] DISARMING", style="red bold")
        console.print("ERROR : {}".format(e), style="red")

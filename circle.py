#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *


def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)
    # Create an object for the API.
    drone = gnc_api()
    
    # Wait for FCU connection.
    drone.wait4connect()

    # Set GUIDED mode
    drone.set_mode("GUIDED")
    # Wait for the mode to be switched.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    # Specify some waypoints
    goals = [[0, 0, 3, 0]]
    
    # Calculation for circle
    width, height = 11, 11
    a, b = 5, 5
    r = 5
    EPSILON = 2.2
    
    # draw the circle
    for y in range(height):
        for x in range(width):
            # see if we're close to (x-a)**2 + (y-b)**2 == r**2
            if abs((x-a)**2 + (y-b)**2 - r**2) < EPSILON**2:
                goals.append([x, y, 3, 0])
    
    
    # Last coordinate, make all default 
    goals.append([0, 0, 3, 0])
    
    i = 0
    while i < len(goals):
        drone.set_destination(x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


def run():
    try:
        main()
    except KeyboardInterrupt:
        exit()

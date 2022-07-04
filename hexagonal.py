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
    # Wait for the mode to be switched.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()
    # Set GUIDED mode
    drone.set_mode("GUIDED")
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    # Specify some waypoints
    goals = [[0, 0, 3, 0], [5, 0, 3, -90], [2, 3, 3, 45],
             [-2, 3, 3, 90], [-5, 0, 3, 135], [-2, -3, 3, -135], 
             [2, -3, 3, -90], [5, 0, 3, -45], [0, 0, 3, 90], [0, 0, 3, 0]]
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
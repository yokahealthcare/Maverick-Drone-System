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
    
    # Asking for altitude value
    altitude = float(input("Set The Altitude : "))

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
    
    # Takeoff
    drone.takeoff(altitude)
    
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
# Maverick Drone System

**CPS DRONE Sampoerna University Team 2022**
1. Erwin Yonata
2. Edrick Hansel
3. Pieter Timothy

## Setup in IoT lab PC
***Run these commands in home directory***
1. run ```gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world``` this opens an arducopter simulator
2. run ```./startsitl.sh``` or ```cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console``` to start software in the loop
3. run ```roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@```
4. open QGroundControl by double-clicking app image or run ```./QGroundControl.AppImage```

## Code Execution
***Code in this repo needs python package rich and pyfiglet***
1. ```cd Maverick-Drone-System && python3 run.py```

#Useful Links
- [Tutorial playlist by Intelligent Quads](https://www.youtube.com/watch?v=AP1UC0DlIrE&list=PLy9nLDKxDN683GqAiJ4IVLquYBod_2oA6)
- [iq_gnc library documentation](https://github.com/Intelligent-Quads/iq_gnc/blob/master/docs/py_gnc_functions.md)
- [ROS Documentation](http://wiki.ros.org/Documentation)
- [Mavros Documentation](http://wiki.ros.org/mavros)

# Endnote
This program runs on python 3. Codes are written using iq_gnc.py_gnc_functions library. 


# ***CPS DRONE TEAM 2022*** 

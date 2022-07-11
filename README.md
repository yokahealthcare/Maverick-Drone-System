# Maverick Drone System

CPS DRONE Sampoerna University Team
1. Erwin Yonata
2. Edrick Hansel
3. Pieter Timothy

## Setup in IoT lab PC
1. run ```gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world``` this opens an arducopter simulator
2. run ```./startsitl.sh``` starts software in the loop
3. run ```roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@```
4. open ./QGroundControl.AppImage

## Run Python Code
1. ```cd maverick/```
2. ```python3 run.py```

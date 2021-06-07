# gazebo_ros_drone_parts
The ROS nodes, models and worlds to add sensors to the iris drone model from [ardupilot_gazebo](https://github.com/khancyr/ardupilot_gazebo).

## Included sensors:
- _Optical Flow sensor_ already implemented in [ardupilot_gazebo](https://github.com/khancyr/ardupilot_gazebo) package 
  (No ROS support!! --> NO data published)
- _8 Distance sensors_ using hokuyo sensor with 8 rays in every direction (Type: sensor_msgs/LaserScan)

## Run simulation with ROS support:

TERMINAL01: 
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
```

TERMINAL02: 
```
roslaunch gazebo_ros_drone_parts distance_sensor.launch
```

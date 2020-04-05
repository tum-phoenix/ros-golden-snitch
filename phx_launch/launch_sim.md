# The commands to run the simulator

Note that these only works if you have installed gazebo as described at the [ardupilot wiki](https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html)

1. window(Gazebo):

    gazebo --verbose worlds/iris_arducopter_runway.world
    
2. window(SITL):

    cd ~/ardupilot/ArduCopter
    ../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
   
 3. window(Mavros):
 
    roslaunch phx_launch phx_sim_mavros.launch
   
 4. window (Whatever code you want to run on the pi, e.g.)
 
    rosrun phx_launch simpleFlight.py
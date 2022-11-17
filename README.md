Muti Agent Drone System using RotorS simulator
===============

RotorS is a MAV gazebo simulator.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

Installation Instructions - Ubuntu 20.04 with ROS noetic
---------------------------------------------------------

 1. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 $ wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
 $ wstool merge rotors_hil.rosinstall
 $ wstool update
 $ git clone git@github.com:orimana2020/Multi_Agent_Drone_ROS_Simulation.git
 $ cd ~/catkin_ws
 $ catkin build
 $ source devel/setup.bash
 ```


# Map My World

![MapMyWorld](https://user-images.githubusercontent.com/18179768/156851419-8591042f-0b27-4421-8f99-d7f2d3a725f4.jpeg)

In this project you will create a 2D occupancy grid and 3D octomap from a simulated environment using your own robot with the RTAB-Map package.

RTAB-Map (Real-Time Appearance-Based Mapping) is a popular solution for SLAM to develop robots that can map environments in 3D. RTAB-Map has good speed and memory management, and it provides custom developed tools for information analysis. Most importantly, the quality of the documentation on ROS Wiki (http://wiki.ros.org/rtabmap_ros) is very high. Being able to leverage RTAB-Map with your own robots will lead to a solid foundation for mapping and localization well beyond this Nanodegree program.

For this project we will be using the rtabmap_ros package, which is a ROS wrapper (API) for interacting with RTAB-Map. Keep this in mind when looking at the relative documentation.

## DB Link: 
https://drive.google.com/file/d/1PV9AbRfmO7eishBhtfHljB84FGv_XHVN/view?usp=sharing

## Structure
```
    .Map_My_World             # Map My World
├── my_robot                               # my_robot package        
│   │   ├── config                         # config folder for configuration files   
│   │   │   ├── base_local_planner_params.yaml
│   │   │   ├── costmap_common_params.yaml
│   │   │   ├── global_costmap_params.yaml
│   │   │   ├── local_costmap_params.yaml
│   │   ├── launch                         # launch folder for launch files   
│   │   │   ├── amcl.launch
│   │   │   ├── robot_description.launch
│	│	│	├── mapping.launch
│   │   │   ├── world.launch
│   │   │   ├── teleop.launch   
│   │   ├── maps                           	   # maps folder for maps
│   │   │   ├── map.pgm
│   │   │   ├── map.yaml
│   │   ├── meshes                         # meshes folder for sensors
│   │   │   ├── hokuyo.dae
│   │   ├── models                           # models folder for models files
│   │   │   ├── white_ball
│   │   │   │   ├── model.config
│   │   │   │   ├── model.sdf
│   │   │   ├── project2_building
│   │   │   │   ├── model.config
│   │   │   │   ├── model.sdf	      
│   │   ├── urdf                           # urdf folder for xarco files
│   │   │   ├── my_robot.gazebo
│   │   │   ├── my_robot.xacro
│   │   ├── worlds                         # world folder for world files
│   │   │   ├── empty.world
│   │   │   ├── myworld.world
│   │   ├── CMakeLists.txt                 # compiler instructions
│   │   ├── package.xml                    # package info
│   ├── teleop_twist_keyboard              # teleop_twist_keyboard
│   │   ├── CHANGELOG.rst                  # change log
│   │   ├── CMakeLists.txt                 # compiler instructions
│   │   ├── README.md                      # README for documentation
│   │   ├── package.xml                    # package info
│   │   ├── teleop_twist_keyboard.py       # keyboard controller
```

## Prerequisites
- ROS (Kinetic), Gazebo on Linux
- CMake & g++/gcc
## Install rtabmap-ros package
```
$ sudo apt-get install ros-${ROS_DISTRO}-rtabmap-ros
```
## Build and Launch
Clone project and initialize a catkin workspace
```
$ mkdir catkin_ws && cd catkin_ws
$ git clone (Package GIT)
$ mv (Package) src
$ cd src && catkin_init_workspace
```
Within the catkin_ws/src folder, clone the teleop project
```
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard
```
Move back to catkin_ws\ and build
```
$ cd ..
$ catkin_make
```
Launch the world and robot
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

Open another terminal and launch the mapping.launch file.
```
$ source devel/setup.bash
$ roslaunch my_robot mapping.launch
```

Open another terminal for teleop node.

```
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

The rtabmap-ros package will save the resulted map with the localized trajectory of the robot in a database file ~/.ros/rtabmap.db.

## to get the Map

Open another terminal to open database file using rtabmap-databaseViewer
```
$ rtabmap-databaseViewer ~/.ros/rtabmap.db
```

Choose View -> Constraints View and Graph View
To see 3D Map, Choose Edit -> View 3D Map ...

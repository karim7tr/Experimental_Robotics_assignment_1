# ROS Package: assignment1_exp_rob_lab

## Introduction

Welcome to the `assignment1_exp_rob_lab` ROS package! This package is designed for the Experimental Robotics Lab assignment 1. It involves simulating a robot in a virtual world, utilizing ArUco markers for navigation.

## Package Overview

### Folder Structure:

- **src:** Contains Python scripts (`ArUcoMarkerExtractor.py` and `MarkerBasedNavigation.py`) for ArUco marker extraction and robot navigation.
- **launch:** Contains ROS launch files (`MarkerBasedNavigation.launch` and `MarkerBasedNavigation_Full.launch`) for launching Gazebo, RViz, and the robot nodes.
- **config:** Holds RViz configuration file (`rvizconfig.rviz`).
- **worlds:** Contains Gazebo world file (`aruco_assignment.world`).
- **CMakeLists.txt and package.xml:** Define package dependencies, build configuration, and other metadata.
- **setup.py:** Specifies Python scripts to be installed.

### Dependencies:

- rospy
- sensor_msgs
- cv_bridge
- geometry_msgs
- std_msgs
- gazebo_msgs

## Getting Started

Follow these steps to run the ROS package and simulate the robot in a virtual environment:

### Step 1: Build the ROS Package

```bash
catkin_make
```

### Step 2: Source the Workspace

```bash
source devel/setup.bash
```

### Step 3: Launch Gazebo and Navigation Nodes

```bash
roslaunch assignment1_exp_rob_lab MarkerBasedNavigation.launch
```

### Step 4:  Launch RViz for Visualization

```bash
rosrun rviz rviz -d /path/to/assignment1_exp_rob_lab/config/rvizconfig.rviz
```
### Step 5: Observe the Simulation

Gazebo should simulate the robot's environment, and RViz should visualize camera images, ArUco markers, and the robot model.

### Step 6:  Run ArUco Marker Extraction

If you want to observe ArUco marker extraction, uncomment the corresponding lines in `MarkerBasedNavigation.launch`. Save the file and relaunch the package.

### Step 7: Observe Robot Movement

As the simulation runs, the robot should move based on detected ArUco markers.

## Additional Resources

### Flowchart(s)

![Alt Text](media/flowchart.jpg)


### Videos

1. [Robot Simulation in Virtual World](link_to_video1)
2. [Robot Operation in Real World](link_to_video2)


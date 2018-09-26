# Flock

A ROS driver for [DJI Tello](https://store.dji.com/product/tello) drones.

## Installation

### 1. Set up your Linux environment

Set up a Ubuntu 16.04 box or VM. This should include ffmpeg 2.8.15-0.
~~~
ffmpeg -version
~~~

### 2. Set up your Python environment

Use your favorite Python package manager to set up Python 2.7 and the following packages:

* numpy 1.11.0
* av 0.4.1 (note: newer versions may require a newer ffmpeg library)
* opencv-python 3.4.3.18
* opencv-contrib-python 3.4.3.18
* image 1.5.25
* tellopy 0.5.0 -- this is the stable version available via PyPI / pip install

### 3. Install ROS

[Install ROS Kinetic](http://wiki.ros.org/Installation/Ubuntu) with the `ros-kinetic-desktop-full` option.
This will install Gazebo 7 and OpenCV 3.3.1, among other things.

### 4. Get Flock

Create a catkin workspace:
~~~
source /opt/ros/kinetic/setup.bash
mkdir -p ~/flock_catkin_ws/src
cd ~/flock_catkin_ws/
catkin_make
source devel/setup.bash
~~~

Download and compile flock:
~~~
cd ~/flock_catkin_ws/src
git clone https://github.com/clydemcqueen/flock.git
cd ..
catkin_make
~~~

### 5. Test the environment

Turn on the drone, connect to it via wi-fi, and test the environment:
~~~
python ~/flock_catkin_ws/src/flock/flock_driver/scripts/environment_test.py
~~~
This script will connect to the drone, display a video feed in an OpenCV window.
It will also look for ArUco 6x6 markers highlight them in green.

## Design

Flock provides these packages:
* `flock` meta-package glue
* `flock_msgs` message types
* `flock_description` robot description files
* `flock_driver` interface between the Tello hardware and ROS, not required for simulation
* `flock_base` base nodes
* `flock_rviz` extensions to rviz
* `flock_gazebo` extensions to Gazebo

# Turtlebot Walker
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Project Overview
This project implements a roomba type robot on the turtlebot platform.It has a launch file `demo.launch` which launches the turtlebot gazebo simulation and also launches the custom `walker` node.

## Licence
MIT License

Copyright (c) 2018 Saurav Kumar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Assumptions
- ROS Kinetic is installed properly.
- ROS environment is properly configured.
- Dependencies are met

## Dependencies
- ROS Kinetic
- Turtlebot packages
- geometry_msgs
- move_base_msgs
- roscpp
- rospy
- sensor_msgs
- std_msgs


 To install turtlebot, type the following:
```
sudo apt-get install ros-kinetic-turtlebot-*
```

## Standard build via command-line
```
cd <path to workspace>
mkdir src
cd src
git clone --recursive https://github.com/sauravkdeo/turtlebot_walker.git
cd ..
catkin_make
```
## Steps to run the package

### Sourcing to .bashrc
- open the .bashrc file located in the home folder using your favorite editor.Add the undermentioned lines at the end of the file and then save it.
- This step is done to avoid sourcing  ~/```path to workspace```/devel/setup.bash every time.
```
source ~/<path to workspace>/devel/setup.bash
ex:
source ~/catkin_ws/devel/setup.bash
```

### Using rosrun

####  Open three terminals concurrently :

- Run following commands in Terminal 1 :

```
roscore
```

- Run following commands in Terminal 2 for turtlebot simulation:

```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
- Run following commands in Terminal 3 for walker ros node:

```
rosrun turtlebot_walker walker
```
### Using roslaunch
- To use the launch file, type the undermentioned command in the terminal :
```
roslaunch turtlebot_walker demo.launch
```

## Recording bag files with the launch file

Once the project is build using catkin_make as described earlier. Run the command below to launch the nodes and record all the topics except the camera data. The bag file will be in the results directory once the recording is complete. By default it records for 30 seconds and it can be changed with the  argument `secs`

```
roslaunch turtlebot_walker demo.launch record:=true secs:=48
```

The bag file is stored in `..turtlebot_walker/results/walker.bag`

## Playing back the bag file

First, navigate to the results folder.

```
cd <path to repository>/results
```

Ensure that the roscore is running. Then in a new terminal, run the undermentioned command from results directory.

```
rosbag play walker.bag
```

You will be able to see the elapsed time output on the screen. It would be playing the same messages that were recorded on all the recorded topics.

You can also view all the messages being published on a topic e.g. `/mobile_base/commands/velocity`.

```
rostopic echo mobile_base/commands/velocity
```

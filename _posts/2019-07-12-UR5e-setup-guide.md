---
layout: post
title: UR5e Setup Guide
author: Meiying Qin
description: "Setup Guide for UR5e"
tags: [UR5e,robot,tutorial]
categories: [wiki]
comments: false
permalink: ur5e_setup_guide.html
excerpt_separator: <!-- More -->
---

Welcome to the Setup Guide for the UR5e robot.

<!-- More -->

## Ubuntu System

The following instructions has been tested on Ubuntu 16.04 only.
- You can get the 64 bits system [here](http://releases.ubuntu.com/16.04/).
- To create a bootable USB, please refer to information [here](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows#0).
- You are recommended to partition the harddrive with the `Disks` that comes with Ubuntu. Otherwise, you might see this during installation:
> The partition * assigned to / starts at an offset of * bytes from the minimum alignment for this disk, which may lead to very poor performance.


## ROS

The version `kinetic` is used for UR5e. Basically, you just need to follow [these instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu). The commands are listed here for your convenience.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

```
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

```
sudo rosdep init
rosdep update
```

```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

You are also recommended to get the catkin tools, the official installation guide is [here](https://catkin-tools.readthedocs.io/en/latest/installing.html):

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```

```
sudo apt-get update
sudo apt-get install python-catkin-tools
```

## UR5e ROS
The UR5e ros installation official guide can be found [here](http://wiki.ros.org/action/show/universal_robots?action=show&redirect=universal_robot). Here is the command you need:
```
sudo apt-get install ros-kinetic-universal-robots
```

## UR5e driver
You will also need the ur driver package. There are currently two packages available, `ur_driver` and `ur_modern_driver`. It is highly recommended that you get the `ur_modern_driver`. Also it is highly recommended that you create a workspace for the libraries and a separate one for your projects.

# Create library workspace 

Please feel free to create the directory anywhere you want and with any names you like

```
mkdir -p ~/ros_libs_ws/src
cd ~/ros_libs_ws
catkin build
```
Later you can save all your libraries in ros_libs_src.


# Download the driver

You may choose to download it from the official souce, but I ran into problems with it

```
cd ~/ros_libs_ws/src
git clone https://github.com/ScazLab/ur_modern_driver.git
```

# Get the dependencies
```
cd  ~/ros_libs_ws
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```


# Build the driver

```
cd ~/ros_libs_ws
catkin build
```

# Source the bash file
```
echo "source ~/ros_libs_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## UR5e simulator
You can get the UR sim [here](https://www.universal-robots.com/download/?option=51846#section41511). We downloaded the current latest one, which is `UR Sim for Linux 5.3.1`. Download the simulator by following the [link](https://www.universal-robots.com/download/?option=51846#).

To install the simulator, save it to your home folder. Change to your home folder and extract the file to the root of your home folder:
```
cd ~
tar xvzf URSim_Linux-5.3.1.64192.tar.gz
cd ursim-5.3.1.64192
./install.sh
```

However, you will see this error message:

> Setting up runit (2.1.2-3ubuntu1) ...
> start: Unable to connect to Upstart: Failed to connect to socket /com/ubuntu/upstart: Connection refused
> dpkg: error processing package runit (--configure):
>  subprocess installed post-installation script returned error exit status 1
> Processing triggers for ureadahead (0.100.0-19.1) ...
> Errors were encountered while processing:
>  runit
> E: Sub-process /usr/bin/dpkg returned an error code (1)


This is because the simulator was only tested in Ubuntu 14 when we install it, and the latest one doesn't work with 16 because the upstart package was replaced with systemd. To bypass this package, just do the following:
```
cd /var/lib/dpkg/info/
sudo subl runit.postinst
```

Of course, you can use any text editor you like here. You just need to comment out the following lines (which is lines 58-60 on my installation):
```
if [ -x /sbin/start ]; then #provided by upstart
  /sbin/start runsvdir
fi
```
Change it to:
```
#if [ -x /sbin/start ]; then \#provided by upstart
#  /sbin/start runsvdir
#fi
```
Then you can do:
```
sudo apt-get install -f
```

And continue with your installation:
```
cd ~/ursim-5.3.1.64192
./install.sh
```

You can run the simulator by clicking the icon on the desktop or with the command line:
```
cd ~/ursim-5.3.1.64192
./start-ursim.sh
```

## Simple Tutorials

After you turn on the robot and set it to `Remote Control`, to connect to the real robot, you need to (You will need to replace `IP_OF_THE_ROBOT` with the real robot ip, and [this cheatsheet](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/UR5e_cheatsheet) might include information on how to find the robot ip):

```
roslaunch ur_modern_driver ur5e_bringup.launch robot_ip:=IP_OF_THE_ROBOT
```

To use a simulator, you will need to first run the simulator, and turn on the robot and set it to remote control, with the same way as on the real robot. The ip will be `0.0.0.0` which is the lcoalhost, then connect to the simulator:

```
roslaunch ur_modern_driver ur5e_bringup.launch robot_ip:=0.0.0.0
```

Once to connect to the real robot or the simulator, the commands will be the same in the mini tutorials below. If you cannnot see the remote control, here is how to enable it: 

the sandwich menu on the top right corner &rightarrow; settings &rightarrow; System &rightarrow; Remote Control &rightarrow; Enable

### Get joint angles

You can get the current joint angles form the topic `/joint_states` after launching ur5e_bringup.launch in ur_modern_driver. You can view it directly with `rostopic echo /joint_states`. In the code, just subscribe to this topic. For more topics provided by the package, please refer to [this](https://github.com/ScazLab/ur_modern_driver/blob/kinetic-devel/README.md).

### Set joint angles



## moveit!

todo (no plans to complete this part yet...)

## Collaboration

- **Please edit this file** to add things you think are going to be useful. You can edit it by modifying [this file](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2019-07-12-UR5e-setup-guide.md)

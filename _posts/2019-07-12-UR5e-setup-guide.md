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

## Ubuntu System

The following instructions has been tested on Ubuntu 16.04 only.
- You can get the 64 bits system [here](http://releases.ubuntu.com/16.04/).
- To create a bootable USB, please refer to information [here](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows#0).
- You are recommended to partition the harddrive with the `Disks` that comes with Ubuntu. Otherwise, you might see this during installation:
> The partition * assigned to / starts at an offset of * bytes from the minimum alignment for this disk, which may lead to very poor performance.


## ROS

The version `kinetic` is used for UR5e. Basically, you just need to follow [these instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu). The commands are listed here for your convenience.

"""
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
"""

"""
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
"""

"""
sudo rosdep init
rosdep update
"""

"""
echo "source /opt/ros/kinetic/setup.bash" >> \~/.bashrc
source \~/.bashrc
"""

"""
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
"""

You are also recommended to get the catkin tools, the official installation guide is [here](https://catkin-tools.readthedocs.io/en/latest/installing.html):

"""
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
"""

"""
sudo apt-get update
sudo apt-get install python-catkin-tools
"""

## UR5e driver
The UR5e ros installation official guide can be found [here](http://wiki.ros.org/action/show/universal_robots?action=show&redirect=universal_robot). Before you can install the ROS-Industrial package for the UR robot arms, you will need the ur driver package. There are currently two packages available, `ur_driver` and `ur_modern_driver`. It is highly recommended that you get the `ur_modern_driver`. Also it is highly recommended that you create a workspace for the libraries and a separate one for your projects.

1. create library workspace (also please feel free to create the directory anywhere you want and with any names you like)

"""
mkdir -p \~/ros_libs_ws/src
cd \~/ros_libs_ws
catkin build
"""
Later you can save all your libraries in ros_libs_src.

2. Download the driver

"""
cd \~/ros_libs_ws/src
git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git
"""


3. Build the driver

"""
cd \~/ros_libs_ws
catkin build
"""

4. Source the bash file
"""
echo "source \~/ros_libs_ws/devel/setup.bash" >> \~/.bashrc
source \~/.bashrc
"""

## UR5e ROS
Now let's install the ROS-Industrial UR5e adapter

"""
sudo apt-get install ros-kinetic-universal-robots
"""

## UR5e simulator
You can get the UR sim [here](https://www.universal-robots.com/download/?option=51846#section41511). We downloaded the current latest one, which is `UR Sim for Linux 5.3.1`. Download the simulator by following the [link](https://www.universal-robots.com/download/?option=51846#).

To install the simulator, save it to your home folder. Change to your home folder and extract the file to the root of your home folder:
"""
cd \~
tar xvzf URSim_Linux-5.3.1.64192.tar.gz
cd ursim-5.3.1.64192
./install.sh
"""

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
"""
cd /var/lib/dpkg/info/
sudo subl runit.postinst
"""

Of course, you can use any text editor you like here. You just need to comment out the following lines (which is lines 58-60 on my installation):
"""
if [ -x /sbin/start ]; then #provided by upstart
  /sbin/start runsvdir
fi
""" 
Change it to:
"""
\#if [ -x /sbin/start ]; then \#provided by upstart
\#  /sbin/start runsvdir
\#fi
"""
Then you can do:
"""
sudo apt-get install -f
"""

And continue with your installation:
"""
cd \~/ursim-5.3.1.64192
./install.sh
"""

You can run the simulator here:
"""
cd \~/ursim-5.3.1.64192
./start-ursim.sh
"""

## Simple Tutorials
### Get joint angles
### Set joint angles

## moveit!

to be added!

## Collaboration

- **Please edit this file** to add things you think are going to be useful. You can edit it by modifying [this file](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2019-07-12-UR5e-setup-guide.md)

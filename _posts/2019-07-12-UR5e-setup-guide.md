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

Welcome to the Setup Guide for the UR5e robot. (Added instructions for the new UR ros driver)

<!-- More -->

## Ubuntu System

The following instructions has been tested on Ubuntu 18.04 only.
- You can get the 64 bits system [here](http://releases.ubuntu.com/18.04/).
- To create a bootable USB, please refer to information [here](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows#0).
- You are recommended to partition the harddrive with the `Disks` that comes with Ubuntu. Otherwise, you might see this during installation:
> The partition * assigned to / starts at an offset of * bytes from the minimum alignment for this disk, which may lead to very poor performance.

## UR Simulator

If tried with installing ROS first and then the simulator, but it somehow deleted a few ros packages. I could be wrong, but installaing the simulator works for me.

Download the simulator from here:
https://www.universal-robots.com/download/?option=51846#section41511

The version tested is 5.3.1, which is the same as the hardware in the lab. Install the simulator with the following instructions (Following the instructions on the website would not install the simulator properly on Ubuntu 18. The instructions below are adapted from https://forum.universal-robots.com/t/ursim-no-controller-error/2829/7).

- unzip the software to the home folder (the same as the installation instructions from the download link)
- install java 8 (note: other version doesn't work), and make sure it is the default version with `java -version`
```
sudo apt install openjdk-8-jre
```
- install libxmlrpc for 32 bit executable
```
sudo apt install libxmlrpc-c++8v5:i386
```
- change the file install.sh: in commonDependencies: Change libcurl3 to libcurl4
- run `./install.sh` to install the simulator

To use the simulator, you can either run `./start-ursim.sh`(and UR5 is the default) or double-click the shortcut created on the desktop (However, this won't show error messages if anything goes wrong). 

If you see errors like `java.awt.AWTError: Assistive Technology not found: `, this can be solved by:
```
This can be done by editing the accessibility.properties file for OpenJDK:
    sudo vim /etc/java-8-openjdk/accessibility.properties
Comment out the following line:
    assistive_technologies=org.GNOME.Accessibility.AtkWrapper
```
The solution was found: https://github.com/Microsoft/vscode-arduino/issues/644

If you cannot start the arm because no controllers found, manually execute `starturcontrol.sh` should solve the issue.


## ROS

The version `melodic` is used for UR5e. Basically, you just need to follow [these instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). The commands are listed here for your convenience.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

```
sudo apt update
sudo apt install ros-melodic-desktop-full
```

```
sudo rosdep init
rosdep update
```

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
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

## UR5e ROS Driver

It is highly recommended that you create a workspace for the libraries and a separate one for your projects.

# Create library workspace 

Please feel free to create the directory anywhere you want and with any names you like

```
mkdir -p ~/ros_lib_ws/src
cd ~/ros_lib_ws
catkin build
```
Later you can save all your libraries in ros_lib_src.


# Download the driver

All of the official ros packages are: https://github.com/UniversalRobots/
The driver is here: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

The following installation instructions are adapted from the github

```
cd ~/ros_lib_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-path src --ignore-src -y

cd ..
catkin build
echo "source ~/ros_lib_ws/devel/setup.bash" >> ~/.bashrc
```

The robot arm has been prepared to use this, so you can skip the section `Setting up a UR robot for ur_robot_driver`


## UR5e simulator

# Install simulator for Ubuntu 16.04 (Those are the old instructions, but may be useful for anybody who wish to install in on a Ubunt 16 machine)

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

# Prepare the simulator for the ros drivers

To install the URCap, copy the file `externalcontrol-1.0.urcap` in `~/ros_lib_ws/src/src/Universal_Robots_ROS_Driver/ur_robot_driver/resources` to `~/ursim-5.3.1.64192/programs.UR5/` or `~/ursim-5.3.1.64192/programs/` if you are running the simulator, then following the instructions on https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md as if you are installing it on an actual arm. You should set the ip to be localhost (127.0.0.1) as the remote host machine on a simulator.

# Run the Simulator

Run the ros driver first:
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=127.0.0.1
```

Also make sure you load the externalcontrol program on the simulator, and then click run (The same as if you are working a real arm)

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

### Get end effector positon and pose

You can get the information form the topic `\tf`. The pose is in quaternion.

### Set joint angles

You can publish commands as strings as messages to the topic `ur_driver/URScript`. The syntax is the same as the language URScript. The documentation can be found [here](https://www.universal-robots.com/download/?option=50688#section50495). More information can be found [here](https://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/universal-robots-script-programming/). You may also consider using the topics `follow_joint_trajectory/*` which wraps up moveit.


## moveit!

todo (no plans to complete this part yet...)

## Others

### Using urdfpy

While polyscope and ros does provide kinematics information, urdfpy is a python lib that can quickly and easily help you understand what the names of joints and links are. To install:

```
pip install urdfpy
``` 

If you don't have pip on you computer, install it:

```
sudo apt-get install python-pip
```

And if you need to upgrade pip:

```
sudo pip install --upgrade pip
```

If you see this error after installation
>Traceback (most recent call last):
>  File "/usr/bin/pip", line 9, in <module>
>    from pip import main
>ImportError: cannot import name main

You can resolve it with:

```
sudo subl /usr/bin/pip
```

And change 

> from pip import main

to 

> from pip.\_internal import main

If you see this while installing `urdepy`:

> RROR: Command "python setup.py egg_info" failed with error code 1 in /tmp/pip-install-QwGdQx/urdfpy/

Just run:

```
 sudo pip install --upgrade setuptools
```

More information about urdfpy can be found [here](https://urdfpy.readthedocs.io/en/latest/examples/)


## Collaboration

- **Please edit this file** to add things you think are going to be useful. You can edit it by modifying [this file](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2019-07-12-UR5e-setup-guide.md)

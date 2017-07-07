---
layout: post
title: Baxter Cheat Sheet
author: Olivier Mangin and Alessandro Roncone
description: "Cheat sheet for the baxter robot"
tags: [baxter,robot,tutorial]
categories: [wiki]
comments: false
permalink: baxter_cheat_sheet.html
image:
  feature: baxter_tictactoe.jpg
excerpt_separator: <!-- More -->
---

Welcome to the cheat sheet for the Baxter robot.

## Useful links

- Baxter uses [ROS indigo](http://wiki.ros.org/indigo).
  - Installation according to ROS: <http://wiki.ros.org/indigo/Installation/Ubuntu>
  - Installation according to Alessandro: <https://alecive.github.io/ros_installation.html>, <http://wiki.ros.org/indigo/Installation/Source>
- Baxter:
  - [Setup](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) (see the following for the setup)
  - [Hello Baxter](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter)
  - [Baxter API](http://sdk.rethinkrobotics.com/wiki/API_Reference)
- [Alessandro's ROS tips and tricks](https://alecive.github.io/ros_concepts.html)

<!-- More -->

## How to use the robot

### How to turn it on

 0. Turn on the robot by pressing the button in the back of its base. Wait for the robot to finish its start-up phase (it takes a couple minutes).
 1. Be sure that the system you're running the code has access to the Baxter robot. This is usually done by running the `baxter.sh` script that should be provided in your Baxter installation. See [here](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter#Source_ROS_Environment_Setup_Script) for more info. **Important** → for what concerns the Baxter robot on the ScazLab, this means that every time you have to run some ROS software to be used on the robot you should open a new terminal, and do the following: ` cd ros_devel_ws && ./baxter.sh `. A change in the terminal prompt should acknowledge that you now have access to `baxter.local`. __Please be aware of this fact when you operate the robot__.
 2. Untuck the robot. We have an alias for this, so you just have to type `untuck` in the terminal you previously opened.

### How to turn it off

 1. Tuck the robot. We have an alias for this, so you just have to type `tuck` in a terminal with access to the robot system.
 2. Shut down the robot with the button in the back of its base.

## Software Infrastructure

This is the software infrastructure that we are currently using to perform human-robot collaborative tasks with the Baxter robot:

- **Baxter collaboration**: The interface with the Baxter (low level) is done in [this repository](https://github.com/scazlab/baxter_collaboration). In there, both perception and control are taken care of. The repository readme also details the components of the architecture.
- **HTM**: High level POMDP planning is done in [this repository](https://github.com/scazlab/htm).
- **Baxter tictactoe**: The tictactoe demo is located in [this repository](https://github.com/ScazLab/baxter_tictactoe).

They are public repositories, but to work on them you need to be either a member of the scazlab organization or to be a contributor for the specific repository.

## Facts

- The ROS core is running on the robot, you just have to connect to it.
- We use a separate ethernet card to connect the computer to the robot. That way the robot is not connected to the general university network.
  - Use `link-local` configuration for the computer-robot interface. In other words the computer and the robot each find a free IP address ([wikipedia/Link-local_address](https://en.wikipedia.org/wiki/Link-local_address).
  - Baxter is automatically discovered through [zeroconf](https://en.wikipedia.org/wiki/Zero-configuration_networking) as `baxter.local`. (On ubuntu see: [HowToZeroconf](https://help.ubuntu.com/community/HowToZeroconf)).
  - Typically you only have to set `baxter_hostname="baxter.local"` and `your_ip="$(/sbin/ip -o -4 addr list eth0 | awk '{print $4}' | cut -d/ -f1)"` in the `baxter.sh` file. Replace `eth0` by the interface connected to Baxter.

## Misc

- [(Un/)Tuck](https://github.com/RethinkRobotics/sdk-docs/wiki/Tuck-Arms-Example): `rosrun baxter_tools tuck_arms.py -t # and -u`

- [Enable / Disable](http://sdk.rethinkrobotics.com/wiki/Enable_Robot_Tool): `rosrun baxter_tools enable_robot.py -e # and -d`

- [Cameras](http://sdk.rethinkrobotics.com/wiki/API_Reference#Cameras): through topics:

  ~~~
  /cameras/head_camera/image
  /cameras/right_hand_camera/image
  /cameras/left_hand_camera/image
  ~~~

- [Screen](http://sdk.rethinkrobotics.com/wiki/API_Reference#Screen_.28xdisplay.29): just push images to `/robot/xdisplay` or execute
  `rosrun baxter_examples xdisplay_image.py --file=image.png`

## Collaboration

- **Please edit this file** to add things you think are going to be useful. You can edit it by modifying [this file](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2015-02-19-Baxter-cheat-sheet.md)
- Please register to the [scazlab.slack.com/#baxter](https://scazlab.slack.com/messages/baxter/) channel.

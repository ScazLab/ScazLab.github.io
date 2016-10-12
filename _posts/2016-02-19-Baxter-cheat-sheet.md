---
layout: post
title: Baxter cheat sheet
author: Olivier Mangin
description: "Cheat sheet for the baxter robot"
tags: [baxter,robot,tutorial]
categories: [wiki]
comments: false
image:
  feature: baxter_tictactoe.jpg
excerpt_separator: <!-- More -->
---
# Useful links

- Baxter uses [ROS indigo](http://wiki.ros.org/indigo). Installation:
  - According to ROS: <http://wiki.ros.org/indigo/Installation/Ubuntu>
  - According to Alessandro: <http://alecive.github.io/blog/2015/11/12/ROS-naive-installation>, <http://wiki.ros.org/indigo/Installation/Source>
- Baxter:
  - [Setup](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) (see the following for the setup)
  - [Hello Baxter](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter)
  - [Baxter API](http://sdk.rethinkrobotics.com/wiki/API_Reference)
- [Alessandro's ROS tips and tricks](http://alecive.github.io/blog/2016/02/08/ROS-concepts/)

<!-- More -->

# Software Infrastructure

This is the software infrastructure that we are currently using to perform human-robot collaborative tasks with the Baxter robot:

- **Baxter collaboration**: The interface with the Baxter (low level) is done in [this repository](https://github.com/scazlab/baxter_collaboration). In there, both perception and control are taken care of. The repository readme also details the components of the architecture.
- **HTM**: High level POMDP planning is done in [this repository](https://github.com/scazlab/htm).
- **Baxter tictactoe**: The tictactoe demo is located in [this repository](https://github.com/ScazLab/baxter_tictactoe).

They are public repositories, but to work on them you need to be either a member of the scazlab organization or to be a contributor for the specific repository.

# Facts

- The ROS core is running on the robot, you just have to connect to it.
- We use a separate ethernet card to connect the computer to the robot. That way the robot is not connected to the general university network.
  - Use `link-local` configuration for the computer-robot interface. In other words the computer and the robot each find a free IP address ([wikipedia/Link-local_address](https://en.wikipedia.org/wiki/Link-local_address).
  - Baxter is automatically discovered through [zeroconf](https://en.wikipedia.org/wiki/Zero-configuration_networking) as `baxter.local`. (On ubuntu see: [HowToZeroconf](https://help.ubuntu.com/community/HowToZeroconf)).
  - Typically you only have to set `baxter_hostname="baxter.local"` and `your_ip="$(/sbin/ip -o -4 addr list eth0 | awk '{print $4}' | cut -d/ -f1)"` in the `baxter.sh` file. Replace `eth0` by the interface connected to Baxter.

# Quick reminders

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

# Collaboration

- **Please edit this file** to add things you think are going to be useful. You can edit it by modifying [this file](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2015-02-19-Baxter-cheat-sheet.md)
- Please register to the [scazlab.slack.com/#baxter](https://scazlab.slack.com/messages/baxter/) channel.

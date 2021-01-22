---
layout: post
title: Hello Stretch Setup Guide
author: Debasmita Ghose
description: "Setup Guide for the Stretch Robot with basic ROS commands"
tags: [Stretch,perception,tutorial]
categories: [wiki]
comments: false
permalink: stretch_robot.html
excerpt_separator: <!-- More -->
---

Welcome to the Setup and Startup Guide for the Stretch Robot.

<!-- More -->


# Starting up Stretch

- Connect the stretch to the power source and switch it on.
- Wait for the onboard Ubuntu to start up until you hear a beep.
- Press the **Mode** button on the battery for 3 seconds until the lights on the bottom pannel come up. Then cycle through the modes and set Mode to **12V Supply**.
- Connect the robot to a monitor, keyboard and mouse to start coding. 

# ROS Commands to Interact with Stretch

- Launch Stretch Drivers  
``` roslaunch stretch_core stretch_drivers.launch ```

- Launch On-board RealSense Camera  
``` roslaunch stretch_core d435i_high_resolution.launch ```

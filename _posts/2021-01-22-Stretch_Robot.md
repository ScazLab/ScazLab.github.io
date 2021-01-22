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


## Starting up Stretch

- Connect the stretch to the power source and switch it on.
- Wait for the onboard Ubuntu to start up until you hear a beep.
- Press the **Mode** button on the battery charger for 3 seconds until the lights on the bottom pannel come up. Then cycle through the modes and set Mode to **12V Supply**.
- Connect the robot to a monitor, keyboard and mouse to start coding. 

## Shutting down Stretch

- Shut down the PC from the desktop.
- After the laser range finder stops spinning, switch off the robot.
- On the battery charger long press the **Mode** button until the top panel is illuminates. Select **12V AGM** for the robot to charge when not in use. 

## ROS Commands to Interact with Stretch

- Launch Stretch Drivers  
``` roslaunch stretch_core stretch_drivers.launch ```

- Launch On-board RealSense Camera  
``` roslaunch stretch_core d435i_high_resolution.launch ```

## References

- Starting up instructions and best practices - https://docs.hello-robot.com/quick_start_guide/
- Battery Charger Manual - https://no.co/media/nocodownloads/format/g/e/genius10na_user_guide_1.pdf
- Troubleshooting `Transport Error` - https://docs.hello-robot.com/troubleshooting_guide/


## Contributing

To contribute to this page, make changes here: https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2021-01-22-Stretch_Robot.md

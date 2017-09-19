---
title: Lab Cheat Sheet
author: Alessandro Roncone
description: "Things to do if you are new to the lab"
tags: [scazlab,robot,tutorial]
categories: [wiki]
comments: false
permalink: lab_cheat_sheet.html
image:
  feature: baxter_tictactoe.jpg
excerpt_separator: <!-- More -->
---

The following is a list of initial things to do if you are new to the lab. Please consider this a non-exhaustive, always changing cheat sheet. As such, please ask us to modify it if there is anything missing or anything you would like to add!!

<!-- More -->

# Initial setup

 1. Create an account on our [official Slack team](scazlab.slack.com), and share your username with your supervisor. Ideally, install slack on your machine as well as on your phone in oder to be always up to date with what happens on the lab. Most of our internal communication happens there.
 2. Create an account on [GitHub](github.com), and share your username with your supervisor.
 3. Write to [Larissa](http://scazlab.yale.edu/people/larissa-hall) in order to manage all the bureaucracy and get lab access with your Yale ID. Put your supervisor in Cc.

# Software installation

For what concerns the software, your mileage may vary. This is what you need to if you are doing software development on the Human--Robot Collaboration side of the lab (but it may apply to others as well)

## Install Ubuntu [14.04]

Linux (and Ubuntu) is our operating system of choice. It is not important to be a Linux expert, but you need at least to have it installed in order to be able to develop on an environment as similar as possible to the one you will use when using the Baxter Robot (or similar ROS-based robots).

You have the following options:

1. You don't want to use your laptop: we can provide you with a machine with Ubuntu and ROS pre-installed (or we may ask you to do the job)
2. You want to work on your laptop:
    1. You already use Ubuntu → great! You are a true nerd. Move on to the next section
    2. You use MacOS / Windows:
        1. You can install Ubuntu side by side with your main OS partition [preferred option as it's the most flexible overall and it pays off over the time]
        2. You can use a virtual machine with Ubuntu on it
        3. You can use Docker container to virtualize an Ubuntu machine on your main OS with ROS preinstalled [but this is the option that requires you to be an advanced terminal user]
        4. If you use MacOS, there is also the option to install ROS on Mac directly, although it is highly unsupported and usually there are problems. But many students were able to do so!

**NOTE:** the Baxter Robot ~~is stuck to~~ uses Ubuntu 14.04, so it is highly recommended to use that.

## Install ROS [indigo]

ROS is the so-called Robot Operating System. It is the core software we use to interface with the robot.

Also here, you have some specific requirements. Baxter ~~is stuck to~~ uses [ROS indigo](http://wiki.ros.org/indigo). Here are some installation instructions:

  - [Installation according to ROS](http://wiki.ros.org/indigo/Installation/Ubuntu)
  - [Installation from sources](http://wiki.ros.org/indigo/Installation/Source)
  - [Installation according to Alessandro](https://alecive.github.io/ros_installation.html)

It is **very useful** and **recommended** to have some degree of understanding of ROS. This is a [useful link](https://alecive.github.io/ros_concepts.html).

## Setup your editor of choice / GitHub options

Here are some good things to set up before contributing to our code:

 * Rebase by default when doing git pull → `git config --global branch.autosetuprebase always`
 * Use **ALWAYS** tab as spaces → on Sublime text, you should add this in your preferences `"translate_tabs_to_spaces": true`
 * Ensure newline at the end of files → on Sublime text, you should add this in your preferences `"ensure_newline_at_eof_on_save": true`
 * Trim trailing white space on save → on Sublime text, you should add this in your preferences `"trim_trailing_white_space_on_save": true`
 * **DO NOT** create backup files (those that end with `~`)
 * Set up `4` spaces as tab width.

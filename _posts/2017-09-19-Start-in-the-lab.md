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

The following is a list of initial things to do if you are new to the lab.

<!-- More -->

# Initial setup

 * Create an account on our [official Slack team](scazlab.slack.com), and share your username with your supervisor. Ideally, install slack on your machine as well as on your phone in oder to be always up to date with what happens on the lab. Most of our internal communication happens there.
 * Create an account on [GitBub](github.com), and share your username with your supervisor.
 * Write to [Larissa Hall](http://scazlab.yale.edu/people/larissa-hall) in order to manage all the bureaucracy and get lab access with your Yale ID.

# Software installation

For what concerns the software, your mileage may vary. This is what you need to if you are doing software development on the Human--Robot Collaboration side of the lab (but it may apply to others as well)

## Install Ubuntu [14.04]

Linux (and Ubuntu) is our operating system of choice. It is not important to be a Linux expert, but you need at least to have it installed in order to be able to develop on an environment as similar as possible to the one you will use when using the Baxter Robot (or similar ROS-based robots).

You have the following options:

 1. You don't want to use your laptop: we can provide you with a machine with Ubuntu and ROS pre-installed (or we may ask you to do the job)
 2. You want to work on your laptop:
    1. You already use Ubuntu → great! You are a true nerd.
    2. You use MacOS / Windows:
      1. You can install Ubuntu side by side with your main OS partition [preferred option as it's the most flexible overall and it pays off over the time]
      2. You can use a virtual machine with Ubuntu on it
      3. You can use Docker container to virtualize an Ubuntu machine on your main OS [but this is the option that requires you to be an advanced terminal user]
      4. If you use MacOS, there is also the option to install ROS on Mac, although it is highly unsupported and usually there are problems. But many students were able to do so!

**NOTE:** the Baxter Robot ~~is stuck to~~ uses Ubuntu 14.04, so it is highly recommended to use that.

## Install ROS [indigo]

## Setup your editor of choice

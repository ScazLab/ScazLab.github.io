---
title: Lab Cheat Sheet
author: Alessandro Roncone
description: "Things to do if you are new to the lab"
tags: [scazlab,robot,tutorial]
categories: [wiki]
comments: false
permalink: lab_cheat_sheet.html
image:
  feature: group_pic.jpg
excerpt_separator: <!-- More -->
---

# Contents
{:.no_toc}

* This line will be replaced by the ToC, excluding the "Contents" header
{:toc}

The following is a list of initial things to do if you are new to the lab. Please consider this a non-exhaustive, always changing cheat sheet. As such, please ask us to modify it if there is anything missing or anything you would like to add!!

<!-- More -->

# 1. Initial setup

 1. Create an account on our [official Slack team](scazlab.slack.com), and share your username with your supervisor. Ideally, install slack on your machine as well as on your phone in oder to be always up to date with what happens on the lab. Most of our internal communication happens there.
 2. Create an account on [GitHub](github.com), and share your username with your supervisor.
 3. Write to [Larissa](http://scazlab.yale.edu/people/larissa-hall) in order to manage all the bureaucracy and get lab access with your Yale ID. Put your supervisor in Cc.

# 2. Software

For what concerns the software, your mileage may vary. This is what you need to if you are doing software development on the Human--Robot Collaboration side of the lab (but it may apply to others as well)

## 2.1 Install Ubuntu [14.04]

Linux (and Ubuntu) is our operating system of choice. It is not important to be a Linux expert, but you need at least to have it installed in order to be able to develop on an environment as similar as possible to the one you will use when using the Baxter Robot (or similar ROS-based robots).

You have the following options:

1. You don't want to use your laptop: we can provide you with a machine with Ubuntu and ROS pre-installed (or we may ask you to do the job)
2. You want to **work on your laptop**:
    1. **You already use Ubuntu** → great! You are a true nerd. Move on to the next section
    2. **You use MacOS / Windows:**
        1. You can **install Ubuntu side by side** with your main OS partition [**recommended** option as it's the most flexible overall and it pays off over the time]
        2. You can **use a virtual machine** with Ubuntu on it
        3. You can **use Docker** containers to virtualize an Ubuntu machine on your main operating system. Our Docker containers come with ROS and our code already preinstalled! Please be aware that this is definitely the option that requires you to be an advanced terminal user. You can get our containers [here](https://hub.docker.com/r/scazlab/human_robot_collaboration/)
        4. If you use another linux distribution or MacOS, there is also the option to install ROS on it directly, although it is (highly) unsupported and usually there are problems. But many students were able to do so!

**NOTE:** the Baxter Robot ~~is stuck to~~ uses Ubuntu 14.04, so it is highly recommended to use that.

## 2.2 Install ROS [indigo]

ROS is the so-called Robot Operating System. It is the core software we use to interface with the robot.

Also here, you have some specific requirements. Baxter ~~is stuck to~~ uses [ROS indigo](http://wiki.ros.org/indigo). Here are some installation instructions:

  - [Installation according to ROS](http://wiki.ros.org/indigo/Installation/Ubuntu)
  - [Installation from sources](http://wiki.ros.org/indigo/Installation/Source)
  - [Installation according to Alessandro](https://alecive.github.io/ros_installation.html)

## 2.3 Learn about ROS

To have some degree of understanding of ROS, please read the following documents:
  - [an introduction by Alessandro](https://alecive.github.io/ros_concepts.html),
  - [the official introduction](http://wiki.ros.org/ROS/Concepts),
  - [the documentation on nodes](http://wiki.ros.org/Nodes).

If you still need some practice, you might have a look at [ROS tutorials](http://wiki.ros.org/ROS/Tutorials).

To learn more about the Baxter robot, please also have a look at [this page]({% post_url 2017-03-08-Baxter-cheat-sheet %}).

## 2.4 Install the Scazlab software

If you are going to work on the Baxter Robot, these are the ROS packages you will need to use and install:


- **Human-Robot collaboration**: The interface with the Baxter (low level) is done in [this repository](https://github.com/scazlab/human_robot_collaboration). In there, both perception and control are taken care of. The repository `README.md` also details the components of the architecture.
- **Task Models**: High level POMDP planning is done in [this repository](https://github.com/scazlab/task-models).
- **Baxter tictactoe**: (non mandatory) the tictactoe demo is located in [this repository](https://github.com/ScazLab/baxter_tictactoe).

## 2.5 Other tools

### 2.5.1 CMake

`CMake` is core to `ROS` development and any advanced, multi-platform, `C++` based development. It is relatively easy to learn, but very difficult to master. Here are some resources that might be useful:

 - [CGold: The Hitchhiker’s Guide to the CMake](http://cgold.readthedocs.io/en/latest/) is a very long, but complete and thorough set of tools
 - The [LLVM primer on CMake](https://llvm.org/docs/CMakePrimer.html) is useful for the language level (i.e. variables, control loops, commands), but it does not provide interesting information like explanation of `target_include_directories` `target_link_libraries`
 - [This one](https://github.com/onqtam/awesome-cmake#resources) is another long tutorial
 - [This one](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1) is a useful collection of tips and tricks
 - [This](https://codingnest.com/basic-cmake/) instead is a series of tutorials oriented towards students that I saw recently and may be useful
 - [This](https://samthursfield.wordpress.com/2015/11/21/cmake-dependencies-between-targets-and-files-and-custom-commands/) is a nice article about what mess `CMake` is---and also a good way to start learning it

# 3 Guidelines / options / editor preferences

Here are some good things to set up before contributing to our code:

 * Rebase by default when doing git pull → `git config --global branch.autosetuprebase always`
 * Use **ALWAYS** tab as spaces → on Sublime text, you should add this in your preferences `"translate_tabs_to_spaces": true`
 * Ensure newline at the end of files → on Sublime text, you should add this in your preferences `"ensure_newline_at_eof_on_save": true`
 * Trim trailing white space on save → on Sublime text, you should add this in your preferences `"trim_trailing_white_space_on_save": true`
 * **DO NOT** create backup files (those that end with `~`), or at least do not commit them
 * Set up `4` spaces as tab width.
 * Be considerate in using `git`. `git` is a great tool, but it needs to be used carefully in order to maximize its effectiveness. To this end, please:
    * commit frequently, push frequently
    * do not create huge commits with all the files you worked on during your day because it is harder for us to review them
    * use branching and pull requests to implement features/bug fixes
 * Keep your code **ALWAYS** in a compile-able state. We don't want our work on the Baxter to be slowed down by some partial feature that is broken (which is fine) but then breaks all the compilations on the machine


Please be aware that, although the suggestions above are only guidelines, they will be **STRICTLY ENFORCED** throughout our code, as they set the minimum bar to have fruitful and effective collaborations.

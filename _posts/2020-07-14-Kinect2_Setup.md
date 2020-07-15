---
layout: post
title: Kinect2 Setup Guide
author: Meiying Qin
description: "Setup Guide for Kinect2"
tags: [kinect2,perception,tutorial]
categories: [wiki]
comments: false
permalink: kinect2_setup_guide.html
excerpt_separator: <!-- More -->
---

Welcome to the Setup Guide for Kinect2.

<!-- More -->

## System

The following instructions has been tested on Ubuntu 14.04 with ros indigo and Ubuntu 18.04 with ros melodic

### Install libfreenect2

Install necessary libs
clone the libfreenect2 to your home folder (at least not the ros folder, as this is not a ros project)
```
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2

sudo apt-get install build-essential cmake pkg-config

sudo apt-get install libusb-1.0-0-dev

sudo apt-get install libturbojpeg0-dev

sudo apt-get update
sudo apt-get install libglfw3-dev

sudo apt-get install beignet-dev (find the appropriate one)
```

Then install the python freenect2 lib

add the following line `export PKG_CONFIG_PATH=$HOME/freenect2/lib/pkgconfig` and `export LD_LIBRARY_PATH=$HOME/freenect2/lib` to `~/.bashrc`. 

Build:
```
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2
make
make install
```

Setup and test:
```
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
./bin/Protonect
```
If it runs successfully, You will see the demo.

### Install iai-kinect2

clone the repo to your ros workspace

```
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ../..
catkin build iai_kinect2 -DCMAKE_BUILD_TYPE="Release"
```

If you see the error message `[ERROR] [DepthRegistrationOpenCL::init] Build Log: stringInput.cl:190:31: error: call to 'sqrt' is ambiguous`. To fix this, in kinect2_bridge.launch, update `depth_method` to `opengl`(Or the one works for you), and `reg_method` to `cpu`
(reference: <https://lubosz.wordpress.com/2016/03/29/viewing-kinect-v2-point-clouds-with-ros-in-arch-linux/>)

Test
```
roslaunch kinect2_bridge kinect2_bridge.launch
```

To view the results
```
rosrun kinect2_viewer kinect2_viewer kinect2 sd cloud
```

## Kinect Calibration

Following the instructions from: <https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration>

To save a frame, make sure your focus is on the image (i.e., click on the image) and press the keys.

You can safely ignore the errors like:

```
[ERROR] Tried to advertise a service that is already advertised in this node [/kinect2_calib_1593699935406844953/compressed/set_parameters]
[ERROR] Tried to advertise a service that is already advertised in this node [/kinect2_calib_1593699935406844953/compressed/set_parameters]
```

Useful commands:

```(bash)
roslaunch kinect2_bridge kinect2_bridge.launch fps_limit:=2

rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record color

rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate color

rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record ir

rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate ir

rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record sync

rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate sync

rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate depth
```

## References

<https://github.com/code-iai/iai_kinect2>
<https://github.com/OpenKinect/libfreenect2/blob/master/README.md#linux>

## Collaboration

- **Please edit this file** to add things you think are going to be useful. You can edit it by modifying [this file](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2020-07-14-Kinect2-Setup.md)

---
layout: post
title: Azure Kinect Setup and Startup Guide
author: Meiying Qin
description: "Setup Guide for Azure Kinect"
tags: [Kinect Azure,perception,tutorial]
categories: [wiki]
comments: false
permalink: azure_setup_guide.html
excerpt_separator: <!-- More -->
---

Welcome to the Setup and Startup Guide for Azure Kinect.

<!-- More -->

## System

The following instructions has been tested on Ubuntu 18.04 with Ros Melodic only.


## Azure Ubuntu SDK

The instructions are adapted from <https://github.com/microsoft/Azure_Kinect_ROS_Driver>, <https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#Installation>, <https://docs.microsoft.com/en-us/windows-server/administration/linux-package-repository-for-microsoft-software> and <https://www.youtube.com/watch?v=XJQtAod5v2A>.


```bash
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update
```

```bash
sudo apt-get install k4a-tools
```
Before you put "Y" to continue, check which version it is. When this instruction is written, I got `libk4a1.4 libsoundio1`. The version will be used in the next step.

```bash
sudo apt-get install libk4a1.4
sudo apt-get install libk4a1.4-dev 
```
Note: replace the 1.4 with whatever version you got in the previous step

Save the file <https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules> to `/etc/udev/rules.d/`. The file content is copied below for your convenience:
```
# Bus 002 Device 116: ID 045e:097a Microsoft Corp.  - Generic Superspeed USB Hub
# Bus 001 Device 015: ID 045e:097b Microsoft Corp.  - Generic USB Hub
# Bus 002 Device 118: ID 045e:097c Microsoft Corp.  - Azure Kinect Depth Camera
# Bus 002 Device 117: ID 045e:097d Microsoft Corp.  - Azure Kinect 4K Camera
# Bus 001 Device 016: ID 045e:097e Microsoft Corp.  - Azure Kinect Microphone Array

BUS!="usb", ACTION!="add", SUBSYSTEM!=="usb_device", GOTO="k4a_logic_rules_end"

ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097a", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097b", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE="0666", GROUP="plugdev"

LABEL="k4a_logic_rules_end"
```

Afterwards, detach and reattach Azure Kinect devices if attached during this process.

Make sure you update the firmware on the device as well. I find it easier to update the firmware on windows as I don't how to find the version number on linux. I followed this link: <https://docs.microsoft.com/en-us/azure/kinect-dk/update-device-firmware>. Make sure you got the same version when downloading sdk to windows.

To check whether your azure could be run, run this command:
```bash
k4aviewer
```
On the popping window, click `Open Device` and then choose `Start`. 


## Azure ROS wrapper

You will need to install the Azure ros driver, the instruction was referenced from <https://github.com/microsoft/Azure_Kinect_ROS_Driver> (choose the location where you would like to install it to. I install it to `~/ros_lib_ws/src`. Depending on how you initialize your workspace, you may need `catkin_make` to build it):

```bash
cd ~/ros_lib_ws/src
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
cd ..
catkin build
```
Source this folder. Then test it with `roslaunch azure_kinect_ros_driver driver.launch `

To test it, you can visualize the point cloud with `rosrun rviz rviz`, then add a point_cloud2. Make sure you choose the right frame. Then you can visualize the result! It can be somewhat slow if you want to reorient the point cloud. Decrease the fps may help.

More information about how to configure the launch file can be found here <https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/usage.md>

## Connect Multiple Devices

The following instructions are adapted from <https://docs.microsoft.com/en-us/azure/kinect-dk/multi-camera-sync>.

### Hardware configuration

Physically connected the devices using a 3.5-mm audio cable as indicated in <https://docs.microsoft.com/en-us/azure/kinect-dk/multi-camera-sync>. The instructions are tested with 2 devices with no external signal trigger. Both devices are connected to the same host computer. More devices may be possible but not guaranteed.

My configuration is: 
sync out port on the master device -- 3.5-mm audio cable -- sync in port on the subordinate device


### Hosting computer configuration

The method of editting the file `/etc/default/grub` mentioned in <https://docs.microsoft.com/en-us/azure/kinect-dk/multi-camera-sync> (or in <https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/485>) somehow doesn't work for me. After checking with `cat /sys/module/usbcore/parameters/usbfs_memory_mb`, it always returns 16 even if I double checked that the `/etc/default/grub` has been updated correctly and updated my computer multiple times with several methods. This workaround works for me though:

```
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```

You may need to replug the device after this.

This is a temporary solution, and everytime the computer get restarted, this commands need to be run again. The solution is found from <https://importgeek.wordpress.com/2017/02/26/increase-usbfs-memory-limit-in-ubuntu/>.

### Validate the connection with k4aviewer

First, check that your usb kernal memory has been updated by running `cat /sys/module/usbcore/parameters/usbfs_memory_mb`.

Then check the image. Make sure you start the subordinate device first, and the master device last.

#### Subordinate device

1. start an instance of k4aviewer
2. choose the right Device S/N for the subordinate device
3. click `Open Device`
4. Under External Sync, choose `Sub`
5. click `Start`. You won't see the image until you start the master device.

#### Master device

1. start another instance of k4aviewer
2. choose the right Device S/N for the master device
3. click `Open Device`
4. Under External Sync, choose `Master`
5. click `Start`

If it doesn't work, it could be that the 3.5 mm audio cable is too long. I tested it with a 25 ft long branch new audio cable, but it doesn't work. Although on the website it said the maximum length it can take is 10 m, which is about 32 ft.

### Run multiple devices with ROS

For a reference of the paramters, please refer to <https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/usage.md>.

Create a launch file that could accomodate mutliple devices (in my case, 2 devices). Several things need to be updated from the `driver.launch` file:

1. create another node of `azure_kinect_ros_driver` to start 2 devices. I called one node `azure_kinect_ros_driver_master` and the other `azure_kinect_ros_driver_sub`
2. provide the corresponding `sensor_sn` in each node
3. For the arg `subordinate_delay_off_master_usec`, set the default to `160`. The `subordinate_delay_off_master_usec` of the master node must be set to 0.
4. Update the `wired_sync_mode` in the master node to be 1
5. Update the `wired_sync_mode` in the sub node to be 2
6. Update the `tf_prefix` in both of the master and the sub node to whatever name you like, so that you will be able to distinguish the point cloud from the two device.

The complete launch file looks like this:

```xml
<!--
Copyright (c) Microsoft Corporation. All rights reserved.
Licensed under the MIT License.
-->

<launch>
  <arg name="tf_prefix"         default="" />                       <!-- Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty. -->
  <arg name="master_tf_prefix"         default="master_" />
  <arg name="sub_tf_prefix"         default="sub_" />
  <arg name="overwrite_robot_description" default="true" />         <!-- Flag to publish a standalone azure_description instead of the default robot_descrition parameter-->

  <group if="$(arg overwrite_robot_description)">
    <param name="robot_description"
      command="xacro $(find azure_kinect_ros_driver)/urdf/azure_kinect.urdf.xacro tf_prefix:=$(arg tf_prefix)" />
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
  </group>

  <group unless="$(arg overwrite_robot_description)">
    <param name="azure_description"
      command="xacro $(find azure_kinect_ros_driver)/urdf/azure_kinect.urdf.xacro tf_prefix:=$(arg tf_prefix)/" />
    <node name="joint_state_publisher_azure" pkg="joint_state_publisher" type="joint_state_publisher">
      <remap from="robot_description" to="azure_description" />
    </node>  
    <node name="robot_state_publisher_azure" pkg="robot_state_publisher" type="robot_state_publisher">
      <remap from="robot_description" to="azure_description" />
    </node>
  </group>

  <arg name="depth_enabled"           default="true" />           <!-- Enable or disable the depth camera -->
  <arg name="depth_mode"              default="WFOV_UNBINNED" />  <!-- Set the depth camera mode, which affects FOV, depth range, and camera resolution. See Azure Kinect documentation for full details. Valid options: NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_UNBINNED, WFOV_2X2BINNED, and PASSIVE_IR -->
  <arg name="color_enabled"           default="true" />           <!-- Enable or disable the color camera -->
  <arg name="color_format"            default="bgra" />           <!-- The format of RGB camera. Valid options: bgra, jpeg -->
  <arg name="color_resolution"        default="1536P" />          <!-- Resolution at which to run the color camera. Valid options: 720P, 1080P, 1440P, 1536P, 2160P, 3072P -->
  <arg name="fps"                     default="5" />              <!-- FPS to run both cameras at. Valid options are 5, 15, and 30 -->
  <arg name="point_cloud"             default="true" />           <!-- Generate a point cloud from depth data. Requires depth_enabled -->
  <arg name="rgb_point_cloud"         default="true" />           <!-- Colorize the point cloud using the RBG camera. Requires color_enabled and depth_enabled -->
  <arg name="point_cloud_in_depth_frame" default="false" />        <!-- Whether the RGB pointcloud is rendered in the depth frame (true) or RGB frame (false). Will either match the resolution of the depth camera (true) or the RGB camera (false). -->
  <arg name="required"                default="false" />          <!-- Argument which specified if the entire launch file should terminate if the node dies -->
  <arg name="sensor_sn"               default="" />               <!-- Sensor serial number. If none provided, the first sensor will be selected -->
  <arg name="master_sensor_sn"               default="000228201512" />               <!-- Sensor serial number of the master device. If none provided, the first sensor will be selected -->
  <arg name="sub_sensor_sn"               default="000247101512" />               <!-- Sensor serial number of the subordinate device. If none provided, the first sensor will be selected -->
  <arg name="recording_file"          default="" />               <!-- Absolute path to a mkv recording file which will be used with the playback api instead of opening a device -->
  <arg name="recording_loop_enabled"  default="false" />          <!-- If set to true the recording file will rewind the beginning once end of file is reached -->
  <arg name="body_tracking_enabled"           default="false" />  <!-- If set to true the joint positions will be published as marker arrays -->
  <arg name="body_tracking_smoothing_factor"  default="0.0" />    <!-- Set between 0 for no smoothing and 1 for full smoothing -->
  <arg name="rescale_ir_to_mono8"  default="false" />    <!-- Whether to rescale the IR image to an 8-bit monochrome image for visualization and further processing. A scaling factor (ir_mono8_scaling_factor) is applied. -->
  <arg name="ir_mono8_scaling_factor"  default="1.0" />    <!-- Scaling factor to apply when converting IR to mono8 (see rescale_ir_to_mono8). If using illumination, use the value 0.5-1. If using passive IR, use 10. -->
  <arg name="imu_rate_target" default="0"/>                       <!-- Desired output rate of IMU messages. Set to 0 (default) for full rate (1.6 kHz). --> 
  <arg name="wired_sync_mode" default="0"/>                       <!-- Wired sync mode. 0: OFF, 1: MASTER, 2: SUBORDINATE. --> 
  <arg name="master_wired_sync_mode" default="1"/>                       <!-- Wired sync mode. 0: OFF, 1: MASTER, 2: SUBORDINATE. --> 
  <arg name="sub_wired_sync_mode" default="2"/>                       <!-- Wired sync mode. 0: OFF, 1: MASTER, 2: SUBORDINATE. --> 
  <arg name="subordinate_delay_off_master_usec" default="160"/>     <!-- Delay subordinate camera off master camera by specified amount in usec. --> 

  <node pkg="azure_kinect_ros_driver" type="node" name="azure_kinect_ros_driver_master" output="screen" required="$(arg required)" ns="master">
    <param name="depth_enabled"     type="bool"   value="$(arg depth_enabled)" />
    <param name="depth_mode"        type="string" value="$(arg depth_mode)" />
    <param name="color_enabled"     type="bool"   value="$(arg color_enabled)" />
    <param name="color_format"      type="string" value="$(arg color_format)" />
    <param name="color_resolution"  type="string" value="$(arg color_resolution)" />
    <param name="fps"               type="int"    value="$(arg fps)" />
    <param name="point_cloud"       type="bool"   value="$(arg point_cloud)" />
    <param name="rgb_point_cloud"   type="bool"   value="$(arg rgb_point_cloud)" />
    <param name="point_cloud_in_depth_frame"   type="bool"   value="$(arg point_cloud_in_depth_frame)" />
    <param name="sensor_sn"         type="string" value="$(arg master_sensor_sn)" />
    <param name="tf_prefix"         type="string" value="$(arg master_tf_prefix)" />
    <param name="recording_file"          type="string" value="$(arg recording_file)" />
    <param name="recording_loop_enabled"  type="bool"   value="$(arg recording_loop_enabled)" />
    <param name="body_tracking_enabled"           type="bool"   value="$(arg body_tracking_enabled)" />
    <param name="body_tracking_smoothing_factor"  type="double" value="$(arg body_tracking_smoothing_factor)" />
    <param name="rescale_ir_to_mono8" type="bool" value="$(arg rescale_ir_to_mono8)" />
    <param name="ir_mono8_scaling_factor" type="double" value="$(arg ir_mono8_scaling_factor)" />
    <param name="imu_rate_target" type="int" value="$(arg imu_rate_target)"/>
    <param name="wired_sync_mode" type="int" value="$(arg master_wired_sync_mode)"/>
    <param name="subordinate_delay_off_master_usec" type="int" value="0"/>
  </node>

  <node pkg="azure_kinect_ros_driver" type="node" name="azure_kinect_ros_driver_sub" output="screen" required="$(arg required)" ns="sub">
    <param name="depth_enabled"     type="bool"   value="$(arg depth_enabled)" />
    <param name="depth_mode"        type="string" value="$(arg depth_mode)" />
    <param name="color_enabled"     type="bool"   value="$(arg color_enabled)" />
    <param name="color_format"      type="string" value="$(arg color_format)" />
    <param name="color_resolution"  type="string" value="$(arg color_resolution)" />
    <param name="fps"               type="int"    value="$(arg fps)" />
    <param name="point_cloud"       type="bool"   value="$(arg point_cloud)" />
    <param name="rgb_point_cloud"   type="bool"   value="$(arg rgb_point_cloud)" />
    <param name="point_cloud_in_depth_frame"   type="bool"   value="$(arg point_cloud_in_depth_frame)" />
    <param name="sensor_sn"         type="string" value="$(arg sub_sensor_sn)" />
    <param name="tf_prefix"         type="string" value="$(arg sub_tf_prefix)" />
    <param name="recording_file"          type="string" value="$(arg recording_file)" />
    <param name="recording_loop_enabled"  type="bool"   value="$(arg recording_loop_enabled)" />
    <param name="body_tracking_enabled"           type="bool"   value="$(arg body_tracking_enabled)" />
    <param name="body_tracking_smoothing_factor"  type="double" value="$(arg body_tracking_smoothing_factor)" />
    <param name="rescale_ir_to_mono8" type="bool" value="$(arg rescale_ir_to_mono8)" />
    <param name="ir_mono8_scaling_factor" type="double" value="$(arg ir_mono8_scaling_factor)" />
    <param name="imu_rate_target" type="int" value="$(arg imu_rate_target)"/>
    <param name="wired_sync_mode" type="int" value="$(arg sub_wired_sync_mode)"/>
    <param name="subordinate_delay_off_master_usec" type="int" value="$(arg subordinate_delay_off_master_usec)"/>
  </node>  
</launch>

```

Test it by running `roslaunch azure_kinect_ros_driver multi_device_driver.launch`. If your launch file is saved in another package or with a different name, the commands should be modified accordingly. If you are able to get the point clouds, etc, and the only error is like this:

```bash
[ERROR] [1598993353.580386046]: Failed to open K4A device at index 0
```

You can safely ignore the error as mentioned in <https://github.com/microsoft/Azure_Kinect_ROS_Driver/issues/97>.

## Calibration

~~The factory calibration do not seem to be ideal, if a hand--eye calibration or multi--azure calibration is needed, it may be necessary to recalibrate the device.~~

I took almost 200 images to calibrate the color image (e.g., a straigh line at the edge is curved), but the result does seem to be as good as the factory calibrated result. One thing to note that aruco only uses the plumb_bob distortion model which takes an array of distortion coefficient with size 5. Azure uses a different distortion model which requires an array with size of 9. The last few coefficients cannot be simply chopped off. To work with aruco, use image_proc nodelet to convert the image to rectified images and then feed it to aruco. image_proc can take both plumb_bob (size 5) and the rotational model (size 9). If rectifying the depth image in the depth frame is needed, set interpolatin to 0 according this thread <https://github.com/microsoft/Azure_Kinect_ROS_Driver/issues/103>.

After some investigation, it turned out we are not alone to find out the inaccuracy of the depth perception from ToF device. The inaccuracy of the depth perception is likely due to the materials of the objects. This thread <https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1341> and this thread <https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/803> elaborated on this more. More information about the comparison between cameras using different depth perception techniques can be found here: <https://roscon.ros.org/2017/presentations/ROSCon%202017%203D%20Vision%20Technology.pdf>, and <https://www.revopoint3d.com/comparing-three-prevalent-3d-imaging-technologies-tof-structured-light-and-binocular-stereo-vision/> .

That being said, if you still would like to calibrate the device yourself, the following source code has been tested and could serve the function.

To calibrate the device, please get the code from: <https://github.com/ScazLab/azure_customize_calibration>. The code is adapted from Kinect One's calibration code from: <https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration>, since the two device uses similar techniques for depth perception. Therefore, the calibration instructions are very similar. A few changes are:

1. To specify the board size, use `'*'` instead of `'x'`
2. This is not really a change, but a note. If you launched multiple device, you need to specify which one you would like to calibrate, `master` or `sub`, which is the `ns` specificed in the launch file. If you just launched `driver.launch` without modification, then don't add `master` at the end.
3. New functionality: you can specifiy the diplay image size by setting, for example, colorDispResize=0.65 so that the image is 0.65 of the original size of the color image, and irDispResize=0.5 of the size of the ir image. A number smaller than 1 means a smaller image, and a number larger than 1 means a larger image is wanted.
4. New functionality: a simple mode has been added as the calibration mode. Originally, the intrinsics of the color and ir cameras need images different from the extrinsics calibration (e.g., sync). In the simple mode, the intrinsics of the calibration of the color and ir could use the images from sync, and no longer need additional images. This could make the calibration faster since less images are needed. However, this may lead to worse result. As the color and ir cameras may have different views, the images works for both cameras may lead to biased results for intrinsics calibration (e.g., certain part the color/ir images doesn't have points due to the different views between the two cameras). Therefore, if you need accurate calibration, the simple mode is **NOT** recommended. **WARNING: ONLY USE THE SIMPLE MODE IF YOU UNDERSTAND THE CONSEQUENCES.**

Here are the commands that corresponds to the *Detailed Steps* in the kinect One instruction (e.g., use th original mode).

0. To start the device, use the above code if you followed the this instructions, or just launch the `driver.launch` in Azure_Kinect_ROS_Driver if you only need to start one device.
1. `mkdir ~/kinect_cal_data; cd ~/kinect_cal_data`
2. `rosrun azure_calibration azure_calibration chess9*11*0.02 colorDispResize=0.65 record color master`.
3. `rosrun azure_calibration azure_calibration chess9*11*0.02 calibrate color`
4. `rosrun azure_calibration azure_calibration chess9*11*0.02 irDispResize=0.5 record ir master`
5. `rosrun azure_calibration azure_calibration chess9*11*0.02 calibrate ir`
6. `rosrun azure_calibration azure_calibration chess9*11*0.02 colorDispResize=0.65 irDispResize=0.5 record sync master`
7. `rosrun azure_calibration azure_calibration chess9*11*0.02 calibrate sync`
8. `rosrun azure_calibration azure_calibration chess9*11*0.02 calibrate depth`

Here are the steps if you wish to use the simple mode:

0. To start the device, use the above code if you followed the this instructions, or just launch the `driver.launch` in Azure_Kinect_ROS_Driver if you only need to start one device.
1. `mkdir ~/kinect_cal_data; cd ~/kinect_cal_data`
2. `rosrun azure_calibration azure_calibration chess9*11*0.02 colorDispResize=0.65 irDispResize=0.5 record sync master`
3. `rosrun azure_calibration azure_calibration chess9*11*0.02 calibrate simple`. It will first calibrate the color camera, then the ir camera, then the extrinsics between the two, and then calibrate depth.

Other tips I found useful for calibration:
- <https://stackoverflow.com/questions/12794876/how-to-verify-the-correctness-of-calibration-of-a-webcam/12821056#12821056>
- <https://stackoverflow.com/questions/60553097/opencv-calibratecamera-function-yielding-bad-results>

## References

Azure documentation: <https://azure.microsoft.com/en-us/services/kinect-dk/>

## Collaboration

- **Please edit this file** to add things you think are going to be useful. You can edit it by modifying [this file](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2020-08-31-KinectAzure.md)

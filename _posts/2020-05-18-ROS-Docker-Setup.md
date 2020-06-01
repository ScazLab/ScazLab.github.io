---
layout: post
title: ROS Docker Setup
authors: Debasmita Ghose
description: "Setup Guide for setting up a docker with ROS"
tags: 
categories: [wiki]
comments: false
permalink: ros_docker_setup.html
excerpt_separator: <!-- More -->
---

Welcome to the Setup Guide for ROS in Docker. 

<!-- More -->


# Install Docker
 
- Create an account on [DockerHub](https://hub.docker.com/)
- Follow instructions [here](https://docs.docker.com/get-docker/) to install docker for your Operating System. 

# General ROS Docker Commands

## One Time Setup

- Pull the ROS docker image by running (one time setup):  
`docker pull ros`

- In your local machine clone the repository using the command:  
`git clone <repository https: path>`

- Spin up a container with the git repository mounted using the command:  
`docker run -v <path_to_repository_in_your_local_machine>:/code/  -it ros`

## Instructions for using the same container every time

- Inside the container run `roscore`
 
- On a new terminal, list all the available containers using the command (The first 3 characters of the container ID or the name of the container listed in the last column can be used as the `container_name`):  
`docker ps`

- If there is a container found, it means that there is a container available and it is running. In order to enter that particular container use the command:   
`docker exec -it <container_name> bash`

- If there is no container found after running `docker ps`, run `docker ps -a`. If there is a container found, it means that the container exists but is not running. In order to enter into the container run:  
`docker start <container_name>`   
`docker exec -it <container_name> bash`
This will start an additional bash session in the same container.    

- Once inside the container, source the `setup.bash` file using:  
`source /opt/ros/melodic/setup.bash`

- To test if the instance is connected to the same `ros-master` run:  
`rostopic list`

# Commands Specific to the Watch One, Do One, Teach One Project

## One Time Setup

- Download the Dockerfile located [here](https://github.com/ScazLab/wodoto/blob/master/Docker_Files/Dockerfile)

- Navigate to the location where the Dockerfile is saved on your local machine and build the docker using:  
`sudo docker build -t ros:melodic-desktop-full .`

- Test if the docker was built using:  
`sudo docker images`  
If the docker was built, the output of this command would look like [this]("https://github.com/ScazLab/ScazLab.github.io/blob/master/images/sudo_docker_images.png") :  

 
 - Obtain the hostname for your system by running: `hostnamectl`
 
 - Pull the [wodoto repository](https://github.com/ScazLab/wodoto) in your local machine if you have not done so already. 
 
 - Spin up a container with the wodoto repository mounted using the command (Replace `path-to-repository` and `hostname` appropriately):  
 `sudo docker run -it -v ~/.ssh:/root/.ssh -v <path-to-wodoto-repository>:/root/catkin_ws --network host --env ROS_MASTER_URI=http://<hostname>:11311 --name ros ros:melodic-desktop-full`


To edit this document make changes [here](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2020-05-18-ROS-Docker-Setup.md)



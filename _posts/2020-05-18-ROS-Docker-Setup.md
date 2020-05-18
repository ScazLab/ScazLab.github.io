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
Follow instructions [here](https://docs.docker.com/get-docker/) to install docker for your Operating System. 

# Steps for Setting up ROS inside the Docker

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



To edit this document make changes [here](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2020-05-18-ROS-Docker-Setup.md)



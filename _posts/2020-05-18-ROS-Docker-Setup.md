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


# Docker Setup

## Install Docker
Follow instructions [here](https://docs.docker.com/get-docker/) to install docker for your Operating System. 

## Steps for Setting up ROS inside the Docker

- Pull the ROS docker image by running (one time setup):  
`docker pull ros`

- Spin up a container using the command:  
`docker run -it ros`

- Inside the container run `roscore`
 
- On a new terminal, list the available containers using the command:  
`docker ps -l`

- Find the name of the container (use either the first 3 characters of the container ID or the name of the container listed in the last column)

- Start an additional bash session in the same container by running using the container name from the previous step:   
`docker exec -it <container_name> bash`

- Once inside the container, source the `setup.bash` file using:  
`source /opt/ros/melodic/setup.bash`

- To test if the instance is connected to the same `ros-master` run:  
`rostopic list`



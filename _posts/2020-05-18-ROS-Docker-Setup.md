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


# Commands Specific to the Watch One, Do One, Teach One Project

## One Time Setup

- Download the Dockerfile located [here](https://github.com/ScazLab/wodoto/blob/master/Docker_Files/Dockerfile)

- Navigate to the location where the Dockerfile is saved on your local machine and build the docker using:  
`sudo docker build -t ros:melodic-desktop-full .`

- Test if the docker was built using:  
`sudo docker images`  
If the docker was built, the output of this command would look like [this](https://github.com/ScazLab/ScazLab.github.io/blob/master/images/sudo_docker_images.png)

 
 - Obtain the hostname for your system by running: `hostnamectl`
 
 - Pull the [wodoto repository](https://github.com/ScazLab/wodoto) in your local machine if you have not done so already. 
 
 - Spin up a container with the wodoto repository mounted using the command (Replace `path-to-repository` and `hostname` appropriately):  
 `sudo docker run -it -v ~/.ssh:/root/.ssh -v <path-to-wodoto-repository>:/root/catkin_ws --network host --env ROS_MASTER_URI=http://<hostname>:11311 --name ros ros:melodic-desktop-full`
 
 - Add the [following lines](https://github.com/ScazLab/wodoto/blob/master/Docker_Files/.bash_profile) to your `~/.bash_profile` file on your local machine replacing `path-to-repository` and `hostname` appropriately. Then source it by running:  
 `source ~/.bash_profile`
 
- Install `pip` inside the container by following [these](https://pip.pypa.io/en/stable/installing/) instructions.

- Install `vim` inside the container by following [these](https://phoenixnap.com/kb/how-to-install-vim-ubuntu) instructions.


## Commands to Start and Test the ROS Docker for the First Time

- Run the ROS Docker Container created in the previous step by:  
`sudo docker start ros`  
`docker exec -it ros bash`  
After running this, we would have entered inside the container, and the prompt will change to `root@<hostname>:/# `

- Inside the container source the `setup.sh` file by running:  
`source /opt/ros/melodic/setup.sh`

- Check if the `catkin_ws` is present inside the `root` directory inside the container. 

- The code files should be present inside `root/catkin_ws/` and the ROS packages should be present inside `root/catkin_ws/src` 

- Run `catkin_make` inside `root/catkin_ws`. After `catkin_make`, check if `build` and `devel` directories are created inside the `catkin_ws`. Then run:  
`source devel/setup.bash`

- Run the following series of commands to test if we have a running ROS installation and can publish on a topic from within the container.  
`roscore &`  
`:`  
`rostopic pub -r 1 /test std_msgs/String "test" &`  
`:`  
These commands mean that we first run `roscore` in the background, and publish a `test` message on the `/test` topic and send that process to the background. 

- In order to check if the message was published on the topic run:  
`rostopic list`  
The topic `/test` should appear on the list. 

- To verify if the `test` message is being published on the `/test` topic run:  
`rostopic echo /test`


## Instructions for using the same container every time and running code

- List all the available containers using the command (The first 3 characters of the container ID or the name of the container listed in the last column can be used as the `container_name`):  
`docker ps`

- If there is a container found, it means that there is a container available and it is running. In order to enter that particular container use the command:   
`docker exec -it ros bash`

- If there is no container found after running `docker ps`, run `docker ps -a`. If there is a container found, it means that the container exists but is not running. In order to enter into the container run:  
`docker start ros`   
`docker exec -it ros bash`  
 The prompt will change to `root@<hostname>:/# ` once we are inside the container. 

- Once inside the container, source the `setup.bash` file using:  
`source /opt/ros/melodic/setup.bash`

- In order to run ROS nodes, first run `roscore &` and send it to the background by running `:`. To spin up the required node, use the appropriate `rosrun` or `roslaunch` command as we do on the robot. 

## Shortcuts for Working with the Docker

Some aliases have been added on the `.bash_profile` file, to make working with this particular docker easier. To refer to what each alias does, see [here](https://github.com/ScazLab/wodoto/blob/master/Docker_Files/.bash_profile)

They are described here:

- `build_ros_docker` - build the ROS docker (one-time)
- `run_docker` - run the docker with the code mounted to create a container (one-time)
- `list_images` - list docker images (one-time)
- `list_container` - list running containers
- `list_all_container` - list all running as well as stopped containers
- `exec_docker` - execute a running container


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
`source /opt/ros/melodic/setup.bash` and inside the `catkin_ws` run `source devel/setup.bash`

- To test if the instance is connected to the same `ros-master` run:  
`rostopic list`

To edit this document make changes [here](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2020-05-18-ROS-Docker-Setup.md)



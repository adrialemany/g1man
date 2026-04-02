# G1 Unitree Robot Manipulation (and Navigation)

This repository contains a workspace for the robot, a docker for proper use and some scripts for using its sensors and motors. 

## What you can find
- The main directory is the workspace for ROS2 itself.
- Then we have some subdirectories, such as: 
    - camera: its main purpose of accessing a easy-2-use client and server. It's purpose is to help the Korean students access the cameras and robot movement within certain inputs that can be sent from other clients (not only mine).
    - manipulation: where the manipulation scripts are. Names are quite self-explanatory.
    - fastlio2_exp: here we can find the scripts I used for generating a pointcloud of the building while also recording both cameras on ROS2 topics. Will soon add the terminal commands for recording video properly with audio to its readme.
    - mujoco: already working simulation using Holosoma FastSAC Locomotion Policy for leg movement and some scripts I used for controlling the simulated robot and visualizing its camera.
    - g1pilot_exp subdirectory serves as a way to explaining the process on how to use g1pilot via WiFi, not relying on being connected to a physical wire.

## How I work on the robot
Mainly, I execute my own scripts on Docker. The ~/.bashrc file has some alias:
```bash
docker_humble
```
It opens the docker container.
If you want to open another terminal inside the container, use:
```bash
docker_open
```
If you update the container's dependences, use:
```bash
docker_commit
```
So you don't have to install the same dependencies again.

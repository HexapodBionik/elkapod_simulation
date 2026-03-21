# Elkapod simulation repository
![ROS2 distro](https://img.shields.io/badge/ros--version-jazzy-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Python Version](https://img.shields.io/badge/python-3.12-g.svg)

## About
This repository contains ROS2 packages used for simulation of Elkapod walking robot. Currently supported simulators are:
- **Gazebo Harmonic** [8.x.x] (Primary)

> [!IMPORTANT]
>
> This repository is a component of the **Elkapod robot stack** It is designed to be used within a workspace managed by [elkapod_stack](https://github.com/HexapodBionik/elkapod_stack).
> To set up the full simulation environment (including the robot description and controllers), follow the instructions from the stack repository.
>

> [!CAUTION]
>
> This repository does not contain the URDF/Xacro files. It relies on the `elkapod_description` package found in `elkapod_core` repository. Ensure both are in your colcon workspace before building.
>

## Prerequisites for Gazebo sim
Before running the simulator check if those environment variables are set
```bash
export GZ_SIM_RENDER_ENGINE=ogre2
export GZ_PARTITION=elkapod_sim
export GZ_IP=127.0.0.1
```

Even though Gazebo can be used without GPU acceleration it is highly recommended to run it on PC with NVIDIA GPU. If it's not an option for you then you can run the simulation in **headless** mode with RViz only.

## Avaliable worlds
Currently 4 worlds are avaliable and are specified using **world** launch argument. <br>
```
world:={empty | bookstore | small_house | warehouse}
```

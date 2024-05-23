# Elkapod teleoperation repository
![ROS2 distro](https://img.shields.io/badge/ros--version-humble-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Python Version](https://img.shields.io/badge/python-3.10-g.svg)

## Installation
1. Create a workspace and clone packages into it
```bash
mkdir -p elkapod_sim/src/
git clone https://github.com/HexapodBionik/elkapod_sim.git elkapod_sim/src/
```
2. Move into `src/` folder and download all additional packages using [vcstool](http://wiki.ros.org/vcstool)
```bash
cd elkapod_sim/src/
vcs import . < repos.yaml
```
> [!IMPORTANT]
> 
> Additionally, you need to install the ElkapodAlgorithms Python package. For more instructions, visit the package's [website](https://github.com/HexapodBionik/ElkapodAlgorithms.git).
>

## How to run the simulation?
First of all you have to run launch from the `elkapod_core_bringup`.

```bash
ros2 elkapod_core_bringup elkapod_core_bringup.launch.py sim:=True
```

## How to control the robot?
Create separate workspace and follow instructions from [ElkapodTeleop](https://github.com/HexapodBionik/elkapod_teleop.git).

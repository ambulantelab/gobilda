# This repository contains the ROS2 Driver for our Gobilda robot setup in class CPE416 
# ROS2 Driver and Jetson Orin Nano Setup

## Prerequisites (tested on following hardware)

- Jetson Orin Nano Super Dev Kit
- Gobilda Starter Kit
- WiFi connectivity, Ethernet connectivity
- A1 RP LiDAR
- Oak-d Pro Camera
- Ethernet, USB Cables, Power Cables
- 12V Gobilda Battery

## Dependencies

Before you begin, ensure you have met the following requirements:

- ROS2 Humble Installation
- Ubuntu 22.04.5 LTS
- Jetpack 6.0

## Setup Guide Gobilda, ROS2, Jetson Orin Nano

### Connection to the internet
Hey! You cannot clone this repo without being connected to the internet. We have a couple options for our platforms, but the easiest will be to use VNC to open up a desktop viewer on the Orin and then use the Network GUI to connect to a network. A good option for now will be to connect to calpoly's 'eduroam' network.

### Clone the Repository
Once connected to the internet you can clone this repo with the following command:

```bash
git clone --recursive https://github.com/ambulantelab/gobilda.git
```
### Compiling the Driver
#### 1. Update CMake
After you can cloned the repo, we can try building the driver. Because the default cmake version installed does not work for the driver, we'll first need to update our cmake, we can use the provided script (env_scripts/update_cmake.sh) to do that:
```bash
sudo bash update_cmake.sh
```
#### 2. Install ROS2 dependencies
Next we should install all the dependencies that are required for the driver. Run the following commands at the workspace level (/gobilda_ws/):
```bash
# Update rosdep database
rosdep update

# Install dependencies for a specific package
rosdep install --from-paths src --ignore-src -r -y
```

#### 3. Compilation
To compile the worksapce run
```bash
# ROS2 Compilation Command
colcon build --symlink-install
```

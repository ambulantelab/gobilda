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

### Setup the permissions, udev rules, and pwm groups
Before you can use the ros2 driver successfully, there are a number of permission and udev rules that need to be changed.
#### 1. jetson-io.py
We first need to set the pins 15 & 32 to on the dev kit's 40-pin expansion board to output PWM signals. Fortunately, NVIDIA provides a terminal tool for generating the required files. Run the following command:
```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```
Select the following options: Configure Jetson 40pin Header -> Configure header pins manually -> pwm1 & pwm7 -> Save pin changes -> Save and exit without rebooting
(We will reboot the Orin in a moment)

#### 2. Create 'pwm' group and write udev rules
Then, we need to run the follwing script (in the /env_scripts/ directory) which will copy the udev rules to the corresponding directory and create a group 'pwm' that will allow our user to send pwm commands without sudo permissions:
```bash
sudo bash jetson_orin.sh
```
After running the above command we can reboot the Orin Nano:
```bash
sudo shutdown now
```

#### 3. Compile and test the 'simple_pwm.cpp'
Finally, to make sure that everything is wired correctly and that we can send pwm signals, let's compile and run the simple_pwm code:
To compile the file and run the executable:
```bash
# Compile code
g++ simple_pwm.cpp -o simple_pwm

# Run the executable
./simple_pwm
```
When you run the executable you should see output and soon the wheels of your Gobilda robot should start moving!
After, confirming that this code works we can move to compiling the ROS2 workspace and running more advanced software techniques.

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
This command should have installed many packages that are required for the driver to work correctly.

#### 3. Compilation
To compile the worksapce run
```bash
# ROS2 Compilation Command
colcon build --symlink-install
```
Workspace should compile with no, issues. (If you see a Cmake warning output you can safely ignore that.)

#### 4. Source Overlap, Underlay, and ROS2 Launch Files

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

### 1. Connection to the internet
Hey! You cannot clone this repo without being connected to the internet. We have a couple options for our platforms, but the easiest will be to use VNC to open up a desktop viewer on the Orin and then use the Network GUI to connect to a network. A good option for now will be to connect to calpoly's 'eduroam' network.

### 2. Clone the Repository
Once connected to the internet you can clone this repo with the following command:

```bash
git clone --recursive https://github.com/ambulantelab/gobilda.git
```

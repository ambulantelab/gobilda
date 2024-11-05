# gobila_ws
Code for the GoBilda ROS2 Driver (work in progress)

To run the driver there are a couple of steps that are necessary to perform

1) Set the pins 15, & 32 on the Jetson's 40-pin header to PWM
2) Compile and install the Rubberazer/JETGPIO library (git submodule)
3) Copmile and source the 'gobilda_ws'
4) To interface with the JETGPIO library and use the funcs in the ros2_control_node you will need to be root. "sudo -i"
5) Run the launch file "ros2 launch gobilda_robot gobilda.launch.py". NOTE: because the launch file was ran using root, the nodes can only communicate with other root nodes. You will need to be root again to run for example "ros2 topic list"

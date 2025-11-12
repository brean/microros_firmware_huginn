# MicroROS firmware for for RaspberryPi
MicroROS Firmware based on FreeRTOS for RaspberryPi Pico 2 for a small robot.

Based on the [MicroROS Raspberrypi Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)

## Features
 - control ackermann steering of servo with PWM
 - control brushless ESP for motor with PPM (and configure it)

## Usage
Build the devContainer as asked by Visual Studio Code when opening this project.
Inside the devContainer you can open a terminal and build the project:

```bash
# Run this on a terminal inside the devContainer
cd /ws/micro_ros_raspberrypi_pico_sdk
mkdir build
cd build
cmake -DPICO_BOARD=pico2 ..
make
```

Note that we overwrite the `CMakeLists.txt` and `main.c` files in the micro_ros_raspberrypi_pico_sdk folder and have our own files in the src-subfolder.
You might want to fork the whole [MicroROS Raspberrypi Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) for a more complex setup.

## About the name
Named after one of Odin's All-seeing Ravens Huginn (which translates to thought), as this little robot is just the cheapest way to implement a prototype.

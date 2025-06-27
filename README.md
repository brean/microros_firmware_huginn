# MicroROS firmware for huginn
MicroROS Firmware based on FreeRTOS for RaspberryPi Pico 2 for a small robot.

## Features
 - control ackermann steering of servo with PWM
 - control brushless motor with PPM

## Usage
Build the devContainer as asked by Visual Studio Code when opening this project.
Inside the devContainer you can open a terminal and build the project:

```bash
cd /ws/micro_ros_raspberrypi_pico_sdk
mkdir build
cd build
cmake -DPICO_BOARD=pico2 ..
make
```

Note that we only copy the `CMakeLists.txt` and `main.c` files over to the docker container, you maybe want to fork the whole `micro_ros_raspberrypi_pico_sdk` and just mount that as volume inside your devContainer.

## About the name
Named after one of Odin's All-seeing Ravens Huginn (which translates to thought), as this little robot is just the cheapest way to implement a prototype.
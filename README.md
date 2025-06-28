# MicroROS firmware for huginn
MicroROS Firmware based on FreeRTOS for RaspberryPi Pico 2 for a small robot.

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

FIXME: Note that we overwrite the `CMakeLists.txt` and `main.c` files in the raspberrypi-pico-sdk and have our own files in the src-subfolder.
However it is probably better to call `ros2 run micro_ros_setup build_firmware.sh` which might fix the problem with adding ros2 services

## About the name
Named after one of Odin's All-seeing Ravens Huginn (which translates to thought), as this little robot is just the cheapest way to implement a prototype.
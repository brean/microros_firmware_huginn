#!/bin/bash
# run this inside the dev container!
cd /ws/micro_ros_raspberrypi_pico_sdk
# cleanup: rm -Rf build/*
cd build
cmake -DPICO_BOARD=pico2 ..
make
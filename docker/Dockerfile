ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

COPY ./docker/entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]

RUN apt-get update \
    && apt-get install -y \
        build-essential \
        cmake \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        git\
    && rm -rf /var/lib/apt/lists/*


WORKDIR /opt

# Clone PICO SDK into workspace
RUN git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git
ENV PICO_SDK_PATH=/opt/pico-sdk

# Clone microros for raspi-pico-sdk
RUN git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git


ENV ROS_WORKSPACE=/ws
RUN mkdir -p ${ROS_WORKSPACE}/src/
WORKDIR ${ROS_WORKSPACE}

# Clone setup for micro-ros/micro-ros-agent into src
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && apt-get update \
    && rosdep update \
    && colcon build

COPY README.md ${ROS_WORKSPACE}/src/README.md

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

CMD ["/bin/bash"]
FROM nvidiajetson/l4t-ros2-foxy-pytorch:r32.5

# Essentials
RUN apt update && apt upgrade -y && rosdep update && \
    apt install -y nano rsync git tio gdb

# ROS 2
WORKDIR /root/ros2_ws/
SHELL [ "/bin/bash", "-c" ]
RUN mkdir src/

# Grab the driver codes
RUN cd src && \
    git clone -b main https://github.com/Triton-AI/transport_drivers && \
    git clone -b ros2 https://github.com/Triton-AI/vesc && \
    git clone -b ros2 https://github.com/ros-drivers/joystick_drivers && \
    git clone https://github.com/SICKAG/sick_scan2.git

# Install dependencies
RUN rosdep install --from-paths src -i -y


# Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Done
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc
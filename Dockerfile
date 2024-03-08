# Start from Ubuntu 16.04 base image
FROM ubuntu:16.04

# Install necessary packages including apt-transport-https to allow the use of repository accessed via the HTTP Secure protocol.
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    software-properties-common \
    apt-transport-https \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Add ROS repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Import ROS key
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Kinetic
RUN apt-get update && apt-get install -y ros-kinetic-desktop-full

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup environment variables
ENV ROS_LANG_DISABLE=genlisp:geneus:gennodejs
ENV ROS_PACKAGE_PATH=/opt/ros/kinetic/share
ENV LD_LIBRARY_PATH=/opt/ros/kinetic/lib
ENV PATH=/opt/ros/kinetic/bin:$PATH

# Create and set the entrypoint script
RUN echo '#!/bin/bash\n\
# Source ROS setup for this session\n\
source /opt/ros/kinetic/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

# Fix for importing GPG key for Kitware's APT repository and installing CMake
RUN apt-get update && \
    apt-get install -y --no-install-recommends gnupg-agent && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" > /etc/apt/sources.list.d/kitware.list && \
    apt-get update && \
    apt-get install -y cmake

# Install git, build-essential for SOEM compilation
RUN apt-get update && apt-get install -y \
    git \
    build-essential

# Clone, build, and install SOEM
RUN cd /tmp && \
    git clone https://github.com/OpenEtherCATsociety/SOEM.git && \
    cd SOEM && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make && \
    make install

# Install python-pip, upgrade it, install python development package, and pymodbus
RUN apt-get update && apt-get install -y python-pip python-dev && \
    pip install "pip<21.0" && \
    pip install "pymodbus<3.0"


# Cleanup the package list to reduce image size
RUN rm -rf /var/lib/apt/lists/*

ENV soem_DIR=/usr/local/lib/cmake/soem


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

FROM osrf/ros:humble-desktop-full

# install ubuntu packages
RUN apt update && apt install -y \
        chrony \
        curl \
        git \
        sudo \
        ssh \
        tmux \
        vim \
        xterm \
        wget \
        lsb-release \
        gnupg

# Add sources for gazebo and install (from: https://gazebosim.org/docs/garden/install_ubuntu)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y gz-garden 

# install ros packages
# note that the "ign" bit refers to ignition, the old name for gazebo
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
        ros-humble-ros-ign 

ENV HOME /home/mines
WORKDIR "/home/mines/mines_ws"

ENV DISPLAY=host.docker.internal:0.0

CMD ["/bin/bash"]
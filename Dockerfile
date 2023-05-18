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

# add gazebo 11 sources (Gazebo Classic, NOT Ignition/Garden)
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# install ros packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
        gazebo \
        libgazebo-dev\
        ros-humble-turtlebot3 \
        ros-humble-turtlebot3-msgs \
        ros-humble-turtlebot3-simulations

ENV HOME /home/mines
WORKDIR "/home/mines/mines_ws"
ENV DISPLAY=host.docker.internal:0.0

# Copy in the bashrc file for convenience functions
COPY .bashrc /home/mines/.bashrc

CMD ["/bin/bash"]
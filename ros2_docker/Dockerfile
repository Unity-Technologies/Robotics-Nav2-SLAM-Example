# adapted from: https://github.com/Tiryoh/docker-ros2-desktop-vnc
FROM dorowu/ubuntu-desktop-lxde-vnc:focal
LABEL maintainer="Unity Robotics <unity-robotics@unity3d.com>"

ENV DEBIAN_FRONTEND noninteractive
ENV DEV_NAME=rosdev
ENV ROS_DISTRO=galactic
ENV GROUP_NAME=ros
ENV WS_NAME=colcon_ws

RUN wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | apt-key add -

RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq \
        wget \
        curl \
        git \
        build-essential \
        vim \
        sudo \
        gnupg2 \
        lsb-release \
        locales \
        bash-completion \
        tzdata \
        gosu \
        python3-argcomplete \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install the immutable components BEFORE we copy in build context stuff to keep the rebuild process manageable
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
        /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update -q && \
    apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      python3-rosdep \
      python3-colcon-common-extensions \
      python3-vcstool \
    && rm -rf /var/lib/apt/lists/* && rm /etc/apt/sources.list.d/ros2-latest.list

RUN useradd --create-home --home-dir /home/${DEV_NAME} --shell /bin/bash --user-group --groups adm,sudo ${DEV_NAME} && \
    echo "$DEV_NAME:$DEV_NAME" | chpasswd && \
    echo "$DEV_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
COPY --chown=${DEV_NAME} colcon_ws /home/${DEV_NAME}/colcon_ws
COPY ros2-setup.bash /bin/ros2-setup.bash
# Doing a second fetch of sources & apt-get update here, because these ones depend on the state of the build context
# in our repo
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
        /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update -q && \
    rosdep init && \
    chmod +x /bin/ros2-setup.bash && \
    gosu ${DEV_NAME} /bin/ros2-setup.bash && \
    runuser -u ${DEV_NAME} ros2-setup.bash && \
    rm /bin/ros2-setup.bash && \
    rm -rf /var/lib/apt/lists/* && rm /etc/apt/sources.list.d/ros2-latest.list

RUN echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${DEV_NAME}/.bashrc && \
    echo ". /home/${DEV_NAME}/${WS_NAME}/install/local_setup.bash" >> /home/${DEV_NAME}/.bashrc

ENV TURTLEBOT3_MODEL=waffle_pi

# To bring up tb3 simulation example (from https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)
# cd catkin_ws && source install/setup.bash && ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True

# Informs the environment that the default user is not root, but instead DEV_NAME
ENV USER ${DEV_NAME}


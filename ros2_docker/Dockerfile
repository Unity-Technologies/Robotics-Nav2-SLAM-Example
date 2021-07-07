# NOTE: This is an incomplete dev environment and isn't configured for IDE integration yet
FROM ros:foxy-ros-core

ENV DEV_NAME=rosdev
# This is probably redundant...
ENV ROS_DISTRO=foxy
ENV GROUP_NAME=ros
ENV WS_NAME=colcon_ws


# Ensure dependencies we need are installed, even though they're likely in the base image
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        curl \
        wget \
        vim \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

ARG APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1
# Install the "immutable" dependencies here, before we copy the build context in (so this step can be cached)
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep
# TODO: Re-instate rm statement here
#    && rm -rf /var/lib/apt/lists/* && rm /etc/apt/sources.list.d/ros2-latest.list

RUN python3 -m pip install \
        ipython \
        argcomplete

# NOTE: The sudo password is set to match the username
RUN groupadd ${GROUP_NAME} && \
    useradd -g ${GROUP_NAME} -G sudo --create-home --shell /bin/bash $DEV_NAME && \
    echo "$DEV_NAME:$DEV_NAME" | chpasswd 

COPY --chown=${DEV_NAME}:${GROUP_NAME} colcon_ws /home/${DEV_NAME}/colcon_ws
COPY --chown=${DEV_NAME}:${GROUP_NAME} launch_example /usr/local/bin/

# Have to re-add all the ROS apt bloat to enable the rosdep install...
RUN . /opt/ros/foxy/setup.sh && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
       /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    cd /home/${DEV_NAME}/${WS_NAME} && \
    rosdep init && rosdep update && \
    rosdep install -yrq --from-paths src --ignore-src --rosdistro foxy && \
    rm -rf /var/lib/apt/lists/* && rm /etc/apt/sources.list.d/ros2-latest.list && \
    chmod +x /usr/local/bin/launch_example

USER ${DEV_NAME}
RUN cd ~/${WS_NAME} && colcon build

RUN echo ". /opt/ros/foxy/setup.bash" >> /home/${DEV_NAME}/.bashrc && \
    echo ". /home/${DEV_NAME}/${WS_NAME}/install/local_setup.bash" >> /home/${DEV_NAME}/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/${DEV_NAME}/${WS_NAME}/src/turtlebot3_simulations/turtlebot3_gazebo/models" >> /home/${DEV_NAME}/.bashrc

ENV TURTLEBOT3_MODEL=waffle_pi

# To bring up tb3 simulation example (from https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)
# cd catkin_ws && source install/setup.bash && ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True

USER ${DEV_NAME}
WORKDIR /home/${DEV_NAME}/${WS_NAME}
ENTRYPOINT ["/usr/local/bin/launch_example"]

# AMR Demo: ROS 2 Integration, Section 3: ROS 2 Setup

In the [Scene Setup section](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/anthony/demo/tutorial/scene_setup.md), we set up our warehouse and the virtual robot in Unity. Now it is time to get the robot software based on ROS 2 running. 

There are two options for running the ROS 2 software. Users who want to run ROS 2 on their local machine can install ROS 2 and follow the steps below. We have also provided a Docker container that is setup for running ROS 2 fully contained in Docker.

## Using ROS 2 with Docker
This projects uses [RViz](https://github.com/ros-visualization/rviz), which is a robotics visualization tool that allows the user to not only see what the robot is seeing but, in this example, also allows the user to assign navigation goals to the robot. In order to launch Rviz through Docker, X11 forwarding is needed.

### Install an X11 host
How exactly you set up your X11 host will depend on your operating system and is to some extent a matter of personal preference. In order to render the RViz display from inside the Docker container, you will need an X11 host appropriately configured to receive networked connection requests from the loopback interface (`127.0.0.1`) at a minimum. There are several guides available online for setting up X11 hosts with Docker for different environments. You may also need to  **configure firewall settings or other network security software to allow communication between Docker and your host machine**. We've installed mesa-utils in the image for convenience. To confirm your x11 forwarding is working, try the following command once your docker build completes:
```
docker run -it --rm -p 10000:10000 -p 5005:5005 -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 --entrypoint=/usr/bin/timeout unity-robotics:nav2-slam-example 10 glxgears

```
If configured correctly, you should see a window with three moving gears for 10 seconds, or however long you specify at the end of the previous  `docker run`  command.

![GLXGears](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/anthony/demo/tutorial/images/3-glxgears.gif?raw=true)

There are many potential pitfalls when doing this step; most will have solutions available via StackOverflow or other troubleshooting forums. However, if you can't find an adequate solution to a problem you encounter, feel free to file an Issue ticket here and we may be able to help you troubleshoot further.

### Start the Docker Container
Start the Docker container using:
```
docker run -it --rm -p 10000:10000 -p 5005:5005 -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 unity-robotics:nav2-slam-example
```
> If you want more control over the ROS2 components in the projects, simply add  `--entrypoint=/bin/bash`  to the above  `docker run`  command to start a bash shell rather than immediately launching everything.

## Using ROS 2 on your local machine
Running ROS 2 on your local machine is also an option. The exact steps will depend on your operating system, but the ones below are used with Ubuntu 20.04 and ROS 2 Foxy.

 1. The ROS 2 workspace can be found in `ros2_docker/colcon_ws`. Go into this directory and run the following commands to get all dependencies:
	 ```
	 rosdep init && rosdep update
	 ```
	 ```
	 rosdep install -yrq --from-paths src --ignore-src --rosdistro foxy
	 ```
	 ```
	 rm -rf /var/lib/apt/lists/ /etc/apt/sources.list.d/ros2-latest.list
	 ```
	 ```
	 chmod +x /usr/local/bin/launch_example
	```
 2. Build the project in the same directory and source the environment
	 ```
	 colcon build
	 ```
	 ```
	 src install/local_setup.bash
	 ```
 3. You can now launch the ROS 2 software
	 ```
	 ros2 launch unity_slam_example unity_slam_example.py
	 ```
## Verify ROS 2 is running
If everything is configured correctly, you should see an RViz window pop up that looks like this: 

![RViz Window](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/anthony/demo/tutorial/images/rviz_blank.png)

You may also see the following message being spammed in the console:
```
[controller_server-4] [INFO] [1626978106.918019100] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "base_link" passed to canTransform argument source_frame - frame does not exist
```
This is normal, as the transform frames will be sent from Unity, which hasn't started yet.

Next we will [perform autonomous navigation](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/anthony/demo/tutorial/autonomous_navigation.md).

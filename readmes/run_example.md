# Running the Nav2 + slam_toolbox Example

**Table of Contents**
- [Start RViz in the Docker container](#start-rviz-in-the-docker-container)
    - [(Optional) Launching ROS2 components manually](#optional-launching-ros2-components-manually)
- [Start the Unity simulation](#start-the-unity-simulation)
- [Interacting with this Example](#interacting-with-this-example)
- [Getting Started with Visualizations](#getting-started-with-visualizations)
- [Troubleshooting](#troubleshooting)

---

## Start RViz in the Docker container
>This section assumes you have already set up your environment according to the guide [here](dev_env_setup.md)

- In a terminal on your host OS, run the following:

    ```
    docker run -it --rm -p 6080:80 -p 10000:10000 --shm-size=1024m unity-robotics:nav2-slam-example
    ```

- In a web browser connect to [http://127.0.0.1:6080](http://127.0.0.1:6080) and follow the steps below:

    - Click on the bottom left system menu and select `System Tools > LXTerminal`
      ![connecting to the docker container](images/start_docker_vnc.png)

    - In the Terminal run:
      ```
      ros2 launch unity_slam_example unity_slam_example.py
      ```
- If everything is configured correctly, you should see an RViz window open that looks like this:
  
  
![Blank, pre-configured RViz window](images/rviz_blank.png)

You may also see the following message being spammed in the console:
```
[controller_server-4] [INFO] [1626978106.918019100] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "base_link" passed to canTransform argument source_frame - frame does not exist
```
This is normal, as the transform frames will be sent from Unity, which hasn't started yet.

### (Optional) Launching ROS2 components manually
- If you are managing your own ROS2 environment, you can launch the example manually by using:
    ```
    ros2 launch unity_slam_example unity_slam_example.py
    ```
If this doesn't work, you may need to first `colcon build` the workspace or `source install/local_setup.bash` to properly populate your ROS2 paths.


---

## Start the Unity simulation
>This section assumes you have already set up your Unity environment according to the guide [here](unity_project.md).  

- With the SimpleWareHouseScene open, simply press the Play button at the top of the Editor.  
  
![animation of Play button being pressed and simulation starting](images/start_unity.gif)  

If the ROS2 nodes were already launched, you should see flashing, non-red arrows in the top-left HUD to indicate Unity is communicating with ROS.

---

## Interacting with this Example
- Once both the ROS2 nodes and Unity simulation are started and communicating correctly, you should see the occupancy map start to fill up in RViz.

![animation of RViz rendering occupancy map and laser scans](images/start_rviz.gif)

- The TurtleBot is now localizing AND mapping, simultaneously!  Now, to do navigation, click the `2D Goal Pose` button, and left-click, drag, and release a location in RViz to send a commanded pose to the navigation stack.


![animation of clicking 2D goal pose in RViz](images/goal_pose.gif)

Congratulations! The TurtleBot is now navigating the map as it generates it from laserscan data published from Unity. 

> While the Nav2 stack is pretty robust to most goal inputs, it may occasionally crash or otherwise reach a state where it no longer responds as expected. These issues can most of the time be fixed by simply killing and restarting the nav2 nodes, and pressing `Play` twice in Unity to stop and restart the simulation.

## Getting Started with Visualizations

The next step is to add visualizations in Unity using the Visualizations Package. Proceed to [Visualizing with Unity](unity_viz.md).

---

## Troubleshooting

**If you get the following error:**
```
launch.invalid_launch_file_error.InvalidLaunchFileError: Caught exception when trying to load file of format [py]: "package 'ros_tcp_endpoint' not found, searching: ['/home/rosdev/colcon_ws/install/unity_slam_example', '/opt/ros/foxy']"
```
You likely forgot to check out the submodules when following the [setup instructions](dev_env_setup.md). You will need to check them out with `git submodule update --init --recursive` and re-build the container as per the instructions in the linked page.

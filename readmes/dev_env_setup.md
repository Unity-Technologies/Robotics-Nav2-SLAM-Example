# Configure your Development Environment

**Table of Contents**
- [Install the Unity Editor](#install-the-unity-editor)
- [Clone the Project](#clone-the-project)
- [Set Up the ROS 2 Environment](#set-up-the-ros-2-environment)
    - [Build the Docker container](#build-the-docker-container)
    - [(Optional) Using an alternative ROS 2 environment](#optional-using-an-alternative-ros-2-environment)
- [Continue on to Setting up the Unity Project](#continue-on-to-setting-up-the-unity-project)

---  
## Install the Unity Editor  
This project was most recently validated with the Editor version listed in the [ProjectVersion.txt file](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/dev/Nav2SLAMExampleProject/ProjectSettings/ProjectVersion.txt#L1). If you have a more recent version, it will most likely work fine, or you may download the exact version we used from the release page [here](https://unity3d.com/unity/qa/lts-releases). Note that if using a more recent version, you will receive some warnings to this effect when you first open the project, but you are fine to proceed through them.

---

## Clone the Project
Check out the project using your method of choice, or simply copy the following line into a terminal with the `git` CLI installed:  
```
git clone --recurse-submodule git@github.com:Unity-Technologies/Robotics-Nav2-SLAM-Example.git
```  
>Observe our usage of `--recurse-submodules` to ensure any dependencies are checked out. If you miss this part, you will need to use `git submodule update --init --recursive` later to download and update the submodules.

---

## Set Up the ROS 2 Environment
This project should work with any appropriately configured ROS 2 environment, but we strongly encourage users new to ROS to build the environment from the [Dockerfile provided with the project](../ros2_docker/Dockerfile), which exposes a pre-configured ROS 2 environment to you via a built-in web VNC client. We may be unable to provide adequate troubleshooting support for other environments. 

### Build the Docker container
- From the root of the repository, run the following:
    ```
    cd ros2_docker
    docker build -t unity-robotics:nav2-slam-example ./
    ```
This build process will take at least 10 minutes, depending on your download speeds and hardware specificiations, but you are free to proceed to next step while it is building.


### (Optional) Using an alternative ROS 2 environment
If you prefer to use your own VM, or have ROS 2 installed natively and would like to use that instead, the colcon workspace for the ROS 2 side is located in the `ros2_docker/colcon_ws` directory.  You can relocate this folder to your VM, or `colcon build` it directly from the workspace root. Note that you may need to use `rosdep` to ensure you have the appropriate dependencies in place before your build and execution will both succeed.

---


## Continue on to Setting up the Unity Project
Once you're done configuring your environment to support this example, you can proceed to [Setting up the Unity Project](unity_project.md).

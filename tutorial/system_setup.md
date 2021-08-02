# AMR Demo: ROS 2 Integration, Section 1: System Setup

To get started, we want to download Unity, Docker, and the robotics packages we will be using with this project. 

### Required Downloads

 - [Download the Unity Hub](https://unity3d.com/get-unity/download) and install Unity 2020.3.* LTS
 >Note: Unity requires a license to use, but most users are able to use a free Unity Personal license
 - [Install Docker](https://docs.docker.com/get-docker/) based on your OS (if you are going to use the ROS 2 Docker)
 - [Install ROS 2](https://docs.ros.org/en/foxy/Installation.html) based on your OS (if you are going to run ROS 2 without Docker)
 - Open a terminal and navigate to the folder where you want to host the repository and run:
 `git clone --recurse-submodule git@github.com:Unity-Technologies/Robotics-Nav2-SLAM-Example.git`

## Create a new Unity project
When you first run Unity, you will be asked to open an existing project or create a new one. Open Unity and create a new project using the ***Universal Render Pipeline***. Name your new project ***AMR Tutorial***, and specify a desired location as shown below.
 TODO Image

## Download Unity Robotics packages
Once your new project is created and loaded, you will be presented with the Unity Editor interface. From this point on, whenever we refer to the "editor", we mean the Unity Editor.

### How to install packages
We will need to download and install several packages. In general, packages can be installed in Unity with the following steps:

 -   From the top menu bar, open  _**Window**_  ->  _**Package Manager**_. As the name suggests, the  _**Package Manager**_  is where you can download new packages, update or remove existing ones, and access a variety of information and additional actions for each package.
    
 -   Click on the  _**+**_  sign at the top-left corner of the  _**Package Manager**_  window and then choose the option  _**Add package from git URL...**_.
    
 -   Enter the package address and click  _**Add**_.
    
It can take a few minutes for the manager to download and import packages.
>Note: If you encounter a Package Manager issue, check the [Troubleshooting Guide](https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation/blob/main/Documentation/troubleshooting.md) for potential solutions.

### Packages to install
Install the following packages with the provided git URLs:

 - [URDF Importer package](https://github.com/Unity-Technologies/URDF-Importer)  -  `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`
    
    -   This package will help us import a robot into our scene from a file in the  [Unified Robot Description Format (URDF)](http://wiki.ros.org/urdf).

 - [TCP Connector package](https://github.com/Unity-Technologies/ROS-TCP-Connector)  -  `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
    
    -   This package will enable a connection between ROS 2 and Unity.
 
- [Robotics Warehouse package](https://github.com/Unity-Technologies/Robotics-Warehouse)  -  `https://github.com/Unity-Technologies/Robotics-Warehouse.git?path=/com.unity.robotics.warehouse`
    
    -   This repository contains a configurable warehouse environment that is ready for use in robotics simulation.

Next we will set up the Unity Scene.
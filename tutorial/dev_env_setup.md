# Configure your Development Environment
---  
## Install the Unity Editor  
You will need to download the version of Unity which matches the [ProjectVersion.txt file](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/dev/Nav2SLAMExampleProject/ProjectSettings/ProjectVersion.txt#L1), which should be `2020.3.12f1`. You may download the editor either through the Unity Hub or directly from Unity's download site, both of which are located [here](https://unity3d.com/get-unity/download).

---

## Clone the Project
Check out the project using your method of choice, or simply copy the following line into a terminal with the `git` CLI installed:  
```
git clone --recurse-submodule git@github.com:Unity-Technologies/Robotics-Nav2-SLAM-Example.git && \
cd Robotics-Nav2-SLAM-Example && \
git lfs install && \
git lfs pull
```  
>NOTE: Observe our usage of `--recurse-submodules` to ensure any dependencies are checked out, and `git lfs pull` to ensure any binary files in the project are also pulled down. 

---

## Set Up a ROS 2 Environment
This project should work with any appropriately configured ROS 2 environment, but we strongly encourage building the environment from the Dockerfile **>>>ADD LINK<<<** provided with the project, as it is the only use case for which we can provide troubleshooting support. This section will assume you are setting up your environment using our Dockerfile.
```
# From the repository root...
cd ros2_docker && \
docker build ./ && \
```


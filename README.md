# Navigation 2 SLAM Example

[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE.md)
![ROS](https://img.shields.io/badge/ros2-galactic-brightgreen)
![Unity](https://img.shields.io/badge/unity-2020.3.11f-brightgreen)

This example provides a Unity Project and a colcon workspace that, when used together, allows a user to substitute Unity as the simulation environment for the purposes of following the [Navigation 2 SLAM tutorials](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html). In following the setup instructions below, you will exercise Unity's ROS 2 integration, explore an example environment generated with our [Robotics Warehouse](https://github.com/Unity-Technologies/Robotics-Warehouse/tree/main) randomizable environment, integrate visualizations with our [Visualizations](https://github.com/Unity-Technologies/ROS-TCP-Connector/tree/main/com.unity.robotics.visualizations/Documentation~) package, and learn about ways in which this project could be extended to support a more specific use case or automated to provide continuous integration testing of a robotics navigation stack.

---

We're currently working on lots of things! As a first step for this tutorial, please take a short moment fill out our [survey](https://unitysoftware.co1.qualtrics.com/jfe/form/SV_0ojVkDVW0nNrHkW) to help us identify what products and packages to build next.

---

## Setup Instructions
1. ### [Configuring Your Development Environment](readmes/dev_env_setup.md)
1. ### [Setting Up the Unity Project](readmes/unity_project.md)
1. ### [Running the Example](readmes/run_example.md)
1. ### [Visualizing with Unity](readmes/unity_viz.md)
1. ### [Making a Custom Visualizer](readmes/custom_viz.md)

## Understanding the Project Components
* ### [Breakdown of this Example](readmes/explanation.md)
* ### [Robotics Warehouse](https://github.com/Unity-Technologies/Robotics-Warehouse)
* ### [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
* ### [Visualizations](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.visualizations/Documentation~/)
* ### [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer)

## Learning More About Unity
If you are brand new to Unity, found this example intersting, and want to learn more about how to use the Unity Editor, you may find these tutorials very useful for better familiarizing yourself with the Editor interface:
* [Exploring the Editor Layout](https://learn.unity.com/tutorial/exploring-the-editor-layout): a 10 minute tutorial giving a quick and straightforward overview of the Editor layout
* [Using the Unity Interface](https://learn.unity.com/tutorial/using-the-unity-interface): a 1 hr tutorial, which includes more context for each window and short introductions to commonly used components of the Scene

If you'd like more information about how to create customized environments, you may find some of these very in-depth tutorials on the creation of environments in Unity useful:

* [Foundations of Real-Time 3D](https://learn.unity.com/project/foundations-of-real-time-3d?uv=2019.4&missionId=5f777d9bedbc2a001f6f5ec7): A comprehensive look on the various aspects of building a realtime 3D environment in Unity. Although the whole course is 3 hours and 30 minutes, you may skip to the courses that are most relevant to your use case, which are likely to be [Work with GameObjects in a 3D Scene](https://learn.unity.com/tutorial/work-with-gameobjects-in-a-3d-scene?uv=2019.4&missionId=5f777d9bedbc2a001f6f5ec7&projectId=5fa1e431edbc2a001f53e6cc), [Add components to 3D GameObjects](https://learn.unity.com/tutorial/add-components-to-3d-gameobjects?uv=2019.4&missionId=5f777d9bedbc2a001f6f5ec7&projectId=5fa1e431edbc2a001f53e6cc), and [Get 3D assets](https://learn.unity.com/tutorial/get-3d-assets?uv=2019.4&missionId=5f777d9bedbc2a001f6f5ec7&projectId=5fa1e431edbc2a001f53e6cc)
* [Environments Live Session](https://learn.unity.com/tutorial/environments-march-31-2021#602326a4edbc2a4e1667c4c4): This Session provides a deeper dive into the intricacies of constructing a believable environment. While its focus is on environments for game development, most of the content is just as relevant to anyone interested in building realistic environments for simulation.

---

## Getting  Support
Many issues you may encounter can be solved by performing an internet search on the relevant error message and following the troubleshooting tips you find online. If you've already investigated your issue and haven't been able to find a suitable solution, please submit an Issue ticket here, describing your issue with as much detail as possible, including logs and screenshots as appropriate, and we'll do our best to help you resolve it.

## More from Unity Robotics
Visit the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) for more robotics-related tutorials and examples. Join in on discussions on robotics or start your own in the official [Unity Robotics Forums](https://forum.unity.com/forums/robotics.623/). 

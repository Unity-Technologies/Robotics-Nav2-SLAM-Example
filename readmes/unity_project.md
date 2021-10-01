# Setting up the Unity Project

**Table of Contents**
- [Open the Unity Project](#open-the-unity-project)
- [Open the SimpleWarehouseScene](#open-the-simplewarehousescene)
- [Configure your ROS Settings](#configure-your-ros-settings)
- [Continue to running the example](#continue-to-running-the-example)
- [Troubleshooting](#troubleshooting)

---

## Open the Unity Project
Assuming you have already [configured your dev environment](dev_env_setup.md), you can open the `Nav2SLAMExampleProject` either by adding the directory to your Unity Hub via the `Add` button, or opening it directly from an open Editor. 

![Unity Hub with projects tab, showing the Nav2SLAMExampleProject](images/unity_hub_projects.png)

> On Linux, you may encounter a bug in the Hub where, if you have no Projects in your list already, Add will not work properly. If you see this, create a New, empty project first, then try and Add Nav2SLAMExampleProject again.
---

## Open the SimpleWarehouseScene
Find the example scene, SimpleWarehouseScene, in the project browser, located in the bottom left of the Editor by default, and open it by double-clicking. 

![Project Browser pointing to Scenes directory](images/browser_to_scene.png)   

You should see something like this open in your Editor window:  

![SimpleWarehouseScene open in Editor](images/warehouse_scene.png)

---

## Configure your ROS Settings
Use the `Robotics` drop-down menu to open the `ROS Settings` menu.  

![Robotics drop-down with ROS Settings selected](images/ros_settings_menu.png)  

In the ROS Settings panel:
* Ensure the `Connect on Startup` checkbox is checked
* Ensure `Protocol` is set to `ROS2`
* For those using a Docker container on the same machine as per our guidance from the [setup instructions](dev_env_setup.md), leave your `ROS IP Address` as the default: `127.0.0.1`. 
  * Otherwise, set your `ROS IP Address` to the address of the adapter your ROS2 environment is using.
* Ensure the `Show HUD` is checked. This is to ensure you get visual feedback on the state of your ROS connection during runtime.

![ROS Settings Panel with appropriate settings](images/ros_settings_window.png)  

---

## Continue to running the example
Everything is now configured to [run the example](run_example.md). If you'd like to understand more about how this project is set up and what the different components are doing, you may skip to [Understanding the Project Components](explanation.md)

---

## Troubleshooting

If you encounter the following error when opening the Unity project:

```
An error occurred while resolving packages:
  Project has invalid dependencies:
    com.unity.robotics.warehouse: Error when executing git command. error: RPC failed; curl 56 LibreSSL SSL_read: SSL_ERROR_SYSCALL, errno 54
    error: 5749 bytes of body are still expected
    fetch-pack: unexpected disconnect while reading sideband packet
    fatal: early EOF
    fatal: index-pack failed
```

You may need to modify the compression, e.g. `git config --global core.compression 9`.
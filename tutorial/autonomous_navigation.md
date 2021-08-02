# AMR Demo: ROS 2 Integration, Section 4: Autonomous Navigation

In the ROS 2 Setup section, installed prepared our ROS 2 build and started running the software which also launched RViz. Now we are ready to start Unity and perform autonomous navigation.
## Start the Unity simulation
With the AMRwarehouse scene open, simply press the Play button at the top of the editor.

**TODO** Add image

If the ROS2 nodes were already launched, you should see flashing, non-red arrows in the top-left HUD to indicate Unity is communicating with ROS.

## Performing autonomous navigation
Once both the ROS2 nodes and Unity simulation are started and communicating correctly, you should see the occupancy map start to fill up in RViz.

**TODO** Add an image

The TurtleBot is now localizing AND mapping, simultaneously! Now, to do navigation, click the `2D Goal Pose` button, and left-click, drag, and release a location in RViz to send a commanded pose to the navigation stack.
**TODO** Add something about the package that the robot is using to perform this navigation. And the workflow of how imformation is flowing from Unity to ROS 2.

**TODO** Add an image

## Changing the environment
Autonomous navigation is powerful because it doesn't restrict the robot to a single layout. Developers can write code once and use it for many different environments, both static and dynamic.

### Dynamic environment changes
During Play mode, a UI overlay will appear in the top-right corner of the Game view. This can be used to spawn box towers or piles of debris with specified parameters.

**TODO**

The following parameters are available:
-   **Show Location Picker**: This toggle enables a visualizer that displays where the objects will spawn.
-   **Spawn boxes**: The text field to the right of this button expects an input with the format  `width,length,height`, e.g.  `2,3,4`. Pressing the  `Spawn boxes`  button will create a tower of boxes with the given dimensions on the Location Picker location.
-   **Debris Size**: This slider defines the maximum scale of the debris spawned.
-   **Debris is kinematic**: This toggle defines if the debris spawned is kinematic or not.
-   **Spawn debris**: The text field to the right of this button expects an integer input that describes the number of objects to spawn. Pressing the  `Spawn debris`  button will instantiate that number of random primitives of randomized scale (up to the  `Debris Size`).

### Static environment changes
The warehouse generator used in the Scene setup has additional settings that can be changed to change the look and layout of the environment. When you modify the width and length in step 5 of Set up the Scene, you can also change more about the appearance of the Scene.  In the _**Inspector**_ tab, the **Scenario Shim** on the `Warehouse Manager` Game Object defines the core logic for randomization. Assign the values as desired (usage defined below):

-   **SunAngleRandomizer**  - Assigns the directional light angle and location.
-   **LocalRotationRandomizerShim**  - Assigns local rotation, used only on the shelves.
	> Note: It is suggested to keep the  `X`  and  `Z`  values at 0
	
-   **ShelfBoxRandomizerShim**  - Randomizes the number of boxes on the shelves. Use  `Box Spawn Chance`  to define the percent chance of a box spawning at each possible location.
-   **FloorBoxRandomizerShim**  - Spawns boxes on the floor of the warehouse. Use  `Num Box To Spawn`  to define how many boxes are spawned.

This can be re-run by incrementing the scenario iteration. This can be done via the  `WarehouseManager`  GameObject, which shows an  `Increment Iteration`  button in the Inspector.

**TODO** Add picture

Congratulations! You completed the first part of the AMR Demo making the TurtleBot simulataniously generate and navigate the map awhile reading laserscan data published from Unity!

## Additional Challenges
Great work getting the Turtlebot3 running in Unity! This simple AMR example is just the start of our project and if you are interested in keeping up with new releases and news for Unity Robotics, sign up for the [Unity Robotics Newsletter](https://create.unity3d.com/robotics-simulation-newsletter-sign-up).

While you wait for new parts, here are some other ideas that you can try out to see if you can get them to work:

 - Add your own robot to the Unity Scene instead of the Turtlebot and and perform the same task.
 - Add two robots to the Unity Scene and see how you would manage the communication (will you have them do their own thing or work together?)
 - Try different warehouse layouts and see if they affect the navigation abilities at all
 
We are excited to see the work that you do with this project! Please share your project or any questions or comments you have on the [Unity Robotics forum](https://forum.unity.com/forums/robotics.623/)!
# Making a Custom Visualizer

While the Message Visualizations package provides a preconfigured default visualization suite, there are many compelling cases for custom visualizations. This page steps through how to create a custom visualizer for the nav2 project that tracks a history of `/goal_pose` messages over time, drawing a path between each point. 

TODO: screenshot

**Table of Contents**
- [Creating a New Visualizer](#creating-a-new-visualizer)
    - [Drawing the UI Window](#drawing-the-ui-window)
    - [Creating Drawings](#creating-drawings)
- [What's Next?](#whats-next)

> This section assumes you have already set up your environment according to the [configuration guide](dev_env_setup.md) and have run the example successfully as described [here](run_example.md).

---

## Creating a New Visualizer

- To start writing a custom visualizer, start by making a new script named `PoseTrailVisualizer`.

    > To create a new script, right click in Unity's Project window, and select `Create > C# Script`.

    > TODO: To use the completed script, you can add the file from [DIRECTORY PoseTrailVisualizer]() and skip to []() step.

- Make a new GameObject in your scene named `Visualizer`. Add the newly created PoseTrailVisualizer component to the Visualizer GameObject. 

    - If you have the `DefaultVisualizationSuite` in your scene from the previous tutorial, the necessary components are already added and you can move to editing the script.
    
    - If you do not have the `DefaultVisualizationSuite` in your scene, you will also need the `VisualizationsTopicsTab` component in order to add the `Topics` tab to your HUD. 
    
    On your `Visualizer` GameObject, add the `VisualizationsTopicsTab` component now. This component extends the ROS Connection HUD to show options for visualizations on registered topics. 

    > Learn more about the HUD TEMP LINK [here](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/amanda/default-tutorial/com.unity.robotics.message-visualizations/Documentation~/README.md#the-hud).

- Creating a new script will create a template that automatically inherits from [MonoBehaviour](https://docs.unity3d.com/Manual/class-MonoBehaviour.html) with the basic using directives. To turn this script into a visualizer, we'll need to reference the additional required packages. This includes the generated messages, ROS Geometry for coordinate conversions, and, of course, message visualizations.

    Open your newly created script for editing. At the top of the script, import these namespaces:

    ```csharp
    using RosMessageTypes.Geometry;                     // Generated message classes
    using Unity.Robotics.MessageVisualizers;            // Message visualizations
    using Unity.Robotics.ROSTCPConnector.ROSGeometry;   // Coordinate space utilities
    ```

    You now have access to the necessary classes and functions for this visualization. A visualizer that manages multiple drawings over time for a specific message type should inherit from the `MultiDrawingVisualizer<T>` class. Do this now by replacing the `MonoBehaviour` class with `MultiDrawingVisualizer<PoseStampedMsg>`.

    > Learn more about the visualizer base classes TODO link [here]().

```csharp
using System;
using System.Collections.Generic;
using RosMessageTypes.Geometry;                     // Generated message classes
using Unity.Robotics.MessageVisualizers;            // Message visualizations
using Unity.Robotics.ROSTCPConnector.ROSGeometry;   // Coordinate space utilities
using UnityEngine;

public class PoseTrailVisualizer : MultiDrawingVisualizer<PoseStampedMsg>
{
    // Start is called before the first frame update
    void Start()
    ...
}
```

Your template class should now look something like the above code block. However, this script won't do anything yet! Move onto the next step to populate the UI window for the message.

### Drawing the UI Window

In the Message Visualizations package, UI windows are registered based on its topic, and will update whenever a message is sent or received on it. That being said, the visualizer base class (in this case, `MultiDrawingVisualizer`) manages all of this already. All we'll have to do is format the message contents in the window.

- Implementations of the visualizer classes override the function `CreateGUI()` in order to populate the UI window. By default, this function will simply display the raw message converted to a string. The MultiDrawingVisualizer acts a bit differently, managing multiple messages over time.

    Begin by adding an empty override for MultiDrawingVisualizer's `CreateGUI` function to your visualizer script.

    ```csharp
    public override Action CreateGUI(IEnumerable<Tuple<PoseStampedMsg, MessageMetadata>> messages)
    {
        return () =>
        {

        };
    }
    ```

- The Message Visualizations package contains a convenient set of utility functions to format common message types to strings as well as draw common geometries. As the messages are passed in as an IEnumerable, we can simply iterate through them and call the desired functions.

    As `/goal_pose` is a PoseMsg, we will use the utility function for its GUI, as defined [TEMP LINK] [here](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/4c74d53961e581ab68021dc082aa0c6c5a83ea8f/com.unity.robotics.message-visualizations/Runtime/Scripts/MessageVisualizationUtils.cs#L309-L317). The function takes an optional string parameter that serves as a label for the formatting. In this case, label the string as with a goal number, counting up from a newly initialized counter variable.

    Add this functionality between the `return` curly brackets. Your overridden function should look as follows:

    ```csharp
    public override Action CreateGUI(IEnumerable<Tuple<PoseStampedMsg, MessageMetadata>> messages)
    {
        return () =>
        {
            int count = 0;
            foreach (var (message, meta) in messages)
            {
                message.pose.GUI($"Goal #{count}");
                count++;
            }
        };
    }
    ```

- The GUI window is almost ready to be tested! Open a terminal window in your ROS workspace. Source the workspace and, once again, run the `ros2 launch` for the project if it is not already running.

    ```bash
    ros2 launch unity_slam_example unity_slam_example.py
    ```

- Return to Unity. Select the `Visualizer` object, and in the Inspector, find this `PoseTrailVisualizer` component. 

    Change the `History Length` to however many goals you would like to track over time--for example, `5`. 

    TODO: screenshot

- TODO: Enter Play mode. Click the `Topics` button in the top-left HUD to open the list of subscribed topics. Find the `/goal_pose` topic and toggle on the `UI`. 

    On the right side of the `/goal_pose` row, click to expand the hamburger menu and select your new visualizer, `PoseTrailVisualizer`. 

    The UI window should appear, waiting for messages on the topic. Begin publishing goal poses, and you will see the UI window update appropriately!

    TODO: screenshot

Move onto the next step to begin customizing and adding the 3D drawing to your visualization.

### Creating Drawings

Like the text and UI windows, 3D visualizations from this package are customizable. A set of basic customization parameters can include, for example, the thickness or color of lines drawn.

- Define these customizable parameters for the trail drawing as serialized private fields at the top of your class.

    ```csharp
    [SerializeField]
    Color m_Color = Color.white;
    [SerializeField]
    float m_Thickness = 0.1f;
    [SerializeField]
    string m_Label = "";
    ```

    > Note: Size-related fields are in Unity coordinates, where 1 unit = 1 meter. Learn more about visualization settings TEMP LINK [here](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/amanda/default-tutorial/com.unity.robotics.message-visualizations/Documentation~/README.md#visualization-settings).

- Like the GUI function, implementations of the visualizer classes also override the function `Draw()` for updating a 3D drawing in the Unity scene. Once again, the MultiDrawingVisualizer acts a bit differently, managing multiple messages over time--the updating and cleanup of message drawings past the saved history is managed by the MultiDrawingVisualizer class--all we'll have to do is feed the pose messages in.

    Start by adding an empty override for MultiDrawingVisualizer's `Draw` function in your visualizer script.

    ```csharp
    public override void Draw(Drawing3d drawing, IEnumerable<Tuple<PoseStampedMsg, MessageMetadata>> messages)
    {

    }
    ```

- In the Draw function, we'll define a few necessary variables for setting the drawing's color and label, as well as each intermediate point, used to draw the line segments and update the position of the text label.

    ```csharp
    var firstPass = true;
    var prevPoint = Vector3.zero;
    var color = Color.white;
    var label = "";
    ```

- Similar to the GUI window, we can simply iterate through the set of saved messages in order to update the line drawing, drawing each segment along the way using the utilities provided for `DrawLine` (as defined TODO link [here]()). Add the following loop to the `Draw()` function.

    ```csharp
    foreach (var (msg, meta) in messages)
    {
        var point = msg.pose.position.From<FLU>();
        if (firstPass)
        {
            color = MessageVisualizationUtils.SelectColor(m_Color, meta);
            label = MessageVisualizationUtils.SelectLabel(m_Label, meta);
            firstPass = false;
        }
        else
        {
            drawing.DrawLine(prevPoint, point, color, m_Thickness);
        }

        prevPoint = point;
    }
    ```

- Finally, we want to update the position of the text label to be at the most updated position, so add a call to the utility `DrawLabel()` after the `foreach` loop.
    
    ```csharp
    drawing.DrawLabel(label, prevPoint, color);
    ```

Your overridden function should look as follows:

```csharp
public override void Draw(Drawing3d drawing, IEnumerable<Tuple<PoseStampedMsg, MessageMetadata>> messages)
{
    var firstPass = true;
    var prevPoint = Vector3.zero;
    var color = Color.white;
    var label = "";

    foreach (var (msg, meta) in messages)
    {
        var point = msg.pose.position.From<FLU>();
        if (firstPass)
        {
            color = MessageVisualizationUtils.SelectColor(m_Color, meta);
            label = MessageVisualizationUtils.SelectLabel(m_Label, meta);
            firstPass = false;
        }
        else
        {
            drawing.DrawLine(prevPoint, point, color, m_Thickness);
        }

        prevPoint = point;
    }

    drawing.DrawLabel(label, prevPoint, color);
}
```

- Your visualizer is ready to fully test! You are free to modify the thickness, color, and label of the visual in the `PoseTrailVisualizer`'s Inspector window.

    Enter Play mode, and begin publishing goal poses. You should now see line segments connecting each of the published goal poses in order.

You have now completed the tutorial for creating a custom visualizer for your nav2 simulation!

TODO: gif

## What's Next?

<!--  -->

To learn more about using the Message Visualizations package, visit the package [TEMP link] [documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/amanda/default-tutorial/com.unity.robotics.message-visualizations/Documentation~/README.md).
# Making a Custom Visualizer

TODO: default visualizers are cool but custom ones can be better

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

- TODO: if you are still using the dfeualt visualizer...
- TODO: else.... In order to toggle on visualizations, you will also need the `VisualizationsTopicsTab` component. Add this component to your Visualizer GameObject.

    This component extends the ROS Connection HUD to show options for visualizations on registered topics. 

    > Learn more about the HUD TEMP LINK [here](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/amanda/default-tutorial/com.unity.robotics.message-visualizations/Documentation~/README.md#the-hud).

- Creating a new script will create a template that automatically inherits from [MonoBehaviour](https://docs.unity3d.com/Manual/class-MonoBehaviour.html) with the basic using directives. To turn this script into a visualizer, we'll need to reference the additional required packages. This includes the generated messages, ROS Geometry for coordinate conversions, URDF importer for robot drawings, and, of course, message visualizations.

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

Your template class should now look something like the above code block. However, this class won't do anything yet! Move onto the next step to populate the UI window for the message.

### Drawing the UI Window

In the Message Visualizations package, UI windows are registered based on its topic, and will update whenever a message is sent or received on it. That being said, the visualizer base class (in this case, `MultiDrawingVisualizer`) manages all of this already. All we'll have to do is format the message contents in the window.

- Implementations of the visualizer classes override the function `CreateGUI()` in order to populate the UI window. By default, this function will simply display the raw message converted to a string. The MultiDrawingVisualizer acts a bit differently, managing multiple messages over time.

    Begin by adding an empty override for the `CreateGUI` function to your visualizer script.

    ```csharp
    public override Action CreateGUI(IEnumerable<Tuple<PoseStampedMsg, MessageMetadata>> messages)
    {
        return () =>
        {

        };
    }
    ```

- TODO: Let's start 

    The Message Visualizations package contains a convenient set of utility functions to format common message types to strings as well as draw common geometries.

    As `/goal_pose` is a PoseMsg, we will use the utility function for its GUI, as defined [TEMP LINK] [here](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/4c74d53961e581ab68021dc082aa0c6c5a83ea8f/com.unity.robotics.message-visualizations/Runtime/Scripts/MessageVisualizationUtils.cs#L309-L317). he function takes an optional string parameter that serves as a label for the formatting. In this case, label the string as with a goal number, counting up from a newly initialized counter variable.

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

- TODO The GUI window is almost ready to be tested! Open a terminal window in your ROS workspace. Source the workspace and, once again, `ros 2` the server endpoint and trajectory subscriber if it is not already running.

```bash
```

- TODO: In Unity, change the history length

- TODO: Enter Play mode. Click the `Topics` button in the top-left HUD to open the list of subscribed topics. Find the `/goal_pose` topic and toggle on the `UI`. The UI window should appear TODO

    TODO: screenshot

Move onto the next step to begin customizing and adding the 3D drawing to your visualization.

### Creating Drawings

TODO: expos

The visualizations from this package are customizable. A basic customization parameter can include, for example, the thickness or color of lines drawn.

- Define this customizable parameters for the trail drawing as serialized private fields at the top of your class.

    ```csharp
    [SerializeField]
    Color m_Color = Color.white;
    [SerializeField]
    float m_Thickness = 0.1f;
    [SerializeField]
    string m_Label = "";
    ```

    > Note: Size-related fields are in Unity coordinates, where 1 unit = 1 meter. Learn more about visualization settings TEMP LINK [here](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/amanda/default-tutorial/com.unity.robotics.message-visualizations/Documentation~/README.md#visualization-settings).

- Like the GUI function, implementations of the visualizer classes also override the function `Draw()` for updating a 3D drawing in the Unity scene. Once again, t he MultiDrawingVisualizer acts a bit differently, managing multiple messages over time--the updating and cleanup of message drawings past the saved history is managed by the MultiDrawingVisualizer class--all we'll have to do is feed the pose messages in.

    Start by adding an empty override for the `Draw` function in your visualizer script.

    ```csharp
    public override void Draw(Drawing3d drawing, IEnumerable<Tuple<PoseStampedMsg, MessageMetadata>> messages)
    {

    }
    ```

- TODO: whatever these are 
    ```csharp
    var firstPass = true;
    var prevPoint = Vector3.zero;
    var color = Color.white;
    var label = "";
    ```

- TODO: drawing, utilities, drawline, selecting colors

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

- TODO: draw the label at the end
    
    ```csharp
    drawing.DrawLabel(label, prevPoint, color);
    ```

Add all of this functionality between the `return` curly brackets. Your overridden function should look as follows:


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

- TODO: enter play mode

You have now completed the tutorial for creating a custom visualizer for your nav2 simulation!

## What's Next?

<!--  -->

To learn more about using the Message Visualizations package, visit the package [TEMP link] [documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/amanda/default-tutorial/com.unity.robotics.message-visualizations/Documentation~/README.md).
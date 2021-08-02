# AMR Demo: ROS 2 Integration, Section 2: Scene Setup
In the System Setup section, we got our machine ready for the project. Now we will set up the Unity Scene.

Simply put in Unity, a Scene contains any object that exists in the world. This world can be a game, or in this case, a data-collection-oriented simulation. Every new project contains a Scene named `SampleScene`, which is automatically opened when the project is created. This Scene comes with several objects and settings that we do not need, so let's create a new one.

## Set up the Scene

 1. In the  _**Project**_  tab, right-click on the  `Assets/Scenes`  folder and click  _**Create -> Scene**_. Name this new Scene  `AMRwarehouse`.
 2. In the same _**Project**_ tab, create new folders by right-clicking a folder and selecting _**Create->Folder**_ with the following structure `Assets/Resources/Prefabs/SavedWarehouses`
 **TODO** Add image
 3. The Robotics Warehouse package contains a warehouse generator that allows you to generate a warehouse easier than starting from scratch. Go to the _**Project**_ tab and double-click `Packages/Robotics Warehouse/Scenes/Warehouse` scene object to open the Warehouse scene. The scene contains `Main Camera`, `Directional Light`, and `Warehouse Manager` GameObjects in the _**Hierarchy**_ tab. Select the `Warehouse Manager` in the _**Hiearchy**_ tab.
 **TODO** Image
 4. Under the _**Inspector**_ tab, find the _**Warehouse Manager (Script)**_ object and ensure that the following prefabs are set:
	   - **Shelf Prefab:**  _ShelvingRackRandom_ 
	   - **Warehouse Prefab:** _Warehouse (Transform)_ 
	   >Note: If these are not populated, you will have to locate these objects under `Packages/Robotics Warehouse/Prefabs` and drag-and-drop these items into the respective field as noted above.
	  
	**TODO** Image
 
 5. Expand the  `App Param`  menu on the  _**Warehouse Manager (Script)**_ object . This defines the length and width of the warehouse, and how many rows and columns of shelves are instantiated. Set these values as you want to modify the appearance of the warehouse.
 6. Click the  `Generate`  button on the WarehouseManager to generate the warehouse with the specified parameters.
 7. Click  `Save prefab`  to save this version of the warehouse to  `Assets/Resources/Prefabs/SavedWarehouses`. 
 8.  In the _**Project**_ tab, open your new scene back up by double-clicking the scene that was made in step 1 `Assets/Scenes/AMRwarehouse`.
 9. In the _**Project**_ tab, open `Assets/Resources/Prefabs/SavedWarehouses` and drag-and-drop the **GeneratedWarehouse** prefab into the _**Hierarchy**_ tab.

## Set up the robot

 1. Unity is able to import robots using the [URDF Importer package](https://github.com/Unity-Technologies/URDF-Importer).
 2. **TODO** Turlebot import and setup
 3. **TODO** Add on laserscan sensor
 4. Configure the ROS 2 settings by going to the top drop-down menu and selecting _**Robotics->ROS Settings**_. In the ROS Settings window that opens, ensure the following settings:
	 - **Connect on Startup:** _checked_
	 - **Protocol:** _ROS 2_
	 - **ROS IP Address:** _127.0.0.1_

 Next we will setup the ROS 2 software.
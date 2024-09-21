# Resources
- Good [video](https://www.youtube.com/watch?v=9AsDmPJWcnQ&t)
# Basics
- Some basics [here](https://control.ros.org/humble/doc/ros2_control_demos/doc/index.html#quick-hints)
- ROS1 has RobotHW (Robot Hardware) which is like abstraction layer between Hardware & ROS control system. It basically defines how to read sensor data adn how to send commands to the actuators
- In ROS2, this has evolved to: Hardware interface (Hardware Component)
# Hardware components
- Three types:
	1. System: Both read and write capability.
	2. Sensor: Only read
	3. Actuator: Both read (optional if not necessary) and write
- Hardware components are added to robot's URDF file as Hardware Description with `<ros2_control>` tag.
- One `<ros2_control>` used for one hardware interface. And there can be multiple hardware interfaces in a single robot. [This](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md) example has multiple example of using this tag.
Example of `ros2_control` tag:
```xml
<ros2_control>

%% Hardware Driver Plugin %%
<hardware name="any_name_here" type="system|sensor|actuator">
	<plugin>name_of_hardware_driver_ros_package/ros_executable(hola)</plugin>
	<param name="param_of_the_above_executable_node"> param_value </param>
	... more params here
</hardware>

%% Joints and Sensors? %%
<joint name="joint_name_as_mentioned_in_the_same_urdf_file">

	%% Interface in this joint to send actuator command %%
	<command_interface name="">
		<param name="param_name_for_this_command_interface"> param_val </param>
		... other params here
	</command_interface>

	%% Interface in this joint for sensing %%
	<state_interface name=""/>

	... Other command and state interfaces here

</joint>

<sensor name="as mentioned in urdf?????">
	<state_interface name=""/>
	... params for sensor here
</sensor>


</ros2_control>
```
- Each `<joint>` tag can have multiple `<command_interface>` and `<state_interface>`
- Common examples for `command_interface` names are:
	- position, velocity, effort, Kp, Kd, effort_limit

> [!Error] Question
> Does Position, Velocity, Effort used here have any special meaning? or it is just a simple name like Kp, Kd, effort_limit?

- Common examples for `state_interface` names are:
	- position, velocity, effort, timestamp, coil_resistance, contact_information
- params for:
	- command_interface: min, max values
	- state_interface: No (as it just read the given value)
	- sensor: info about sensor like frame_id, ... not sure about others!!!

> [!Error] Confusions
> - params not used for `<state_interface>` anyhwere why?
> - but params is used for `<sensor>` as a whole
> - but params now used for `<hardware>` tag as a whole, and still there is `<state_interface>` tag inside the `<hardware>` tag
> - are sensors mentioned in URDF as well? like joints
- List hardware components by `ros2 control list_hardware_components`
  ![[Pasted image 20240830115540.png]]
- List hardware interfaces by `ros2 control list_hardware_interfaces`
  ![[Pasted image 20240830101726.png|350]]
- available: means the interface (command) is available to use by controllers
- claimed: means the controller is actively using it to control the hardware
- state_interfaces doesn't need to be available or claimed, as these terms are solely associated to the controllers, and state_interface is just used for gathering data
# Controller Manager
- Needs two things
	1. URDF: to know the hardware interface to load
	2. launch params file: to know which controllers to load
- Normally, other ros packages gets URDF file from `/robot_description` topic published by `robot_state_publisher`. But the newer version does this ?????? 
- Yes, from jazzy onwards, the `/controller_manager` subscribes to `/robot_description` topic
- Example of UR5:
  Humble:...................................................................Rolling:
  ![[Pasted image 20240827151644.png|330]] ![[Pasted image 20240827151741.png|330]]
- `gazebo_ros2_control` does take URDF from `/robot_description` topic
# ros2_controllers
## Categories
Can be categorized to 3 variants:
1. For Wheeled Mobile Robots
   - Differential Drive Controller
   - Tricycle Controller
2. For Manipulators and others
   - Joint Trajectory Controller (imp)
   - Position Controllers
   - Velocity Controllers
   - Effort Controllers
   - PID Controller
   - Gripper Controllers
   - etc...
3. Broadcasters
   Not technically a controller, still falls under ros2_controllers.
   - Joint State Broadcaster (imp)
   - Force Torque Sensor Broadcaster
   - IMU sensor Broadcaster
   - Range Sensor Broadcaster

List all available **types of controllers** by: `ros2 control list_controller_types`
![[Pasted image 20240830101232.png|500]]
- See list of Controllers by: `ros2 control list_controllers`
  ![[Pasted image 20240830102219.png]]
## States
- A controller must already be in `controller.yaml` file which was sent to the `controller_manager` before doing this.
- A Controller can be in 3 states:
	1. unconfigured: Not loaded , Not configured, Not available to use
	2. inactive: available, but a controller is not using it
	3. active: available and a controller is using it as well
![[Pasted image 20240830102537.png]]
- Important command here are: `load_controller`, `set_controller_state`
# Joint Trajectory Controller
- Controls all the available joints at once? Unlike joint position controller which controls each joint one at a time! 
![[ros2_control.excalidraw|600]]
# xacro
- It's just a convention. URLs (which is what they are, not really URLs) are a convenient and globally-familiar way of identifying resources.
- reserved keywords in xacro ?
## Property and Argument
- `xacro:property` is like const keyword | `#define`
- `xacro:arg` is like variable | function parameter
## Macro
- `xacro:macro` is like function
  ```xml
    <xacro:macro name="rrbot" params="parent prefix *origin">
   ```
   is similar to python code:
   ```python
   def rrbot(parent, prefix, *origin)
   ```
- Like any other function, it needs to be imported, and then called.
- Importing a function|macro:
  If above function is defined in a file named `/path/to/file.xacro`, then it is imported as
  ```xml
  <xacro:include filename="/path/to/file.xacro" />
  ```
- Calling the function|macro:
  ```xml
  <xacro:rrbot parent="world" prefix="my_bot">
    <origin xyz="0 0 0" rpy="0 0 0" />
  </xacro:rrbot>
  ```
- Here you see, `parent` and `prefix` are simple string, but `*origin` is entire block of `xml` code. That means `*` denotes a block of `xml` code.
- To use these parameters inside function body, here is quick peep inside a part of function definition
  ```xml
  <joint name="${prefix}_base_joint" type="fixed">
    <xacro:insert_block name="origin" />
    <parent link="${parent}"/>
    <child link="${prefix}_base_link" />
  </joint>
  ```
- `"${parent}"` is used to directly extract the `string` parameter
- `xacro:insert_block` is used to extract the xml code block `*origin`
## xmlns: XML Namespaces
  ```xml
  <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
   ```
- Here, the word `xacro` (in xmlns:**xacro**) is a namespace prefix and the URL is unique identifier
- The `xacro` prefix is used as namespace, and put before other "xacro" special commands. for example: `xacro:macro`, `xacro:parameter` etc.
- You can use any string for prefix, eg: put `xmlns:my_cool_xacro_name` and use it as `my_cool_xacro_name:macro`
- Here the Namespace prefix is used to protect the xacro special xml tags. So, it will be easier for xacro parser to identify and don't get confuse with other xml tags like `joint`, `link`
- URL used here is unique identifier, and can be any unique string. Not necessary to be URL!
- General convention and for consistency ROS users uses above given prefix and identifier.

> [!Success] Question
> - Can I use another word than xacro for xmlns?

# Extras
![[Pasted image 20240827134704.png|350]]
![[Pasted image 20240827134911.png|300]]  ![[Pasted image 20240827141104.png|300]]

# Very important Stuff
Even if the `my_robot_controllers.yaml` file defines the `joint_state_broadcaster` and `joint_trajectory_controller`, these controllers still need to be actively loaded and started by the `controller_manager` during runtime. The spawner tool is used for this purpose.

### **Why Use the Spawner?**

1. **Configuration vs. Activation**:
    
    - The YAML file configures the controllers by specifying parameters such as the type of controller, which joints it controls, and other relevant settings. However, this configuration alone doesn't activate or start the controller. It only tells the `controller_manager` how to configure the controller once it's started.
2. **Loading and Starting Controllers**:
    
    - The **spawner** is a utility that loads and starts the controllers defined in your configuration. When you use the spawner, it instructs the `controller_manager` to instantiate the controller with the specified configuration and begin its operation.
3. **Separation of Concerns**:
    
    - This separation between configuration (in the YAML file) and activation (via the spawner) allows for greater flexibility. For example, you might want to define multiple controllers in your configuration file but only start specific ones depending on the situation. Using the spawner, you can control exactly which controllers are loaded and when.
4. **Dynamic Control**:
    
    - The spawner can be used at any point during runtime to load and start controllers. This is useful in complex systems where different controllers might need to be started or stopped based on the robot’s current mode of operation.

### **Summary**

- **YAML File**: Defines the configuration of the controllers (e.g., types, parameters, joints).
- **Spawner**: Loads and starts the controllers, making them active so they can interact with the hardware or simulation.

Without the spawner, the controllers defined in the YAML file would remain inactive, meaning no control would be exerted over the robot's joints, and no joint states would be published. The spawner essentially "brings the configuration to life."

```bash
╭─rashik@dell ~
╰─$ ros2 control list_hardware_interfaces
command interfaces
        joint1/position [available] [claimed]
        joint2/position [available] [claimed]
state interfaces
        joint1/position
        joint2/position
╭─rashik@dell ~
╰─$ ros2 control list_controllers
forward_position_controller[forward_command_controller/ForwardCommandController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
╭─rashik@dell ~
╰─$ ros2 control load_controller joint_trajectory_position_controller
Successfully loaded controller joint_trajectory_position_controller
╭─rashik@dell ~
╰─$ ros2 control list_controllers
forward_position_controller[forward_command_controller/ForwardCommandController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] unconfigured
╭─rashik@dell ~
╰─$ ros2 control set_controller_state joint_trajectory_position_controller inactive
Successfully configured joint_trajectory_position_controller
╭─rashik@dell ~
╰─$ ros2 control list_controllers
forward_position_controller[forward_command_controller/ForwardCommandController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
╭─rashik@dell ~
╰─$ ros2 control set_controller_state forward_position_controller inactive
Successfully deactivated forward_position_controller
╭─rashik@dell ~
╰─$ ros2 control list_controllers
forward_position_controller[forward_command_controller/ForwardCommandController] inactive
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
╭─rashik@dell ~
╰─$ ros2 control set_controller_state joint_trajectory_position_controller active
Successfully activated joint_trajectory_position_controller
╭─rashik@dell ~
╰─$ ros2 control list_controllers
forward_position_controller[forward_command_controller/ForwardCommandController] inactive
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] active
```
- [[Introduction to ROS2]] framework for robotic motion planning.
- Python API is not available for ROS2 humble, it is available for ROS2 rolling!
# Packages
- moveit_common
- moveit_configs_utils
- moveit_core
- moveit_kinematics
- moveit_msgs
- moveit_planners_ompl
- moveit_ros_move_group (imp)
- moveit_ros_occupancy_map_monitor
- moveit_ros_planning
- moveit_ros_planning_interface
- moveit_ros_robot_interaction
- moveit_ros_visualization
- moveit_ros_warehouse
- moveit_servo
- moveit_simple_controller_manager

# Null Space
- **Null space** is the set of all possible configurations (joint angles) of a robot that result in the same end-effector pose
- For a robot with redundant degrees of freedom (DOF), there can be multiple ways to achieve the same end-effector pose
- Null space can be explored using this slider
  ![[Pasted image 20240903195546.png|300]]

# Setup Assistant
- Helps to auto generate config files (.yaml, .rviz, .srdf) and launch files (.launch.py)
- Launch setup Assistant using:
  `ros2 launch moveit_setup_assistant setup_assistant.launch.py`
- `rsp.launch.py`: robot state publisher
# SRDF
- Semantic Robot Description Format (SRDF) is a file format used in the MoveIt framework to define the semantic properties of a robot
- URDF file has physical and kinematics properties only
- SRDF files can include properties like:
	- planning group
	- virtual joint: eg. to attach robot to the world
	- passive joint: some joints will not be moved actively by motors, something like a hinge
	- etc...
- **Planning Group**: Imagine you have a robotic arm with six joints, like a human arm with shoulder, elbow, and wrist joints. You want to control this arm to pick up objects, so you create a **Planning Group** called "Arm." This group will include all the joints from the shoulder to the wrist. When you tell MoveIt to move the "Arm" group, it will plan and execute motions for all those joints together to achieve the desired position.
# `move_group` node
- This node is responsible for moving a planning group!! Hence the name **move** , **group**
- i.e it moves a group of joints !
- This is a level of abstraction higher than `ros2_control`.
- `ros2_control` controls specific joint, but moveit deals with a group of joints at once. How cool is that!!
![[Pasted image 20240903231345.png|500]]
- The **move_group** node will generate a desired trajectory in response to your motion plan request. This trajectory will move the arm (or any group of joints) to the desired location. Note that the result coming out of **move_group** is a **trajectory** and not just a path. 
- will use the desired maximum velocities and accelerations (if specified) to generate a trajectory that obeys velocity and acceleration constraints at the joint level.
# Private Groups
- They are nodes running in background that supports its main node
  e.g. `move_group_private` supports `move_group`
- It manages private functionalities and internal operations that shouldn't be exposed directly to the user
- example tasks: parameter loading, configuration management
# Random number suffix
- Sometimes a node will be named with long integer number suffix
  e.g. `/interactive_marker_display_96498700456000`
- This is to uniquely identify the node within the ros ecosystem
- Usually some combination of timestamp, random number is used
- This is used if there is high change of running another node with same/similar name (eg. another interactive marker)
# Path vs. Trajectory
**Path**
- A path is a sequence of positions or waypoints that the robot needs to follow to move from the starting point to the goal.
- It specifies where the robot should be at each step

**Trajectory**
- A trajectory is a more detailed plan. It includes not only the sequence of positions (the path) but also the **timing** of the movement, including the **velocity** and **acceleration** at each point along the path.
- This means a trajectory tells the robot not just where to go, but also **how fast** to move at each point and how to **accelerate or decelerate** smoothly.
# Planners
- Different types of planners available in Moveit
http://moveit.ros.org/documentation/planners/

# OMPL
- Open Motion Planning Library
- moveit tutorial [here](https://moveit.picknik.ai/main/doc/examples/ompl_interface/ompl_interface_tutorial.html)

# Plan and Execute
- When you press `Plan` in the rviz, following is output ![[Pasted image 20240904154403.png]]
- When you press `Execute` in the rviz, following is output ![[Pasted image 20240904154538.png]]


  

# msg
- trajectory_msgs/msg/JointTrajectoryPoint.msg
  ```
  double[] positions
  double[] velocities
  double[] accelerations
  double[] effort
  builtin_interfaces/msg/Duration time_from_start
  ```
- trajectory_msgs/msg/JointTrajectory.msg
  ```
  std_msgs/msg/Header header
  string[] joint_names
  trajectory_msgs/msg/JointTrajectoryPoint[] points
  ```
- moveit_msgs/RobotTrajectory Message
  ```
  trajectory_msgs/JointTrajectory joint_trajectory
  trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
  ```
# topics
- moveit_msgs/ExecuteTrajectory.action
  ```
  RobotTrajectory trajectory  # The trajectory to execute  
  ---  
  MoveItErrorCodes error_code  # Error code - encodes the overall reason for failure  
  --- 
  string state  # The internal state that the move group action currently is in  
  ```
- moveit_msgs/action/MoveGroup
  ```
	MotionPlanRequest request  # Motion planning request to pass to planner 
	PlanningOptions planning_options  # Planning options 
	---  
	MoveItErrorCodes error_code  
	# The full starting state of the robot at the start of the trajectory  
	moveit_msgs/RobotState trajectory_start  
	  
	# The trajectory that moved group produced for execution  
	moveit_msgs/RobotTrajectory planned_trajectory  
	  
	# The trace of the trajectory recorded during execution  
	moveit_msgs/RobotTrajectory executed_trajectory  
	  
	# The amount of time it took to complete the motion plan  
	float64 planning_time  
	---  
	string state # The internal state that the move group action currently is in
	```
- 
- 

# Move Group Interface
- This helps to interact/interface with the `move_group` node!
## Following Tutorial of moveit c++ interface
- planning group = joint model group
- `PlanningSceneInterface` is a class that allows you to interact with the robot's environment (i.e. planning scene)
	- add/remove objects (obstacles) in scene
	- attach objects to robot
	- get info about the scene
- **planning frame** refers to the coordinate frame used as the reference for motion planning. It's the "origin" or main frame where planning computations are anchored.
- **end effector link** is by default the last link of kinematics chain of the robotic arm

# Moveit Config Builder
- `moveit_config_builder.py` can help to extract:
	- robot_description
	- robot_description_semantic
	- robot_description_kinematics
- By default it takes `robot_name` and searches for the package **`robot_name`\_moveit_config**. Or a package name can be provided directly
  ![[Pasted image 20240911114859.png]]
- This package is supposed to have all the moveit configs for the particular robot in standard ROS2 package format (i.e standard directory structure)
- C++ moveit interface tutorial uses **moveit_resources_panda** as package name. And when I searched for package **moveit_resources_panda_moveit_config**, the pkg did exist.
  ![[Pasted image 20240911115114.png]]
- In fact, I found multiple **\*\_moveit_configs** packages  on my machine
  ![[Pasted image 20240911115323.png]]
# Vscode setup
- Finally adding these include paths helps to reach out all the header files for vscode intellisense
  ![[Pasted image 20240911111752.png]]
- 
# ToDos
- study the action server wala code to run the UR5
- why collision object only shows up in moveit wala RViz and not on other one?
- 
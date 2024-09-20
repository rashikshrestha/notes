# API difference
![[Pasted image 20240815124108.png|500]]
- Python2 is not supported on ROS2
- C++11 and 14 will work on ROS2 my default
# Defining a Node
- In ROS1, there is no mentioning of how to write a node. But in ROS2, the node should be a class that should inherent 'Node' class
![[Pasted image 20240815125329.png]]
# Nodelets vs Components
- In ROS1, one executable file can contain only one Node.
- If more nodes is to be made within single executable file, it has to be done using Nodelets.
- Nodelets also enabled intra-process communication (i.e. communitation between nodelets within the same executable/process)
- In ROS2, this can be done using Components.
![[Pasted image 20240815130558.png]]
# lifecycle nodes
- This functionality is only found in ROS2 Humble and further.
- Lifecycle node is just one another Node running together with other Nodes.
- These are special nodes that controls the lifecycle of all other nodes.
- Each node can have 4 states: Unconfigured, Inactive, Active and Finalized
- Example
	- ROS2 Navigation Stack (Nav2) runs multiple nodes.
	- And some node might fail affecting the overall stack
	- Lifecycle node ensure that the stack can successfully execute without missing any nodes
	- Whenever a single node fails to transition to the configured state, other nodes are also blocked from becoming active
	- That means initially it waits for all the nodes to change from Unconfigured state to Inactive state, and then change all the nodes all to Active state all at once.
	- May be during the mid of execution, if any node fails, it changes it to inactive state and then stops all other nodes as well!
# Launch files
- Launch files allow you to start all your nodes from a single file!
- ROS1 uses XML, whereas ROS2 uses Python for launch files
- ROS2 also has a XML api, so you can as well use xml files for ros2 launch files
# ROS Master
- No more ROS Master in ROS2
# Parameters
- In ROS1, parameters are handled by Parameter Server, which is a part of ROS master
  ![[Pasted image 20240815133039.png|400]]
- In ROS2, parameters are specific to a Node. So, parameters are defined by a Node and are destroyed when the Node is Killed.
  ![[Pasted image 20240815132843.png|400]]
- ROS2 implement parameters as a service 
- 
# Services
- ROS topics just pumps and pulls the information to and from a big pool.
- But we may need a way to get information only when requested.
- This is done using ROS service.
- They consist of two message types: One for requesting data, and one for the response. 
- They are gateway to event-based ROS executions
- ROS1: Synchronous. They stop the execution of program flow until the response has been received.
- ROS2: Asynchronous (with an option to make it Synchronous)
# Actions
![[Action-SingleActionClient.gif]]
- In msg format: service has 2 section, whereas action has 3
- Where ROS actions asynchronous in ROS1? what about in ROS2?
- While sending action via command line, the formatting of the data string is very important
  for e.g.: 
  `ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"` this works
  but,
  `ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta:1.57}"` this doesn't work
- Notice a **missing space** between theta: and 1.57 !!!
- ros2 actions uses the ros2 service for sending goal and result, and ros2 topic for sending feedback
- That being said, it will not be available in `ros2 topic list` and `ros2 service list` like other topics and services
- 

# msg, srv, action 
![[Pasted image 20240815180430.png|500]]
![[Pasted image 20240815180555.png|500]]
# Interface
- Shows information about ROS interfaces
- For example: to see format of ROS msg
# Quality of Service (QoS)
- Not customizable in ROS1 but can be customized in ROS2
# Building package
- ROS1: catkin_make, catkin_build
- ROS2: ament, colcon build
- ROS2 package can be of 2 types: C++ or Python
  ![[Pasted image 20240815182141.png|400]]
- Both have package.xml
- ROS2 cmake package is more similar to ROS1
- Cmakelist.txt is replaced my setup.py
- ROS2 package that uses both C++ and Python needs a bit more work!! Don't know how yet!
# ROS overlays
- If a package has same name on lower level overlay and higher level overlay, then only the higher level package will be used
- Use case: if you have bunch of default packages in ros2, but only want to use updated version of a single package. Then you can create an overlay of just the specific package you want, using the same name as used in the base/default ros2 packages
# colcon
## colcon_cd
Installation `sudo apt install python3-colcon-common-extensions`

Add following to zshrc:
```bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.zsh
```
# ROS dir structure
workspace -> packages -> executables
# Building packages
Build type -> ament
Build tool -> colcon

# ros2 run
- Command format: `ros2 + run + name of the package + name of the executable`
- If the executable file (i.e the code part) has **Node** written in it, it will be shown in the `ros2 node list`
- 
# rosdep
- `rosdep` is a command-line tool that helps you install system dependencies for your ROS2 packages
- It simplifies the process of setting up your environment by automatically resolving and installing the necessary dependencies that your ROS2 project needs to run on your OS
- what is rosdep mentioned [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#build-and-run)?
- can be used to check if dependencies are okay before colcon build ?
# vcs tool
- VCS: Version Control System
- TODO....
- 
# Topics
## `/parameter_events` topic
- `/parameter_events` topic is a special system topic that is used for broadcasting changes in parameters on a node
- This topic allows nodes to communicate any changes to their parameters, making it possible for other nodes to be aware of these changes and potentially act upon them
- It goes from every node to every other nodes ![[Pasted image 20240817104607.png]]
# Create first package
```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name <executable_name> <package_name>
```
# Publisher
- What is Queue length while publishing and subscribing?
- If queue length is 10, and the subscriber stops receiving the message, then publisher will stop after 10 unread messages?
## Bare Minimum Publisher
```python
#! ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#! Node class
class SimpleTalker(Node):
    def __init__(self):
        super().__init__("simple_talker") # Name of the Node
       
        #! Create a Publisher 
        self.my_publisher = self.create_publisher(String, 'random_name', 10)
        
        #! Create a Timer
        self.my_timer = self.create_timer(0.5, self.my_timer_callback)
        
    def my_timer_callback(self):
        my_msg = String()
        my_msg.data = "hahahahaha"
        self.my_publisher.publish(my_msg)
        
#! Main Function
def main():
    rclpy.init()
    simple_talker = SimpleTalker()
    rclpy.spin(simple_talker)
    simple_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
# Subscriber
# Launch file
- Launch file can be in python, xml or yaml format! (Python prefered)
- Everything you need to know about ros2 launch [here★](https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/)
- ![[ros2_launch.excalidraw]]
- Python launch file must have `generate_launch_description()` function which should return `launch.LaunchDescription`
## Conventions
- `*_launch.py` is proper convention for writing launch files.
- launch file should be inside launch dir, and launch dir should be kept inside the ros package, along with package.xml file

- Follow [this](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html) tutorial to create launch file
## colcon_build for launch files
- While doing `colcon_build`, it won't reconize the launch files directly, and the launch files will not be installed in the `install` dir.
- For the build system to recognize the launch files, `setup.py` should be edited to append `(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),` line on setup->data_files
- More stuffs on launch file:
	- DeclareLaunchArgument: to declare launch args
	- LaunchConfiguration: to extract the declated launch args
## Actions
General:
- DeclareLaunchArgument (imp)
- ExecuteProcess
- SetEnvironmentVariable
- LogInfo
- GroupAction
- OpaqueFunction: Dynamically set the nodes to execute during runtime. It can wrap a function which outputs list of nodes to execute

ROS Specific:
- Node (imp)
- LifecycleNode
- SetParameter
- SetParametersFromFile
- ROSTimer
- SetROSLogDir
- SetUseSimTime
## Substitutions
ROS specific
- FindPackageShare

General:
- Command: Gets the output of any linux command as a string
- FindExecutable: Tries to locate an executable on the $PATH
- LaunchConfiguration: Access configuration variables, declared by DeclareLaunchArgument
- PathJoinSubstitution: To Join Path
- AndSubstitution
- OrSubstitution
- NotSubstitution

And,Or,NotSubstitution was introduced on Humble.
## Conditions
- IfCondition: This is if statement
- UnlessCondition: This is else statement (without if statement, weird!!! )
- LaunchConfigurationEquals: Check if particular launch configuration equals to something ★
- LaunchConfigurationNotEquals: Not equales

Example usecase of Conditions is on the LogInfo action.
```bash
LogInfo(condition=IfCondition(use_gui), msg="GUI is being used!")
```
Similarly also in GroupAction action
```bash
GroupAction(condition=<any_condn_here>, actions=[list,of,actions,here])
```
# Node `robot_state_publisher`
![[Pasted image 20240818075932.png]]
- More on the [readme](https://github.com/ros/robot_state_publisher) file.
- Robot state publisher is responsible for publishing the exact configuration of the robot.
- This is combination of static and dynamic part!
- `/robot_description` topic is static and is published only once at the beginning ! It is nothing but the complete URDF file of the robot in simple String format
- `/tf_static` is the transforms published using the URDF file itself
- `/tf` is dynamic. It gives the transformation to the already available static TF using the data obtained from `/joint_states` topic
- The Parameter: `robot_description` (string) - The original description of the robot in URDF form. This _must_ be set at robot_state_publisher **startup time**, or the node will fail to start. Updates to this parameter will be reflected in the `robot_description` topic.
# Node `joint_state_publisher`
- This node is responsible for publishing `/joint_states` topic, which is the state of each and every joint of the robot!
- Normally sensors of the robot are responsible for generating `/joint_states`
- So, `/joint_state_publisher` node is mainly used in the simulation
- joint_state_publisher takes in the `/robot_description` topic to figure out the joints for which it should publish the `joint_states` and does so!
- If you run `joint_state_publisher` without `robot_description`, if keeps waiting for it ![[Pasted image 20240818152624.png]]
- Also, there is `joint_state_publisher_gui`, which helps provides you easy GUI to change the joint state values
# Transforms
- View transforms `ros2 run tf2_tools view_frames`
- This will listen to `/tf` `/tf_static` topic for 5 seconds and generate a pdf file of transforms in the same dir where you've launched the command 
# Executors
- Documentation [here](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html#executors)
- Good article [here](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-5-concurrency-executors-and-callback-groups-c45900973fd2)
- ![[Pasted image 20240820224100.png]]
- The default_callback_group is a ==Mutually Exclusive== Callback Group. This means that callbacks within this group can't run simultaneously.
- **SingleThreadedExecutor** will only execute callbacks one at a time, so the concurrent capabilities of `ReentrantCallbackGroup` won't be utilized.
- So if you want to run multiple callbacks together, then you should use `MultiThreadedExecuter` together with `ReentrantCallbackGroup`

# Custom msg
- Creating custom msg is not possible using python ROS2 package as mentioned [here](https://www.reddit.com/r/ROS/comments/15miocl/how_to_use_custom_messages_when_using_python/)
# Calling service from C++ class
- See issue [here](https://answers.ros.org/question/340389/client-doesnt-return-when-declared-inside-c-class-in-ros-2/)]
- 
# To Know
- apt install python3-colcon-common-extension usage ?
- 


# ToDos
- Learn about the Executers. why can't we subscribe two topics from a single node ?
- Learm about ros2_control
- 
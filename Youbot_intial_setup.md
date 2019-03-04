# Setting up KUKA Youbot

This tutorial contains instructions on how to set up the KUKA Youbot onboard computer.

## Installing the operating system and ROS

First you will have to install Ubuntu 16.04 on the computer. There should be a flash drive with Ubuntu 16.04 in the room. If not, you can download it from [here](https://www.ubuntu.com/download/alternative-downloads) and put it on a flash drive.

- Install Ubuntu 16.04 using [this tutorial](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0).

- Install the full desktop version of ROS Kinetic using [this tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu).

## Making a workspace

First you will have to make a catkin workspace for your ROS packages and compile it.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now you will have to source the setup file. So you don't have to source it every time you open a new terminal we will write the source command in the .bashrc file.

```bash
$ sudo gedit .bashrc
```

This will open the text editor. You will need to add this line to the end of the file:

```bash
source ~/catkin_ws/devel/setup.bash
```

Save the file and exit.

## Installing ROS packages needed for running the robot

First we will install some packages with apt-get. Fist we will install a driver for the Youbot.

```bash
$ sudo apt-get install ros-kinetic-youbot-driver
```

Next we will install packages that are dependencies for other Youbot packages that we will compile in the catkin workspace.

```bash
$ sudo apt-get install ros-kinetic-pr2-msgs ros-kinetic-brics-actuator ros-kinetic-moveit
```

Next we will download needed Youbot packages to the catkin workspace with git. First we will need to download the git package.

```bash
$ sudo apt-get install git
```

Now go to catkin_ws/src and download the packages there with git.

```bash
$ cd ~/catkin_ws/src
$ git clone http://github.com/youbot/youbot_description.git -b kinetic-devel
$ git clone https://github.com/youbot/youbot_driver_ros_interface.git -b indigo-devel
$ git clone https://github.com/svenschneider/youbot-manipulation.git -b indigo
```

So that all the packages will compile in kinetic, we need to make some changes in a file in the youbot_manipulation package.

```bash
$ cd ~/catkin_ws/src/youbot_manipulation/youbot_arm_kinematics_moveit/
```

Open the CMakeLists.txt file.

```bash
$ sudo gedit CMakeLists.txt 
```

And add this line before *find_packages*:

```bash
add_compile_options(-std=c++11)
```

Now you should be able to compile the workspace.

```bash
$ cd ~/catkin_ws
$ catkin_make
```

## Setting up the EtherCAT connection

First you will have to set the required rights for EtherCAT access.

```bash
$ cd ~/catkin_ws
$ sudo setcap cap_net_raw+ep devel/lib/youbot_driver_ros_interface/youbot_driver_ros_interface
```
 Next you will have to set up the right ethernet device as the etherCAT connection. Enter the command to see all connected ethernet devices.
 
 ```bash
$ ifconfig
```
 
 You should see multiple devices. Since the EtherCAT is connected to an external USB network card, the name should be something like INSERT DEVICE NAME. This is your EthernetDevice.
 
 Now open the Youbot EtherCAT configuration file.
 
 ```bash
$ sudo gedit /opt/ros/kinetic/share/youbot_driver/config/youbot-ethercat.cfg
```
There is a line that says:

 ```bash
EthernetDevice = eth0
```

Instead of eth0, insert the device name that you got from ifconfig.

## Making a launch file to move the robot arm

In the default youbot_manipulation packge there is a demo file which simulates the Youbot, so to make things easier we will make a separate file to run the real Youbot.

First go to the launch folder where the file is:

 ```bash
$ cd ~/catkin_ws/src/youbot_manipulation/youbot_moveit/launch
```
 
 Open the demo file and copy the contents.
 
```bash
$ sudo gedit demo.launch
```

Make a new file.

```bash
$ gedit real_demo.launch
```

Paste the copied contents into the file and delete the following lines.

```bash
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="/use_gui" value="false"/> 
    <rosparam param="/source_list">[/move_group/fake_controller_joint_states]</rosparam>
</node>
```

Modify the value from "true" to "false" in the following line to disable fake execution:

```bash
<arg name="fake_execution" value="true"/>
```

Save the file and close.

## Using LIDAR with urg_node and youbot_navigation

Tutorial and which packages to install is [here](https://github.com/ut-ims-robotics/tutorials/wiki/Navigation-stack).

In addition you will have to install urg_node:

```bash
$ sudo apt-get install ros-kinetic-urg-node
```

The default youbot_navigation package uses hokuyo_node for the LIDAR which is not supported on ROS Kinetic, so we switched over to urg_node. So that the package will work with urg_node, you will have to change a launch file.

Open the file.

```bash
$ cd ~/catkin_ws/src/youbot_navigation/youbot_navigation_common/launch
$ sudo gedit base_front_hokuyo_node.launch
```

Change the file contents to this:

```bash
<launch>

	<!-- start the node -->
	<node type="urg_node" pkg="urg_node" name="urg_node">  
		<!--param name="min_ang" type="double" value="-1.3" /-->
		<!--param name="max_ang" type="double" value="1.3" /-->
		<remap from="/scan" to="/base_scan"/>
		<param name="serial_port" type="string" value="/dev/ttyACM1"/>
	</node>

	<!-- configure the transform -->
	<node pkg="tf" type="static_transform_publisher" name="front_hokuyo_frame" 
	args="0 0 0 0 0 0 /base_laser_front_link /laser 1" />
	
</launch>
```


 










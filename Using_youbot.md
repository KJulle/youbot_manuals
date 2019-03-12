# Using KUKA Youbot

Instructions on how to move the arm and drive around with the robot.

## Powering the robot

Before turning on the robot you will have to connect the external power supply or battery. If you're not planning to drive the robot around, you should use only the external power supply to power the robot. The external power supply connects to a 24V input on the top of the base of the robot. The battery slides into the slot on the side of the robot and connects to a connector there.

To charge the battery connect both the battery and external power supply to the robot at the same time.

**NB! Always disconnect the battery when not in use.**

## Turning on the robot

Press the "ON/OFF"-button for a few seconds to turn on the power for the system. An orange light on the display will indicate that the System has been powered on successfully. Now you can turn on the motors and/or PC. To do this, press and hold the power button and when the screen shows PC on release the button to turn on the PC. Repeat the same step, only now release the button when the screen shows Motor on.

The arm has to be turned on separately. When the motors are on, there should be a button on the arm with a red light. To turn on the arm, press the button. The light is green when the arm is turned on.

**NB! When you turn off the power of the arm, it will NOT hold its position and will collapse, so make sure to support the arm when turning it off.**

To turn off the system press and hold the same button and release when the screen shows System off. You should also power off the PC manually before turning off the system.

If you are logged in with ssh, you can do this by entering:

```bash
$ sudo shutdown -h now
```

## Setting up a remote computer

To run the robot remotely you need to set up another computer/laptop with Ubuntu 16.04 and ROS Kinetic on it(there should be a suitable laptop in the room).

--installing packages

**NB! For the next steps the robot and your remote computer have to be in the same network.**

By default the robot should be in the ??? network.

Now you will have to set up the remote connection. For that you will need to know the Youbot's and the remote computer's IP addresses.

To find the Youbot's IP address enter this command to the terminal:

```bash
$ ifconfig
```

Now you should see multiple ethernet devices. The one you need should be wlp2s0's inet addr which should be something like 192.168.x.x.

To get your remote computer's IP, you need to repeat the same step, only the ethernet device name is probably different.

If you have both IP addresses, you will have to specify ROS master aka Youbot IP address on the remote computer:

```bash
$ export ROS_MASTER_URI=http://192.168.x.x:11311
```

Next, specify the remote computer's own IP:

```bash
$ export ROS_IP=192.168.x.x
```

Modify /etc/hosts file in order to resolve master's name to IP:

```bash
$ sudo gedit /etc/hosts
```

Add the following line to the end of the file:

```bash
192.168.x.x    youbot
```

## Establishing an ssh connection

Now you can use ssh on your remote computer to log into the Youbot's onboard computer.

To establish an ssh connection, enter this to your remote computer's terminal: 

```bash
$ ssh youbot@192.168.x.x
```

Now you should be logged into the onboard computer. That means that when you enter commands in the terminal window you have the ssh connection in, the commands will be run on the robot's onboard computer.

If you need more terminal windows with an ssh connection, just use the same command above to  log in.

## Driving the robot

To control the robot wheels with your keyboard you have to first run the Youbot driver. This also launches roscore, so you don't have to do that yourself. Run this on the onboard computer.

```bash
$ roslaunch youbot_driver_ros_interface youbot_driver.launch
```

To control the robot, run keyboard teleop.

```bash
$ rosrun youbot_driver_ros_interface youbot_keyboard_teleop.py
```

Now you should be able to control the wheels with your keyboard. Note that the keyboard teleop only works when the terminal window where you gave the command is active.

## Moving the robot arm

You must have the youbot driver running before doing any of the following commands.

To move the arm, enter:

```bash
$ roslaunch youbot_moveit real_demo.launch
```

By default this will open rviz, where you should now be able to see the robot model.

**NB! Make sure the robot model arm position in rviz is the same as in real life before proceeding.**

To move the arm, first you can drag the marker on the top of the robot arm in rviz to the wanted position. In the conext tab you can select a planning library(for example RRTkConfigDefault). 

Next, on the planning tab press *Plan*. Now you can see the trajectory the arm will make when moving into the wanted position. Make sure that the planned trajectory will not damage the robot.

To execute the planned trajectory, press *Execute*.

**Note:** Every time you plan/execute the trajectory, make sure that  "Select Start State" option is "current", in section "Query" under tab "Planning". Otherwise the robot will plan from previous starting state, which may end up with damaging movement.


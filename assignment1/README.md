# Experimental robotics laboratory - Assignment 1

## Introduction
This is the repository for the first assignment of the Robotics engineering "Experimental robotics laboratory" course, a.y. 2020/2021, held in the University of Genoa. This work is focused on the implemetation of a ROS-based finite state machine (FSM) nested within a more complex architecture. This project has been coded with [Python 2.7](https://www.python.org/download/releases/2.7/) and it has been tested on [ROS Kinetic](http://wiki.ros.org/kinetic).

## Setting
The theme is based on a robot (MiRo), which wanders around and plays with the user. Given a configurable 2D playing ground, MiRo moves randomly around, until it decides to go to sleep in his house (a configurable predetermined location). Then he wakes up and starts its normal behaviour again (i.e. wandering). An user can interact with the robot, asking him to <<**play**>> a simple game: if MiRo is awake, it will reach the user location (yet another configurable parameter) and then wait for him to send him to a random point via a pointing geture. After some time MiRo gets bored, and goes back to its normal state. 

## Architecture
The architecture is made of three components:
- **Person**: it is implemented by [perception.py](src/perception.py), which is used to simulate reception and processing of incoming voice commands and pointing gestures
- **Command manager**: it is implemented by [command_manager.py](src/command_manager.py), which is used to handle the robot's finite state machine internal architecture
- **Control**: it is implemented by [control.py](src/control.py), which is used to emulate physical delays relative to robot travelling and position reaching

<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment1/images/architecture%20and%20topics.png">
</div>

The first one simulates a person interacting with MiRo; the user can say <<**play**>> in order to ask the robot to play a simple game, and can use pointing gestures to indicate MiRo a location to reach. Then the Command manager is the component which is devoted to the implementation of the finite state machine representing the robot behaviours. Finally, the Control component has the duty of simulating physical delays that, in real life, a robot would require in order to move from one position to another one. All the comunications between components are carried out via publish-subscribe techniques.<br/>
This is the intenal structure of the finite state machine:
  
<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment1/images/fsm%20states.png">
</div>

MiRo starts in the **Sleep** state.<br/>
Topics involved:

- `control_topic`: topic used vy the FSM to order the **Control** component to start simulating a movement
- `gesture_topic`: used to send the pointed location by the user to the FSM
- `motion_over_topic`: topic whose duty is to inform the FSM when the simulated motion is assumed to be over
- `play_topic`: topic used to inform the FSM whenever the user says <<**play**>>

The message types used are:

- `std_msgs/String`:"stock" message type, composed by a simple string
- `assignment1/Coordinates`: message made of two integers **x** and **y**

The latter of the two is of course a custom one, which has been coded for this project and is shipped with this package itself. Standard messages could have been used, but this new type creation had also been created as an extra exercise, in order to get more acquainted with the ROS environment.<br/>
Finally, here are all the parameters loaded in the ROS parameter server:

- `state`: parameter specifying robot current state
- `MiRo/x`: parameter specifying robot current position (x coordinate)
- `MiRo/y`: parameter specifying robot current position (y coordinate)
- `map/x_max`: parameter specifying the maximum x-axis value for the map
- `map/y_max`: parameter specifying the maximum y-axis value for the map
- `home/x`: parameter specifying robot home position (x coordinate)
- `home/y`: parameter specifying robot home position (y coordinate)
- `person/x`: parameter specifying user's position (x coordinate)
- `person/y`: parameter specifying user's position (y coordinate)
- `sim_scale`: parameter used to scale simulation velocity

All of these, as already pointed out, can be adjusted before runtime.

## Requirements
In order to run this piece of software, it is needed to have a Linux distribution that supports (and has them installed):
- [ROS Kinetic](http://wiki.ros.org/kinetic)
- [SMACH](http://wiki.ros.org/smach)
- A [Python](https://www.python.org/) interpreter

## Instructions
It might be needed to make all the nodes executable, before actually being able to run them. So, after reaching this assignment's folder via a terminal, type:

```
chmod +x src/*
```

In order to run the code, simply open a terminal, move to the *assignment1* directory (which should be put in a catkin workspace), and type:

```
roslaunch assignment.launch
```

This will launch all the three nodes with the parameters defined in the launche file itself, with the addition of the SMACH viewer tool, which is useful to visually keep track of the FSM state transitions. The parameters inside the launch file can be adjusted as desired.

## Assumptions
In order to simplify the problem, it is assumed that the user is aware of the robot current state, and that they will only ask MiRo to play when necessary conditions are met (i.e. when it is in **Normal** state). Of course, all the wait times are random and do not resemble reality in a correct way: there has not been made any consideration about the fact that, for instance, a longer movement, distance-wise, takes a greater amount of time in the real world, wit respect to a shorter trip. So another assumption is that travel distances are not directly linked with the time the robot takes to perform them. It is also assumed that the user does not try to "break" the robot, as in fact its simulation is only allowed to say the <<**play**>> keyword and the to point a location in the playing ground. Finally, the user is assumed to use gestures only when the robot is in **Play** behavior, and to point only after the voice command.

## Limitations
Most, if not all, of the assumptions descripted above are indeed limits of the implementation. In addition to those, there is little to no control over input parameters (e.g. one can set a starting robot position outside of the playing field).

## Extra
[MiRo](http://consequentialrobotics.com/miro-beta#:~:text=MiRo%20is%20a%20fully%20programmable,suited%20for%20developing%20companion%20robots.) is the robot that has been shown us in the images during the assignment presentation, and that we would probably have used if we could have worked in the University. Sadly, due time is still inside a COVID-19 outbreak, so no chance to use it.

## Authors
All the files in this repository belong to [Andrea Pitto](https://github.com/andreabradpitto) - [s3942710@studenti.unige.it](mailto:s3942710@studenti.unige.it).

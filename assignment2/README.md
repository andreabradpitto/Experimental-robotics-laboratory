# Experimental robotics laboratory - Assignment 2

## Introduction
This is the repository for the second assignment of the Robotics engineering "Experimental robotics laboratory" course, a.y. 2020/2021, held in the University of Genoa. This work is focused on the implemetation of a ROS-based finite state machine (FSM) nested within a more complex architecture. This project has been coded with [Python 2.7](https://www.python.org/download/releases/2.7/) and it has been tested on [ROS Kinetic](http://wiki.ros.org/kinetic).

## Setting
The theme is based on a robot (MiRo), which wanders around and plays with the user

## Architecture
The architecture is made of three components:
- **Person**: it is implemented by [perception.py](src/perception.py), which is used to simulate reception and processing of incoming voice commands and pointing gestures
- **Command manager**: it is implemented by [command_manager.py](src/command_manager.py), which is used to handle the robot's finite state machine internal architecture
- **Control**: it is implemented by [control.py](src/control.py), which is used to emulate physical delays relative to robot travelling and position reaching

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

## Assumptions
In order

## Limitations
Most, 

## Extra
[MiRo](http://consequentialrobotics.com/miro-beta#:~:text=MiRo%20is%20a%20fully%20programmable,suited%20for%20developing%20companion%20robots.) is the robot that has been shown us in the images during the assignment presentation, and that we would probably have used if we could have worked in the University. Unfortunately, since the project's deadline still falls inside a high peak COVID-19 outbreak phase, there is no chanche we will use it.

## Authors
All the files in this repository belong to [Andrea Pitto](https://github.com/andreabradpitto).<br/>
Contact: [s3942710@studenti.unige.it](mailto:s3942710@studenti.unige.it).

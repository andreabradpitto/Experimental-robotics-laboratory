# Experimental robotics laboratory - Assignment 2

## Introduction
This is the repository for the second assignment of the Robotics engineering "Experimental robotics laboratory" course, a.y. 2020/2021, held in the University of Genoa. This work is focused on the implemetation of a ROS-based finite state machine (FSM) nested within a more complex architecture. This project has been coded with [Python 2.7](https://www.python.org/download/releases/2.7/) and it has been tested on [ROS Kinetic](http://wiki.ros.org/kinetic).

## Setting
The theme is based on a robotic dog, which wanders around and plays with the a ball moved or hidden by the user. Through [Gazebo](http://gazebosim.org/), the idea is to build upon the finite state machine created for the [Assignment 1](https://github.com/andreabradpitto/Experimental-robotics-laboratory/tree/main/assignment1) and implement a graphical version of the moving robot, while also modifying its behaviors, to better match its new "dog" representation. The final result should indeed loosely resemble a real pet.<br/>
The world used, which is delimited by fences, is a 10x10 square and can be referred to as the dog playing field. Inside the square, other than the robotic dog, there are also an human mannequing and its chair, which have been added just to give some flavour to the environment. The dog has its home (see Sleep state) near the human model by default, but that can be changed in the parameters. Here is an image of the playing field:

!!!inserisci immagine campo di gioco

This is the model of the robotics dog:

!!!inserici immagine cane

It loves big green balls

## Architecture
The architecture is made of five components:
- **Person**: it is implemented by [perception.py](src/perception.py), which is used to simulate the dog owner. This component randomly moves along the playing field, or hides the ball from sight
- **Command manager**: it is implemented by [command_manager.py](src/command_manager.py), which is used to handle the robot's FSM internal architecture
- **Control**: it is implemented by [control.py](src/control.py), which is used to emulate physical delays relative to robot travelling and position reaching
- **Go to point ball**: it is implemented by [go_to_point_ball.py](src/control.py), which is used to finalize ball movements issued by perception.py
- **Control**: it is implemented by [control.py](src/control.py), which is used to emulate physical delays relative to robot travelling and position reaching


!!!immagine della fsm

The dog starts in the **Sleep** state.<br/>
Topics involved:

- `control_topic`: topic used by the FSM to order the **Control** component to start simulating a movement
- `gesture_topic`: used to send the pointed location by the user to the FSM
- `motion_over_topic`: topic whose duty is to inform the FSM when the simulated motion is assumed to be over
- `play_topic`: topic used to inform the FSM whenever the user says <<**play**>>

!!!immagine dell'architettura e topics

The message types used are:

- `std_msgs/String`:"stock" message type, composed by a simple string
- `assignment1/Coordinates`: message made of two integers **x** and **y**

The latter of the two is of course a custom one, which has been coded for this project and is shipped with this package itself. Standard messages could have been used, but this new type creation had also been created as an extra exercise, in order to get more acquainted with the ROS environment.<br/>
Finally, here are all the parameters loaded in the ROS parameter server:

- `state`: parameter specifying robot current state
- `dog/x`: parameter specifying robot current position (x coordinate)
- `dog/y`: parameter specifying robot current position (y coordinate)
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

In order to run the code, put the *assignment2* directory in your workspace, build, then open a terminal and type:

```
roslaunch assignment.launch
```

## Assumptions
In order

## Limitations
Most, 

## Authors
All the files in this repository belong to [Andrea Pitto](https://github.com/andreabradpitto).<br/>
Contact: [s3942710@studenti.unige.it](mailto:s3942710@studenti.unige.it).

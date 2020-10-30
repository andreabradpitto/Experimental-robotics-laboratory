# Experimental robotics laboratory - Assignment 1

## Introduction
This is the repository for the first assignment of the Robotics engineering "Experimental robotics laboratory" course, a.y. 2020/2021, held in the University of Genoa. This work is focused on the implemetation of a ROS-based finite state machine (FSM) nested within a more complex architecture. This project has been coded with Python 2.7 and it has been tested on ROS-kinetic.

## Setting
The theme is based on a robot (MiRo), which wanders around and plays with the user. Given a configurable 2D playing ground, MiRo moves randomly around, until it decides to go to sleep in his house (a configurable predetermined location). Then he wakes up and starts its normal behaviour again (i.e. wandering). An user can interact with the robot, asking him to <<play>> a simple game: if MiRo is awake, it will reach the user location (yet another configurable parameter) and then wait for him to send him to a random point via a pointing geture. After some time MiRo gets bored, and goes back to its normal state. 

## Architecture
The architecture is made of three components: Person, Command manager and Control:

<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment1/images/architecture.png">
</div>

The first one simulates a person interacting with MiRo; the user can say <<play>> in order to ask the robot to play a simple game, and can use pointing gestures to indicate MiRo a location to reach. Then the Command manager is the component which is devoted to the implementation of the finite state machine representing the robot behaviours. Finally, the Control component has the duty of simulating physical delays that, in real life, a robot would require in order to move from one position to another one. All the comunications between components are carried out via publish-subscribe techniques.
This is the intenal structure of the finite state machine:
  
<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment1/images/fsm%20states.png">
</div>

MiRo starts in the **Sleep** state.

## Instructions

```
rosrun stage_ros stageros $(rospack find exercise1)/world/exercise.world
rosrun exercise1 exercise1 _xt:=2.0 _yt:=3.0
rosrun pos_server position_server
```
<ins>[This](https://github.com/CarmineD8/exp-lab-exercises/tree/master/exercise1#monday-2809-exercise)</ins> is the official text for the assignment


## Authors
All the files in this repository belong to [Andrea Pitto](https://github.com/andreabradpitto).

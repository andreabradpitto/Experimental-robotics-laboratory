# Experimental robotics laboratory - Assignment 3

## Introduction

This is the repository for the third and last assignment of the Robotics Engineering "Experimental robotics laboratory" course, a.y. 2020/2021, held at the University of Genoa. This work is focused on the implemetation of a ROS-based finite state machine (FSM) nested within a more complex architecture. This project has been coded with [Python 2.7](https://www.python.org/download/releases/2.7/)/[C++](https://www.cplusplus.com/) and it intended to be run on [ROS Kinetic](http://wiki.ros.org/kinetic) and [Ubuntu 16.04](https://releases.ubuntu.com/16.04/).

---

## Setting

Like for the previous assignment, [Assignment 2](https://github.com/andreabradpitto/Experimental-robotics-laboratory/tree/main/assignment2), the main subject of the project is a robotic dog. This time, it is free to wander inside a house, composed by 6 distinct rooms: the entrance, the closet, the living room, the kitchen, the bathroom, and the bedroom. Each of these room locations is schematically represented by a colored ball lying on the ground, so that the dog is able to uniquely identify them. Indeed, in order to avoid ambiguities each ball has a different color; following the same order mentioned above, the colors are: blue, red, green, yellow, magenta and black. Even though the house is not procedurally generated (i.e. it has always the same layout and dimensions), every time the simulation is started, the robot is unaware of where these room are, so it has to learn the house planimetry by exploration. The only thing it knows is the association of each colored ball to its respective room, so that when the robotic dog sees e.g. the yellow ball, it knows that that room is the kitchen. Indeed, the colored balls are just a schematic representation of the peculiar feature of each room, that a real robo-dog is expected to be able to recognize.  
  
Inside the house there is, other than the walls defining each room's boundaries, also a human mannequin model, which represents the dog owner. The robot is in fact listening for human orders, and eager to play with them. The dog, in normal conditions, wanders randomly around the house, looking for undiscovered colored balls; when these are spotted, it reaches their location and stores the coordinates of the corresponding room, so that this piece of information can be used later on. When the human asks the robot to play (i.e., by actually telling the robot the word "play"), the dog reaches them, awaits for the human to tell a room name, and then it tries to reach it. Of course, diferent behaviors have been implemented depending on the fact that the the robot already knows the location it has been instructed to reach or not. When the robotic dog reaches the required room, it outputs a vocal acknowledgement, and finally gets back to the owner. When the robot gets tired, it gets back to its charging dock (in the future, also referred as "home") and waits for the battery to recharge. This behavior is the real-life-pet equivalent of sleeping; this parallelism is the one responsible for the state's actual name, as it will be shown later on in this document. The last behavior is the one devoted to the house exploration, for when the robot is looking for a particular still-to-be-discovered ball: this happens we the human wants it to reach a room that is not yet present in its database.  
  
The code implemented will show a real-time simulation of the assignment over both [Gazebo](http://gazebosim.org/) and [RViz](http://wiki.ros.org/rviz). The human model and the balls will be present only on the former, while the latter is the one that will show the robotic dog's perspective, i.e. all the explored map so far, and the discovered obstacles (the walls and/or the human model), along with more useful information. Gazebo will always show the whole house, so that it is possbile to check if the robot knowledge matches the real planimetry of the environment. In addition, the voice messages shared between the human and the robotic dog will be displayed on terminal.

### The house

Here is a picture of the house, acquired from the Gazebo simulation:

\image html "house.png"

It is comprised inside a 20x20 square and centered at the origin of the three axes. The environment can be thus effectively reduced to a bidimensional scenario, spreading over the xy plane (or, equivalently, on the z = 0 one). The robot initial position, which is also assumed to be its home location, is set, by default, in the launch file to be near the human mannequin: the latter is in *(-6, 8.5)*, while the former is in *(-5, 8)*. Please notice that in the above picture neither the origin axes nor the floor grid is being shown, in order to only present the house model itself; anyway, Gazebo automatically shows both of them with every new run.  
In the top left corner of the screenshot it is possible to see the human mannequin, near another decorative object, both colored in white in order to hinder the robot's room detection feature. The robotic dog is present too: its model is described in detail in the next subsection.

### The robotic dog

The robot I implemented is composed by basic geometric shapes, and is trying to find a medium between looks and functionality. Indeed, it features two eyes, in order to resemble a real dog, but only one of them holds a sensor, i.e., an RGB camera, which is used to spot the colored balls: the other one has no purpose other than giving some flavour to the model. The other sensor present on the robot is a laser scanner, which is mounted on its main chassis, and is used to detect the obstacles in front of it. It is positioned much lower with respect to the camera, in order to guarantee the best possible obstacle detection performances: the tests I made clearly showed that also having the laser scanner embedded in an "eye" of the robot was much harder to successfully implement.  
Here is a picture the robotic dog (again, acquired from the Gazebo simulation):

\image html "robot.png"

It is a two-wheeled differential drive robot. Apart from the wheels, all the joints are fixed, so the dog cannot, for intstance, rotate its head to look around (unlike in the [previous assignment](https://github.com/andreabradpitto/Experimental-robotics-laboratory/tree/main/assignment2)). Notice the orange cube on the front, which is the shape representing the above mentioned laser scanner.

### The environments

In addition to the window showing what the robot sees in real time, this package launches two separate enviroments: Gazebo, which is used to simulate the physics of the robot, and Rviz, which is a flexible visualization tool, also being able to show all the topics involved in the package execution. One of the most important difference between the two, atleast in this package, is that while in Gazebo one can watch the whole house and the robot with no constraints, in RViz I made it so that one only sees what the robot knows, and has discovered so far:

\image html "RViz.png"

On the left side of the above image there are many the elements useful to track and assess the robot self localization, mapping and exploration capabilities. The image in the middle also shows a lot important of information:

- the black elements are the (inflated) shapes of obstacles detected by the robot (i.e., walls, decorative elements and the colored balls)
- the red lines are the current wall detections incoming from the robot's laser
- the magenta arrow is the current goal of the robot (shown in blue in the picture, in the bottom right corner)
- the green line is the local navigation plan
- the yellow line is the global navigation plan
- the blue segments and the green spheres are the elements used by the exploration algorithm (only applies to the Find state)
- the three white squares are, in size-wise increasing order, the local costmap, the global costmap. and the currently comupted world map

The computed world map is acquired by reading from the `/map` topic, which is published by the [gmapping](http://wiki.ros.org/gmapping) laser-based [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) algorithm. It relies on the laser scan data in order to chart the house walls.

Finally, here is a sample image of a possible terminal output provided by the package:

\image html "terminal.png"

Run the assignement to discover more phrases!

---

## Architecture

<div align="center">
  <img src="architecture.png" width="900">
</div>

(Please notice that, in the above picture, the two environments are not reported with their full input for the sake of simplicity)  
  
This is the list of the components that have been coded for the assignment:

- *Human*: it is implemented by [human.py](scripts/human.py), that is used to simulate the dog owner, which randomly decides to play with the robotic dog. The human checks if the robot is able to play; if so, waits for it to come nearby, then orders it to move to a random room of the house, and finally waits for it come back. The game ends when the robotic dog gets tired. All the communications sent to the robot are carried out via a message publisher (over the `/play_topic`), and all the orders are handled by [dog_fsm.py](scripts/dog_fsm.py)
- *Dog FSM*: it is implemented by [dog_fsm.py](scripts/dog_fsm.py), which is used to handle the robotic dog's FSM internal architecture
- *Dog vision*: it is implemented by [dog_vision.py](scripts/dog_vision.py), and constitutes a vision module for the robotic dog that uses OpenCV in order to constantly scan the surroundings, looking for specific colored balls. This node is able to take control, when needed, of the robot movements, allowing it to reach a room when a corresponding new ball is discovered. It also stores the positions of the balls discovered, thus learning the displacement of the rooms inside the house as time passes. A simple algorithm has been added to the script in order to unstick the robotic dog as the ball chasing behavior may cause a collision with other entities in the house
- *A modified version of the [explore_lite](http://wiki.ros.org/explore_lite) package*: it is implemented by [explore.cpp](src/explore.cpp), [costmap_client.cpp](src/costmap_client.cpp), and [frontier_search.cpp](src/frontier_search.cpp), along with their headers: [explore.h](include/explore.h), [costmap_client.h](include/costmap_client.h), [frontier_search.h](include/frontier_search.h), [costmap_tools.h](include/costmap_tools.h). It is the algorithm allowing the robot to explore the house during the **Find** state. I made some adjustments in order to let [dog_vision.py](scripts/dog_vision.py) and [dog_fsm.py](scripts/dog_fsm.py) send service calls in order to (re-)start or stop the exploration

\image html "fsm.png"

The dog starts in the **Sleep** state.  
In the **Sleep** state, the robot goes to sleep once it gets back home. It relies on the [move_base](http://wiki.ros.org/move_base) algorithm in order to move in the environment, and it does so by implementing an action client and asking the server to reach the coordinates corresponding to home.  

In the **Normal** state, the robot wanders randomly, by feeding the move_base algorithm with randomly generated positions. If, while going around, the robot detects a new room/ball, it reaches it and stores the corresponding position. At any time, a play command can be received by the human: if so happens, the robotic dog transitions to the Play state. Every newly detected room, along with its subsequent data collection process, consumes robot battery. If the battery gets depleted, the robot goes to sleep, by transitioning to the **Sleep** state.  

In the **Play** state, using the move_base algorithm, the robotic dog gets back home (i.e., close to the human), then starts listening for the room request. Once received, it checks which is the corresponing ball color to reach. If the ball is not yet in the dog's database, it shifts into the Find state. If the ball had instead been seen before, the dog starts heading towards the chosen room, while still relying on move_base. After reaching that position, it comes back to the user, once again using move_base: whenever this process is completed, a portion of battery is consumed. Furthermore, I assumed that a single cycle of this state consumes slightly more battery, on average, than one of the **Normal** state; for this reason, the threshold of remaining battery before going back to the **Normal** state is higher than in the previous case (i.e., from **Normal** to **Sleep**): this also allows me to make sure that the robot can always go to sleep before completely depleting its battery.  

Lastly, in the **Find** state, the robotic dog looks for the goal ball determined in the **Play** state. In order to do so, this state relies on the explore_lite algorithm; a service client sends a flag to the server, which corresponds to the signal the server itself is waiting it order to run the above mentioned algorithm. Indeed, the core of the algorithm, which is contained inside the Explore class (see [explore.cpp](src/explore.cpp)), has been adjusted with the inclusion of a method able to handle requests incoming from the `/dog_fsm_node` node (see [dog_fsm.py](scripts/dog_fsm.py)). The exploration algorithm keeps running until [dog_vision.py](scripts/dog_vision.py) stops its execution, i.e., a new ball has been found. If the new ball is indeed the goal one, the robot eventually gets back to the **Play** state. The robot will then head over to the human position, and another **Play** state cycle can begin.  
  
The usage of [rqt_graph](http://wiki.ros.org/rqt_graph) shows all the nodes, topics and namespaces involved in the package execution:

<div align="center">
  <img src="rqt_graph_nodes+topics.png" width="900">
</div>

There is only a single topic directly generated by this package:

- `/play_topic`: it is used by the `/human_node` node (see [human.py](scripts/human.py) in order to communicate the willingness to play, and then also the room the robot should reach. The finite state machine node (`/dog_fsm_node`) subscribes to this topic, and so it is the one handling human requests.  
  
The custom [ROS messages](http://wiki.ros.org/Messages) implemented are:

- `assignment3.msg/Coordinates`: message composed by two integers **x** and **y**

A standard message type could have also been used, but this new object creation had also been explited as a quick extra exercise/review, in order to get more acquainted with the ROS environment and the ROS message creation procedure.  
  
The custom [ROS services](http://wiki.ros.org/Services) implemented are:

- `assignment3.srv/Explore`: a simple [int64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int64.html) input and output service that is used to send (re-)start and stop request to the exploration algorithm.  
  
Finally, here are the ROS parameters (well, only the ones strictly related to the robotic dog) loaded in the [ROS parameter server](http://wiki.ros.org/Parameter%20Server) at the beginnning of the package execution:

- `/state`: parameter specifying robot current state
- `/map/x_max`: parameter specifying the maximum x-axis value for the map
- `/map/y_max`: parameter specifying the maximum y-axis value for the map
- `/map/x_min`: parameter specifying the minimum x-axis value for the map
- `/map/y_min`: parameter specifying the minimum y-axis value for the map
- `/home/x`: parameter specifying robot home position (x coordinate)
- `/home/y`: parameter specifying robot home position (y coordinate)
- `/sim_scale`: parameter used to scale simulation velocity
- `/new_ball_detected`: parameter used to specify whether a new ball has been detected or not
- `/unknown_ball`: parameter used to identify which ball has to be looked for
- `/room_list`: list of the available rooms
- `/play_task_ready`: parameter used to specify whether the robot, during the **Play** state, is ready to take orders or not
- `/play_task_done`: parameter used to specify whether the robot, during the **Play** state, has completed the request or not
- `/blue/x`: parameter used to specify the x coordinate of the blue ball, once discovered
- `/blue/y`: parameter used to specify the y coordinate of the blue ball, once discovered
- `/red/x`: parameter used to specify the x coordinate of the red ball, once discovered
- `/red/y`: parameter used to specify the y coordinate of the red ball, once discovered
- `/green/x`: parameter used to specify the x coordinate of the green ball, once discovered
- `/green/y`: parameter used to specify the y coordinate of the green ball, once discovered
- `/yellow/x`: parameter used to specify the x coordinate of the yellow ball, once discovered
- `/yellow/y`: parameter used to specify the y coordinate of the yellow ball, once discovered
- `/magenta/x`: parameter used to specify the x coordinate of the magenta ball, once discovered
- `/magenta/y`: parameter used to specify the y coordinate of the magenta ball, once discovered
- `/black/x`: parameter used to specify the x coordinate of the black ball, once discovered
- `/black/y`: parameter used to specify the y coordinate of the black ball, once discovered

All of these, as already pointed out, can be adjusted before runtime. Please notice that, as a general rule I applied, the value 100 for any of the above parameters always represents missing data or an undefined/default value for that specific parameter.  
Finally, here is the list of all the files and folders featured in this package (up to depth level 2, in order to skip the inclusion of additional [Doxygen](https://www.doxygen.nl/index.html) resources):

\image html "tree.png"

---

## Requirements

In order to run this piece of software, it is needed to have a Linux distribution that supports (and has installed):

- [ROS Kinetic](http://wiki.ros.org/kinetic)
- [SMACH](http://wiki.ros.org/smach)
- [OpenCV](https://opencv.org/)
- [gmapping](http://wiki.ros.org/gmapping)
- [move_base](http://wiki.ros.org/move_base)
- A [Python](https://www.python.org/) interpreter (2.7.x versions recommended; already installed on Ubuntu)
- A [C++](https://www.cplusplus.com/) compiler (already installed on Ubuntu)

---

## Instructions

It might be necessary to make all the nodes executable, before actually being able to run them. So, after reaching this assignment's folder location in a terminal, type:

```bash
chmod +x scripts/* src/*
```

In order to run the code, put the *assignment3* directory inside your [catkin workspace](http://wiki.ros.org/catkin/workspaces)'s `/src` folder, then build with:

```bash
catkin_make
```

And finally launch the whole package with:

```bash
roslaunch assignment.launch
```

This will start the simulation on both Gazebo and RViz, and also open a window showing the robotic dog's live camera output.

---

## Assumptions, limitations and possible improvements

- The robot's simulated battery rationale is a bit simplistic: it only takes into account state cycles, disregarding actual travel distance
- The dog is assumed to start in its home position and, more importantly, the position the robot reaches when it gets close to the human (**Play** state) is again its home, as the mannequin (i.e. the human location) is close the charging station. This is somewhat "hardcoded" in the above mentioned state and, if the mannequin is moved to a location which is far from "home", a couple of lines in the code [dog_fsm.py](scripts/dog_fsm.py) must be changed. In that case, it is thus not sufficient to just adapt the parameters in the [sim.launch](launch/sim.launch) file
- It may happen, on rare occasions, that the robot confuses the chair on which the human is sitting as being red or magenta, thus subsequently starting to try and determine the corresponding room. This might occur if the robot moves really close to the chair, for instance while trying to reach a location behind it. This has been fixed by changing the chair texture from wooden to plain white, like it is, by default, for the human mannequin
- The only situation in which the robot does not take into account for the laser scanner readings is when it discovers a ball for the first time; in that situation the [dog_vision.py](scripts/dog_vision.py) script is the one driving the robotic dog. That may cause lateral collisions with walls, especially when discovering the green ball right after the black one, or the yellow one after the green one, due to the peculiar house layout. This issue has been overcome by implementing an "unstick procedure" (still in [dog_vision.py](scripts/dog_vision.py)). That said, the dog still touches the walls in the process, so I understand that this solution in not perfect
- Rarely, the explore_lite algorithm ends up building a "small virtual prison" from which the robot cannot escape: the exploration process is not aborted as the is still a marker in one of the walls, but the robot has not enough free space to reach it without colliding somewhere or going out of bounds. It may also happen that the exploration algorithm runs out of options before finding a specific ball. At the moment, this is probably the most important undesired behavior, but it could be fixed e.g. by checking the odometry with a subscriber in the **Find** state, and maybe also taking into account the number of markers left (see `current_markers_count` in [explore.cpp](src/explore.cpp)). That said, I tried to implement a recovery behavior that should lead the robot out of this unfavorable situation
- Probably due to lack of computational power, I had a really bad time for weeks with the gmapping algorithm due to sudden and huge changes in the computed map, switching a that-far-perfect model to a completely wrong one. I think that I completely fixed this issue by now, but I feel link I cannot say that it is 100% gone. During my last extensive test sessions though, I found nothing but minimal imperfection in the wall contours, which I think is more than acceptable
- The red ball location is usually a bit hard to be found: going through the corridor of the house does not always reveal the room to the eyes of the robot, thus the eploration algorithm may eventually fail. For that reason, I implemented a safety measure that avoids deadlocks due to this issue, but the only way to actually discover the room is by getting a favorable random location during the Normal state, and this might require some time. But, in the end, it should eventually work
- On some rare occasions the colored balls may get spotted through (or over?) walls: when that happens, the code cannot seamlessly proceed further. Like for the previous point, I created a simple procedure to try and overcome this uncommon event
- The robot is slow; I had no time to test it further, but I am pretty sure that it could go probably faster without major drawbacks. Still, I prefer to be conservative and not to risk to create any new issues or annoyances
- As a bottom line, I tried to make this package completely stable, but I also found many exceptions and annoyances that required a lot of time to be investigated and fixed. In the end, I can assure that the code works, if not every time (due to the instrinsic randomness of some events), atleast for a very good portion of the executions

---

## Author

All the files in this repository belong to [Andrea Pitto](https://github.com/andreabradpitto).  
Contact: [s3942710@studenti.unige.it](mailto:s3942710@studenti.unige.it).

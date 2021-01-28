# Experimental robotics laboratory - Assignment 3

## Introduction
This is the repository for the third and last assignment of the Robotics engineering "Experimental robotics laboratory" course, a.y. 2020/2021, held in the University of Genoa. This work is focused on the implemetation of a ROS-based finite state machine (FSM) nested within a more complex architecture. This project has been coded with [Python 2.7](https://www.python.org/download/releases/2.7/) and it has been tested on [ROS Kinetic](http://wiki.ros.org/kinetic).

---

## Setting
Like for the previous assignment, [Assignment 2](https://github.com/andreabradpitto/Experimental-robotics-laboratory/tree/main/assignment2), the main subject of the project is a robotic dog. This time, it is free to wander inside a house, comprising 6 different rooms: the entrance, the closet, the living room, the kitchen, the bathroom, and the bedroom. Each of these room locations is schematically represented by a colored ball lying on the ground, so that the dog can recognize them. In order to avoid ambiguities, each ball has a different color; following the same order mentioned above, the colors are: blue, red, green, yellow, magenta and black. Event though the house is not procedurally generated (i.e. it is always the same model), every time the simulation is started, the robot does not know where these room are, so it has to learn the house planimetry by exploration. The only thing it knows, indeed, is the association of each colored ball to its respective room, so that when the robotic dog sees e.g. the yellow ball, it knows that that room is the kitchen.<br/>
<br/>
Inside the house there is, other than the walls defining each room's border, also a human mannequin model, which represents the dog owner. The robot is in fact listening for human orders, and eager to play with them. The dog, in normal conditions, wanders randomly around the house, looking for undiscovered colored balls; when these are spotted, it reaches them and stores the coordinates of the corresponding room, so that this info can be used later on. When the human asks the robot to play (ideally, by actually telling the robot the word "play"), the dog reaches them, awaits for the human to tell a room name, and then it tries to reach it. Of course, diferent behaviors have been implmented whether the robot already knows the location it has been instructed to reach or not. When the robotic dog reaches the required room, it outputs a vocal acknowledgement, and finally gets back to the owner. When the robot gets tired, it gets back to its charging dock (also referred as "home") and waits for the battery to recharge. This behavior is the real life pet equivalent of sleeping; this parallelism is the one responsible for the state's actual name, as it will be shown later on in this document. The last behavior is the one devoted to the house exploration, for when the robot is looking for a precise still-to-be-discovered ball: this happens we the human wants it to reach a room that is not yet present in its database.<br/>
<br/>
The code implemented will show a real-time simulation of the assignment over both [Gazebo](http://gazebosim.org/) and [RViz](http://wiki.ros.org/rviz). The human model and the balls will be present only on the former, while the latter is the one that will show the robotic dog's perspective, i.e. all the explored map so far, and the obstacles (the walls or the human model) that it discovered. Gazebo will always show the whole house, so that it is possbile to check if the robot knowledge matches the real planimetry of the environment. In addition, the voice messages shared between the human and the robotic dog will be displayed on terminal.

### The house
Here is a picture of the house taken from Gazebo:

\image html "house.png"

It is comprised inside a 10x10 square, centered a the origin of the three axes. The environment can be thus effectively reduced to a bidimensional scenario, spreading over the xy plane (or, equivalently, on the z = 0 one). The robot initial position, which is also assumed to be its home location, is set in the launch file to be near the human mannequin by default: the latter is in *(-6, 8.5)*, while the former is in *(-5, 8)*. Please notice that in the above picture neither the origin axes nor the floor grid is being shown, in order to only present the house model itself; anyway, Gazebo automatically shows both of them at every new run.<br/>
In the top left corner of the screenshot, it is possible to see the human mannequin, near another decorative object, both colored to white in order to hinder the robot's room detection feature. The robotic dog is present too, and its model is described in detail in the next subsection.

### The robotic dog
The robot I implemented is composed by basic geometric shapes, while trying to find a medium between looks and functionality. Indeed, it features two eyes, in order to resemble a real dog, but only one of them holds a sensor, i.e. a camera, which is used to spot the colored balls: the other one has no purpose other than giving some flavour to the model. The other sensor present on the robot is a laser scanner, which is mounted on its main chassis and is used to detect the obstacles in front of it. It is positioned much lower with respect to the camera in order to guarantee the best obstacle detection as possible: the tests I made clearly showed that having the two robot "eyes" as the two sensor spots was much harder to implement successfully.<br/>
Here is a picture the robotic dog (again, taken from Gazibo):

\image html "robot.png"

It is a two-wheeled differential drive robot. Apart from the wheels, all the joints are fixed, so the dog cannot, for intstance, rotate its head to look around. Notice the orange cube on the front, which is the shape representing the above mentioned laser scanner.

### The environments
Apart from the window showing what the robot sees in real time, this package launches two enviroments: Gazebo, which is used to simulate the physics of the robot, and features nice visuals. The other one is Rviz, which is simply a visualization tool, but that is able to show all the topics involved in the package. One of the most important difference between the two, atleast in this package, is that while in Gazebo one can watch the whole house and the robot with no constraints, in RViz I made so that one only sees what the robot knows and has discovered so far:

\image html "RViz.png"

On the left hand side of the image above, there are many the elements useful to track and assess the robot localization, mapping and exploration capabilities. The image in the middle also shows a lot of information:
- the black elements are the obstacles detected by the robot
- the yellow line is the global navigation plan
- the green line is the local navigation plan
- the red lines are the current detections of the robot's laser
- the blue borders (and the green spheres, not shown here) are elements used by the exploration algorithm (see later on)
- the two white squares centered over the robot are the gloabl and local costmaps

---

## Architecture
slash image html "architecture%20and%20topics.png"

The architecture is made of these components:
- *Human*: it is implemented by [human.py](human_8py.html), that is used to simulate the dog owner, which randomly decides to play with the robotic dog. The human checks if the robot is able to play, waits for it to come nearby, then orders it to move to a random room of the house, and finally waits for it come back. The game ends when the robotic dog is tired. All the communications sent to the robot are carried out via action a message publisher (over `play_topic`), and all the orders are handled by [dog_fsm.py](dog__fsm_8py.html)
- *Dog FSM*: it is implemented by [dog_fsm.py](dog__fsm_8py.html), which is used to handle the robotic dog's FSM internal architecture
- *Dog vision*: it is implemented by [dog_vision.py](dog__vision_8py.html), and constitutes a vision module for the robotic dog that uses OpenCV in order to constantly scan the surroundings, looking for specific colored balls. This node is able to take control, when needed, of the robot movements, allowing it to reach a room when a corresponding new ball is discovered. It also stores the positions of the balls discovered, thus learning the displacement of the rooms inside the house as time passes
- *Ball server*: it is implemented by [ball_server.py](ball__server_8py.html); it is a server providing the coordinates of the ball whose color matches the input room. The received room must have already been coded in a specific single digit integer format in order to be correctly parsed
- *A modified version of the [explore_lite](http://wiki.ros.org/explore_lite) package*: it is implemented by [explore.cpp](explore_8cpp.html), [costmap_client.cpp](costmap__client_8cpp.html), and [frontier_search.cpp](frontier__search_8cpp.html), along with their headers: [explore.h](explore_8h.html), [costmap_client.h](costmap__client_8h.html), [frontier_search.h](frontier__search_8h.html), [costmap_tools.h](costmap__tools_8h.html). It is the algorithm allowing the robot to explore the house during the **Find** state. I made some adjustments in order to let [dog_vision.py](dog__vision_8py.html) and [dog_fsm.py](dog__fsm_8py.html) send service calls in order to (re-)start or stop the exploration<br/>

\image html "fsm.png"

The dog starts in the **Sleep** state.<br/>
In the **Sleep** state, the robot goes to sleep once he gets back home. It relies on the [move_base](http://wiki.ros.org/move_base) algorithm in order to move in the environment, and it does so by implementing an action client and asking it to reach the coordinates corresponding to home.<br/>
In the **Normal** state, the robot wanders randomly, by feeding the move_base algorithm with randomly generated positions. If, while moving around, the robot detects a new room/ball, it reaches it and stores the corresponding position. At any time, a play command can be received by the human: if so happens, the robotic dog transitions to the Play state. Every newly detected room, along with its subsequent data collection process, consumes robot battery. If the battery gets depleted, the robot goes to sleep, by transitioning to the **Sleep** state.<br/>
In the **Play** state, using the move_base algorithm, the robotic dog, gets back home (i.e. close to the human), then starts listening for the room request. Once received, it checks via a service implemented in [ball_server.py](ball__server_8py.html) which are the corresponing ball coordinates to reach. If the ball is not yet in the dog's database, it shifts to the Find state. If the ball has been seen before the dog starts moving to the chosen room, still relying on move_base. After reaching that position, it comes back to the user, again via move_base: whenever this process is completed, some battery is consumed. Furthermore, I assumed that a single cycle of this state consumes slightly more battery, on average, than one of the **Normal** state; for this reason, the threshold of remaining battery before going back to the **Normal** state is higher than in the previous case (i.e. from **Normal** to **Sleep**): this also allows me to make sure that the robot can go to sleep before completely depleting its battery.<br/>
Lastly, in the **Find** state, the robotic dog looks for the goal ball, determined in the **Play** state. In order to do so, this state relies on the explore_lite algorithm; a service client sends a flag to the server, which corresponds to the signal the server itself is waiting it order to run the above mentioned algorithm. The code of the algorithm, which is contained inside the Explore class (see [explore.cpp](explore_8cpp.html)), has been adjusted with the inclusion of a method able to handle requests coming from this node. The exploration algorithm keeps running until [dog_vision.py](dog__vision_8py.html) stops its execution, i.e. a new ball has been found. If the new ball is indeed the goal one, the robot eventually gets back to the **Play** state. The human will then immediately call the dog back to its position, and another **Play** state cycle can begin.<br/>
<br/>

The usage of [rqt_graph](http://wiki.ros.org/rqt_graph) shows all the nodes, topics and namespace involved:

<div align="center">
  <img src="rqt_graph_nodes+topics.png" width="900">
</div>

There is only a single topic directly generated by this package:
- `play_topic`: it is used by the human node in order to communicate the willingness to play, and then also the room the robot should reach. The finiste state machine node subscribes to this topic, and so it is the one devoted to handling human requests.<br/>
<br/>

The message types used are:

- `assignment3.msg/Coordinates`: message made of two integers **x** and **y**

An already-defined message could have also been used, but this new type creation had also been created as an extra exercise, in order to get more acquainted with the ROS environment.<br/>
<br/>

The services message types used are:

- `assignment3.srv/Explore`: a simple [int64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int64.html) input and output service that is used to send (re-)start and stop request to the exploration algorithm
- `assignment3.srv/BallService`: used by [dog_fsm.py](dog__fsm_8py.html) in order to ask for the goal ball's coordinates. If the answer (of type Coordintates) is (100, 100), the requested ball is still to be found, i.e. its coordinates are not yet present in the robotic dog's database. The server is [ball_server.py](ball__server_8py.html)<br/>
<br/>
Finally, here are the parameters (strictly related to the robotic dog) loaded in the ROS parameter server:

- `state`: parameter specifying robot current state
- `map/x_max`: parameter specifying the maximum x-axis value for the map
- `map/y_max`: parameter specifying the maximum y-axis value for the map
- `map/x_min`: parameter specifying the minimum x-axis value for the map
- `map/y_min`: parameter specifying the minimum y-axis value for the map
- `home/x`: parameter specifying robot home position (x coordinate)
- `home/y`: parameter specifying robot home position (y coordinate)
- `sim_scale`: parameter used to scale simulation velocity
- `new_ ball_detected`: parameter used to specify whether a new ball has been detected or not
- `unknown_ball`: parameter used to identify which ball has to be looked for
- `room_list`: list of the available rooms
- `play_task_status`: parameter used to specify **Play** state progress<br/>
                      A value of 0 means that the state is not active or at in initialization phase<br/>
                      A 1 stands for it being in progress<br/>
                      A value of 2 means that it has completed one iteration<br/>
                      The last value is also used by the **Find** state, in case the location of the room asked by the human was not present in the robot's database
- `blue/x`: parameter used to specify the x coordinate of the blue ball, once discovered 
- `blue/y`: parameter used to specify the y coordinate of the blue ball, once discovered
- `red/x`: parameter used to specify the x coordinate of the red ball, once discovered
- `red/y`: parameter used to specify the y coordinate of the red ball, once discovered
- `green/x`: parameter used to specify the x coordinate of the green ball, once discovered
- `green/y`: parameter used to specify the y coordinate of the green ball, once discovered
- `yellow/x`: parameter used to specify the x coordinate of the yellow ball, once discovered
- `yellow/y`: parameter used to specify the y coordinate of the yellow ball, once discovered
- `magenta/x`: parameter used to specify the x coordinate of the magenta ball, once discovered
- `magenta/y`: parameter used to specify the y coordinate of the magenta ball, once discovered
- `black/x`: parameter used to specify the x coordinate of the black ball, once discovered
- `black/y`: parameter used to specify the y coordinate of the black ball, once discovered

All of these, as already pointed out, can be adjusted before runtime.<br/>
Finally, here is the list of all the files and folders featured in this package:

slash image html "tree.png"

---

## Requirements
In order to run this piece of software, it is needed to have a Linux distribution that supports (and has them installed):
- [ROS Kinetic](http://wiki.ros.org/kinetic)
- [SMACH](http://wiki.ros.org/smach)
- [OpenCV](https://opencv.org/)
- A [Python](https://www.python.org/) interpreter

---

## Instructions
It might be needed to make all the nodes executable, before actually being able to run them. So, after reaching this assignment's folder via a terminal, type:

```
$ chmod +x scripts/*
$ chmod +x src/*
```

In order to run the code, put the *assignment3* directory in your workspace, build, then open a terminal and type:

```
$ roslaunch assignment.launch
```

This will start the simulation on both Gazebo and RViz.

---

## Assumptions, limitations and possible improvements
- First of all, due to time constraints, I had no time completely test the code, i.e. I did not leave my program running for hours as suggested (also because my PC would have melted)
- During **Play** the robotic dog ignores (and discards) any new ball discovery: I made this choice in order to grants human orders higher priority with respect to mapping or any other task. That said, I could then have made so that the robot would store its position when a new ball is found during **Play**, and then get back to it once the state ends
- The robot's simulated battery is assumed to drain more rapidly while in **Play** than when in **Normal**, single-cycle-wise. This would be actually true in reality most often than not, but of course it would not always be the case. Furthermore, the **Find** state does not drain battery directly, which is a significant simplification. Anyway, in order to make sure that the robot is always able to get back to its charging station (i.e. home), and also to account for the major battery drain assumption, the robotic dog transitions from **Play** to **Normal** after a certain drain threshold is reached, and then always carries out a single **Normal** state cycle (no matter how far it goes, which is of course another simpification) before going to **Sleep** to recharge its batteries
- The dog is assumed to start in its home position and, more importantly, the position the robot reaches when it gets close to the human (**Play** state) is again its home, as the mannequin (i.e. the human location) is close the charging station. This is somewhat "hardcoded" in the above mentioned state and, if the mannequin is moved to a location which is far from "home", a couple of lines in the code [dog_fsm.py](dog__fsm_8py.html) must be changed. In that case, it is thus not sufficient to just adapt the parameters in the sim.launch launch file
- If, for some reason, the robot loses track of a ball while it is reaching it, there is no way the robot can recover from there. However, the chances of this happening are very slim. A more realistic issue could be that, again while trying to reach a new ball, the robot could hit a wall, as in that situation it ignores incoming laser data. This happens when a wall corner covers the trajectory from the robot position to the ball, so that the latter is still partially visible from the dog's point of view. As this scenario mostly results in somewhat tangential crashes, the robot could still reach its target location in some cases

---

## Authors
All the files in this repository belong to [Andrea Pitto](https://github.com/andreabradpitto).<br/>
Contact: [s3942710@studenti.unige.it](mailto:s3942710@studenti.unige.it).
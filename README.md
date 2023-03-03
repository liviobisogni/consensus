# __Consensus__

### _Author_: Livio Bisogni
###### __&copy; 2021 Turtley & Turtles Ltd.__
___
Let’s agree to turtle-ee!

## Prerequisites

* [ROS](http://wiki.ros.org/ROS/Installation) - An open-source, meta-operating system for your robots. Repository tested only under ROS Kinetic, though.

## How to compile
1. Move this folder (`consensus`) in `~/catkin_ws/src` (or wherever thy ROS workspace is).
2. Launch a terminal window and navigate to the aforementioned ROS workspace, e.g.,

	```bash
	cd ~/catkin_ws/
	```
3. Build the package:

	```bash
	catkin_make
	```

## How to execute
Open the terminal and type:

```bash
roslaunch consensus consensus.launch
```

## How to use

1. `NUM_TURTLES` turtles are given (initial positions defined in `consensus.launch`, line 14 and following; the integer `NUM_TURTLES` defined in `consensus.cpp` should be adjusted accordingly).
2. Firstly, each turtle communicates its initial position to every other comrade.
3. Then, each turtle runs the consensus protocol, in order to determine the centroid(s). Notice that these may slightly differ from the _theoretical_ centroid.
4. Finally, they simultaneously move towards the previously located centroid.
5. Meanwhile, various types of information are printed on the terminal. Wait until the consensus node is shut down. Yet feel free to press `ESC` to exit the program anytime.

## Some _turtlish_ screenshots

* Initial positions:

![](img/c1.png)

* Turtles are movin' toward the centroid(s):

![](img/c2.png)

* Centroid pose(s) finally reached:

![](img/c3.png)

# Path Planning using RRT, RRT-Connect, and Goal_Biased_RRT

## Overview
This repository includes implementations for different Rapidly-exploring Random Trees (RRT) path planning algorithms. It contains all necessary code for running 2D visualizations and Gazebo simulations of RRT, RRT-Connect, and Goal-Biased RRT algorithms.

## Setup and Running Instructions

### Initial Setup
Clone this repository into the `src` folder of your `catkin_ws` and compile the workspace:
```bash
$ cd ~/catkin_ws/src
$ git clone <https://github.com/kalavagunta-vamshi/Path-Planning-of-Turtlebot-Using--RRT--Algorithm.git> rrt_conn
$ cd ..
$ catkin_make
```

### Running the Simulations
To run the Gazebo simulation, use the following command:
```bash
$ roslaunch rrt_conn rrt_conn.launch
```

In a separate terminal, you can run each of the path planning variants:

#### RRT
```bash
$ cd ~/catkin_ws/src/rrt_conn/src
$ python3 rrt.py
```
Close the 2D plot to initiate turtlebot movement in Gazebo. After reaching the goal, close Gazebo and relaunch for other variants.

#### RRT-Connect
```bash
$ python3 rrt_connect.py
```
Follow the same steps as RRT for operation.

#### Goal-Biased RRT
```bash
$ python3 rrt_goal_bias.py
```
Close the 2D plot to start the turtlebot movement. Once the goal is reached, restart Gazebo for new simulations.

## Contributors
- **Surya Chapidi**
  - UID: 119398166
  - Directory ID: Chappidi
  - GitHub: [surya-chapidi](https://github.com/Suryachappidi)
- **Vamshi Kalavagunta**
  - UID: 119126332
  - Directory ID: vamshik
  - GitHub: [kalavagunta-vamshi](https://github.com/kalavagunta-vamshi)

## Simulation Video Links
1. [RRT Simulation](https://drive.google.com/file/d/1g1f-oAiN8eIyEEClWg9qWByuYxcCjW5c/view?usp=share_link)
2. [RRT-Connect Simulation](https://drive.google.com/file/d/1qVUWoIhvkBeW_lVx0Ek4DHxlyLmyMnIT/view?usp=share_link)
3. [Goal-Biased RRT Simulation](https://drive.google.com/file/d/1Xxr5VleOx2CrJcDiPg2yuUci4lI_9p3Q/view?usp=share_link)

## References
1. [IEEE Xplore Document - Optimal path planning for autonomous vehicles](https://ieeexplore.ieee.org/document/9338597)
2. [IEEE Xplore Document - Real-time path planning](https://ieeexplore.ieee.org/document/844730)
3. [GitHub - PathPlanning Repository](https://github.com/zhm-real/PathPlanning)
4. [Paper on Path Planning Algorithms](http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf)
```


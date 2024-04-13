# Path Planning using RRT, RRT-Connect and Goal_Biased_RRT

##### Surya Chappidi (UID:119398166)(Directory ID: Chappidi)
##### Vamshi Kalavagunta (UID: 119126332)(Directory ID: vamshik)

## INSTRUCTIONS TO RUN 2D Visualization and Gazebo Simulation

Place the rrt_conn package into src folder of the catkin_ws and catkin_make.

To run the simulation:

```bash
$ cd ~/catkin_ws/
$ roslaunch rrt_conn rrt_conn.launch
```

In another terminal run,

### RRT
```bash
$ cd ~/catkin_ws/src/rrt_conn/src
$ python3 rrt.py
```
##### close the 2d plot to start the turtlebot movement. Once Reached to goal close Gazebo and relaunch it again for other rrt_variants

### RRT-Connect
```bash
$ cd catkin_ws/src/rrt_conn/src
$ python3 rrt_connect.py
```
##### close the 2d plot to start the turtlebot movement. Once Reached to goal close Gazebo to relaunch it again for other rrt_variants

### RRT-Goal-Bias
```bash
$ cd catkin_ws/src/rrt_conn/src
$ python3 rrt_goal_bias.py
```
##### close the 2d plot to start the turtlebot movement. Once Reached to goal close Gazebo to relaunch it again for other rrt_variants

### Simulation Video Link
1. https://drive.google.com/file/d/1g1f-oAiN8eIyEEClWg9qWByuYxcCjW5c/view?usp=share_link (rrt)
2. https://drive.google.com/file/d/1qVUWoIhvkBeW_lVx0Ek4DHxlyLmyMnIT/view?usp=share_link (rrt_connect)
3. https://drive.google.com/file/d/1Xxr5VleOx2CrJcDiPg2yuUci4lI_9p3Q/view?usp=share_link (rrt_goal_bias)

### References

1. https://ieeexplore.ieee.org/document/9338597
2. https://ieeexplore.ieee.org/document/844730
3. https://github.com/zhm-real/PathPlanning
4. http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf
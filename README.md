# RBE3002 Final Project
This is code to make a turtlebot explore a small, room-sized area using the `move_base` and `gmapping` nodes available online.

A video of this code working can be found [here](https://www.youtube.com/watch?v=JGpubjzQJp0).

In order to run this, first start up the code on the robot:

```bash
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation gmapping_demo.launch
```

and then launch our code:

```bash
roslaunch rbe3002_final turtlebot_no_drive.launch
```

and, if you want to run rviz at the same time, run

```bash
rosrun rviz rviz -d rbe3002_final/rviz/rviz.rviz
```

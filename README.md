Turtlepi
---
### Packages related to Turtlepi

Generates random targets from a feasible target space and records commands to a ros bag.

```
mkdir -p turtlepi_ws/src
git clone https://github.com/surfertas/turtlepi.git turtlepi_ws/src
cd turtlepi_ws/src
# Install turtlebot packages for melodic
./install_all.sh
cd ..
catkin build
source devel/setup.bash
```

To run open three separate windows. Don't forget to source devel/setup.bash if
not done so already for each window.

Window 1: Launch Gazebo, Rviz. The world env0.world is defined in `/models/env0`.
```
cd turtlepi_gazebo/launch
roslaunch turtlepi.launch
```

Window 2: Start up recorder. Specify which topics to record by editing
`turtlepi.yaml` in `/config`. Make sure to update the path to bag to store the recordings.

```
cd turtlepi_recorder/launch
roslaunch turtlepi_recorder.launch
```

Window 3: Start up random target generator.
```
cd turtlepi_navigate/launch
roslaunch turtlepi_gentarget.launch
```

This generate targets and bag files will be automatically generated and stored
in the directory specified in `/turtlepi_recorder/config`.

TODO:

1. Currently trying to figure out how to record tf and cmd_vel by syncing with
other topics.

### Related posts
[Turtlepi #7: Automatic Target Generation for the Turtlebot](http://surfertas.github.io/ros/2017/05/23/autotarget.html)

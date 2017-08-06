Turtlepi
---
### Packages related to Turtlepi

To run open three separate windows. Dont forget to source devel/setup.bash if
not done so already.

Window 1: Launch Gazebo, Rviz. The world env0.world is defined in `/models/env0`
```
cd turtlepi_gazebo/launch
roslaunch turtlepi.launch
```

Window 2: Start up recorder. Specify which topics to record by editing
`turtlepi.yaml` in `/config`

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
in the directory specified in `/turtlepi_recorder/config`

TODO:

1. Currently trying to figure out how to record tf and cmd_vel by syncing with
other topics.



## Husky UR10e Simulation - Instructions

Launch Simulation:

```
ros2 launch clearpath_gz simulation.launch.py setup_path:=/dev_ws/src/husky_commander/config/husky_ur
```

Launch Moveit:

```
ros2 launch clearpath_manipulators moveit.launch.py setup_path:=/dev_ws/src/husky_commander/config/husky_ur use_sim_time:=true
```

Launch Rviz:
```
ros2 launch clearpath_viz view_moveit.launch.py namespace:=a200_0000 use_sim_time:=True
```

Don't forget to change cmd_vel to */a200_0000/cmd_vel*

![Simulation](./media/simulation.png)

To change UR model go to: /dev_ws/src/clearpath_common/clearpath_manipulators_description/urdf/arm/universal_robots.urdf.xacro


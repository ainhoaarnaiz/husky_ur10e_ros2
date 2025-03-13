## Husky UR10e Simulation - Instructions

To change UR model go to: /dev_ws/src/clearpath_common/clearpath_manipulators_description/urdf/arm/universal_robots.urdf.xacro

Launch Simulation:

```
ros2 launch clearpath_gz simulation.launch.py setup_path:=/dev_ws/src/husky_commander/config/husky_ur
```

Don't forget to change cmd_vel to */a200_0000/cmd_vel*

Launch Moveit:

```
ros2 launch clearpath_manipulators moveit.launch.py setup_path:=/dev_ws/src/husky_commander/config/husky_ur use_sim_time:=true
```

Launch Rviz:
```
ros2 launch clearpath_viz view_moveit.launch.py namespace:=a200_0000 use_sim_time:=True
```

![Simulation](./media/simulation.png)

Launch hand teleop:
```
ros2 launch husky_commander husky_hand_teleop_sim.launch.py
```
![Example](./media/example.gif)



Launch:
```
ros2 launch ur_commander iaac_ur10e.launch.py sim:=true
```

```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 ur_base_link world
```

```
ros2 launch clearpath_viz  view_model.launch.py setup_path:=/dev_ws/src/husky_commander/config/husky_basic/
```
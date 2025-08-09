# ros2_descriptions

This package contains robot descriptions (URDF) properly organized to work with Gazebo (>= Harmonic) simulation and control using the `ros2_control` framework.

Visualize the robot model with the following command:

```bash
ros2 launch ros2_descriptions view_robot.launch.py robot:=<robot_name>
```
Robot name options: `ur5`, `spot`

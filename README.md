# learn_urdf

A package for trying out simulations in ROS2.

## Featured
- URDF file of a 2-wheel differential drive robot with Gazebo plugin configuration (camera sensor, IMU sensor, differential drive controller).
- Visualization of the robot with RViz2 and Gazebo.
- Simulation of the robot in Gazebo, controlled by `teleop-twist-keyboard` or any other equivalent control system.


### Nodes
(nothing yet)

### Launch files
- `launch_sim.launch.py` -> Simulate the diff-drive robot in Gazebo with `robot_state_publisher`.
- `rsp.launch.py` -> Launches `robot_state_publisher` and `joint_state_publisher_gui` for the diff-drive robot.
- `spawn_gazebo.launch.py` -> currently not implemented properly.

### Gazebo worlds
(need to figure out how to launch gazebo worlds from within the package.)

### URDF 
- 2-wheel differential drive robot
    * robot.urdf.xacro -> final collection of the robot elements
    * robot_core.xacro -> robot elements
    * inertial_macros.xacro -> inertial objects macros
    * gazebo_control.xacro -> gazebo controller plugin
- rover -> Ingenuity rover URDF (under development)

## How to use
- Have ROS2 Humble installed.
- clone this repo into the `src` directory of a workspace.
- Build with `colcon build --symlink-install`.
- Use the launch files or build your own.

## Things to try out
- Launch a Gazebo world from within the package.
- Fix the rover URDF
- Setup controllers for the six-wheel drive rover.
- Simulate in `mars_yard`.
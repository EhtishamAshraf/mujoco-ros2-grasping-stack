# MuJoCo ROS2 Grasping Stack
This repo implements a robotic manipulation system for pick-and-place tasks using the UR5e robotic arm and the RH8D dexterous hand. The system is built on top of ROS 2 for control, MoveIt for motion planning, and MuJoCo for physics 

**Key Features:**
1. RH8D Hand is controlled using ROS2 Control as discussed [here](https://github.com/EhtishamAshraf/rh8d_mujoco_ros2_control.git)
2. MoveIT2 is used for Cartesian and Joint-space motion planning for the UR5e arm.
3. The robot performs both prehensile and non-prehensile manipulation. 
   - The prehensile part involves grasping and picking an object. 
   - While the non-prehensile part involves pushing the object into its holder without grasping it.

Demo video:
[![Demo Video](https://github.com/EhtishamAshraf/rh8d_mujoco_moveit/blob/807a051345c3ef23b82524161a6e87bc0f99fd67/assets/1-Images/19.png)](https://www.youtube.com/watch?v=nCMynrwSZQo)

---

## Repository Branches

This repository is organized into separate branches for the two RH8D hand configurations and UR5e. One branch contains the **left-hand attached to the ur5e arm**, while the other branch contains the **right-hand attached to the ur5e arm**. 

Demo video:
[![Demo Video](https://github.com/EhtishamAshraf/rh8d_mujoco_moveit/blob/807a051345c3ef23b82524161a6e87bc0f99fd67/assets/1-Images/25.png)](https://www.youtube.com/watch?v=QCvs2jvzyD4)

---

## Project-Specific Modifications to the `mujoco_ros2_control` Plugin

The original [`mujoco_ros2_control plugin`](https://github.com/moveit/mujoco_ros2_control) was modified for the RH8D tendon-driven hand simulation. The main modifications are:

- Added support for the custom `mujoco_actuator` parameter to map a ROS 2 Control joint name to a MuJoCo actuator name.

- Added support for tendon-driven finger control, where a ROS 2 Control command can be sent to a MuJoCo tendon actuator instead of directly commanding every finger joint.

- Added separate URDF joint state export so real movable finger joints can be exposed as ROS 2 Control state interfaces, including position, velocity, and effort, and published through `joint_state_broadcaster` to `/joint_states`.

- Added support to modify the starting position of the UR5e robotic arm.

- Added support for the palm rangefinder sensor:

- Added support for RH8D fingertip force sensors:

**MuJoCo Actuator Mapping**

The RH8D hand is tendon-driven in MuJoCo. Therefore, some ROS 2 Control joints should not be written directly as MuJoCo joints. Instead, their commands must be sent to MuJoCo position/tendon actuators. A custom parameter called `mujoco_actuator` is added to map a ROS 2 Control joint/interface name to the actual MuJoCo actuator name.

Example:

```xml
<joint name="Index_Proximal--palm_assembly:1">
  <param name="mujoco_actuator">pos_index_tendon</param>
  <command_interface name="position" min="0.0" max="1.5708"/>
  <state_interface name="position"/>
  <state_interface name="effort"/>
</joint>
```

This means the ROS 2 Control command: "Index_Proximal--palm_assembly:1/position" is internally written to MuJoCo actuator: "pos_index_tendon"


**First Finger Joint Added to ros2_control**

For each tendon-driven finger, only the first/proximal joint is added as the ROS 2 Control command handle. For example, the index finger has these physical joints:

```text
Index_Proximal--palm_assembly:1
Index_Middle--Index_Proximal
Index_Distal--Index_Middle
```

But in MuJoCo, the full index finger is driven by one tendon actuator:

```text
pos_index_tendon
```

Therefore, only this entry is needed in ros2_control:

```xml
<joint name="Index_Proximal--palm_assembly:1">
  <param name="mujoco_actuator">pos_index_tendon</param>
  <command_interface name="position" min="0.0" max="1.5708"/>
  <state_interface name="position"/>
  <state_interface name="effort"/>
</joint>
```

The middle and distal joints are not added as separate command interfaces because they are not independently actuated.

**URDF Joint State Export**

The plugin also exports all real movable URDF joints as state interfaces. This is important because the controller may command only a tendon actuator, but MuJoCo moves several real joints.

Example:

```text
Commanded actuator:
pos_index_tendon

Actual moving joints:
Index_Proximal--palm_assembly:1
Index_Middle--Index_Proximal
Index_Distal--Index_Middle
```

This allows:

- `joint_state_broadcaster`
- `robot_state_publisher`
- RViz

to visualize the real finger motion.

**Fingertip Force Sensor Support**

The plugin supports RH8D fingertip force sensors. Each fingertip force sensor is exposed as a 3D vector:

```text
index_tip_force/x
index_tip_force/y
index_tip_force/z
```

**Palm Range Sensor Support**

The plugin supports the palm rangefinder sensor. It is exposed as:

```text
palm_range/value
```

---

## System Dependencies and Setup

**Tested on:**

- ROS 2 Humble
- MuJoCo
- ros2_control
- MoveIT2
- Modified mujoco_ros2_control plugin

**The workspace currently depends on:**

- MuJoCo 3.8.0
- GLFW3
- GLEW
- OpenGL / Mesa
- NLOpt
- NLOpt C++ headers: `nlopt.hpp`

Before building the workspace, install the required system dependencies and configure MuJoCo.

```bash
sudo apt update
sudo apt install -y \
  libglfw3-dev \
  libglew-dev \
  libgl1-mesa-dev \
  libnlopt-dev \
  libnlopt-cxx-dev
```

These packages provide the required OpenGL, GLFW, GLEW, and NLOpt dependencies used by the MuJoCo + ROS 2 Control workspace.

**Environment Variables:**

Add the following lines to `~/.bashrc`:

```bash
# MuJoCo
export MUJOCO_HOME=$HOME/.mujoco/mujoco-3.8.0
export MUJOCO_DIR=$MUJOCO_HOME
export MJ_PATH=$MUJOCO_HOME
export LD_LIBRARY_PATH=$MUJOCO_HOME/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=$MUJOCO_HOME:$CMAKE_PREFIX_PATH
export PATH=$MUJOCO_HOME/bin:$PATH
```

Reload the terminal configuration:

```bash
source ~/.bashrc
```

---

### Known Issue: RViz / MoveIt locale-related parameter parsing

**Issue**  
On some systems, RViz and MoveIt may fail to load properly because of locale-related numeric parsing issues, especially when the system uses a decimal comma instead of a decimal point. This can lead to parameter type errors and broken MotionPlanning behavior. [RViz issue reference](https://github.com/moveit/moveit2/issues/1049)

```text
[rviz2-4] [ERROR] [1778660046.411794692] [moveit_background_processing.background_processing]: Exception caught while processing action 'loadRobotModel': parameter 'robot_description_planning.joint_limits.shoulder_pan_joint.max_velocity' has invalid type: Wrong parameter type, parameter {robot_description_planning.joint_limits.shoulder_pan_joint.max_velocity} is of type {double}, setting it to {string} is not allowed.
```

**Solution**  
Set the numeric locale to use `.` as the decimal separator before launching:

Add the following command in the ~/.bashrc and source the ~/.bashrc
export LC_NUMERIC=en_US.UTF-8

---

## Build and Launch Instructions

Go to the workspace. Build the packages:
```bash
colcon build
```

Source the workspace:
```bash
source install/setup.bash
```

Launch the simulation, remember to comment out the right node in the launch file:
```bash
ros2 launch ur5e_moveit_config ur5e_rh8d_mujoco_bringup.launch.py
```

Check active controllers:
```bash
ros2 control list_controllers
```

Start the RH8D hand server:
```bash
ros2 run rh8d_mujoco_control rh8dL_service_controller
```

Demo video:
[![Demo Video](https://github.com/EhtishamAshraf/rh8d_mujoco_moveit/blob/807a051345c3ef23b82524161a6e87bc0f99fd67/assets/1-Images/17.png)](https://www.youtube.com/watch?v=WqfEJmus-HA)

---

## References

1. UR5e arm: MuJoCo XML information can be found [here](https://github.com/google-deepmind/mujoco_menagerie).
2. UR5e arm: ROS2 URDF information can be found [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble).
3. MoveIT Tutorial can be found [here](https://industrial-training-master.readthedocs.io/en/humble/_source/session3/3-Build-a-MoveIt-Package.html).
4. Move group interface Tutorial can be found [here](https://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html).
5. [TracIK](https://github.com/ravnicas/trac_ik) is used as an inverse kinematics solver, providing a faster and more reliable alternative to the standard KDL solver.

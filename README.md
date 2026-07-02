# MECHelangelo ROS 2 Workspace

MECHelangelo is an autonomous gallery robot built around ROS 2 Humble. The active code in this repository lets the robot:

1. drive a differential-drive base with a ThunderBorg motor controller,
2. read LiDAR, IMU, ultrasonic, and camera-derived human-tracking data,
3. explore a room with a simple DVD-bounce style behaviour,
4. approach a detected visitor to an interaction distance,
5. open a timed mimicry session where named human arm poses are forwarded to the robot arms, and
6. run the same high-level behaviour in Gazebo for development without the physical robot.

Large handover assets such as CAD, videos, datasheets, the bill of materials, and extra written reports are not part of this repository anymore. They should be handed over separately. This README and the Doxygen main page describe only what is currently in the MECHelangelo workspace.

## Start Here

Recommended reading order for a new student group:

1. `README.md` - this orientation and package map.
2. `docs/mainpage.dox` or the generated Doxygen HTML - complete architecture and code guide.
3. `Run instructions/MECHelangelo_Pi4_Pi5_Setup_Guide.md` - hardware and software setup notes.
4. `Run instructions/Physical_Robot_Run_Instructions.md` - day-to-day physical robot run procedure.
5. `Run instructions/Arm Node Instructions.pdf` - servo arm setup and testing notes.
6. `src/mechelangelo_behaviour/src/behaviour.hpp` - the cleanest entry point for the main C++ state machine.

## Workspace Layout

| Path | Purpose |
|---|---|
| `src/mechelangelo_behaviour/` | C++ behaviour state machines for simulation, physical autonomous mode, and stationary demo mode. |
| `src/mechelangelo_perception/` | Python ROS nodes for Pi 4 HTTP bridging, simulated camera tracking/mimicry, named arm pose bridging, and ultrasonic sensors. |
| `src/mechelangelo_base_driver/` | Python ThunderBorg driver that converts `/cmd_vel` into left/right motor power. |
| `src/mechelangelo_lidar_driver/` | Python YDLIDAR X4 serial driver publishing `/scan`. |
| `src/mechelangelo_imu_driver/` | Python Sense HAT IMU driver publishing `/imu` and environmental sensor topics. |
| `src/mechelangelo_teleop/` | Keyboard teleoperation utilities. |
| `src/mechelangelo_description/` | URDF robot descriptions and meshes used by `robot_state_publisher` and Gazebo. |
| `src/mechelangelo_gazebo/` | Gazebo Classic worlds, models, arm controller config, and simulation launch files. |
| `src/mechelangelo_bringup/` | Physical robot launch files. `physical_autonomous.launch.py` is the current hardware bringup entry point. |
| `src/Pi4_code/` | Raspberry Pi 4 camera and servo reference code kept with the workspace. The physical run guide still uses the original Pi 4 handover scripts `pose_debug_v8.py` and `servo_test.py`. |
| `Run instructions/` | Setup, running, and arm-node handover instructions. |
| `docs/` | Doxygen configuration and main page source. Generated HTML is written to `docs/doxygen/html/`. |
| `PiBorg_examples/` | Vendor/example ThunderBorg scripts kept for reference. |
| `scratch/` | Old experiments and previous revisions. It is excluded from normal builds by `COLCON_IGNORE` and should be treated as reference material only. |

## Architecture At A Glance

Physical robot:

```text
Pi 4 camera/servo script
  -> HTTP /state
  -> mechelangelo_perception/pi4_bridge.py
  -> /human_tracking, /human_detected, /arm/mimicry_*_pose
  -> mechelangelo_behaviour_physical
  -> /cmd_vel, /arm/right_pose, /arm/left_pose, /interaction_active
  -> ThunderBorg base driver and Pi 4 arm control
```

Simulation:

```text
Gazebo world + robot sensors
  -> /scan and robot camera image
  -> sim_pi4_state_camera + sim_pi4_state_bridge
  -> mechelangelo_behaviour
  -> /cmd_vel and named arm poses
  -> Gazebo diff-drive and arm trajectory controllers
```

The main behaviour node is deliberately central. It subscribes to sensor and human-tracking topics, owns the exploration/interaction state machine, and publishes the final base and arm commands.

## Build

From the outer ROS 2 workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you only want to rebuild this project after editing it:

```bash
colcon build --symlink-install --packages-select \
  mechelangelo_behaviour \
  mechelangelo_perception \
  mechelangelo_base_driver \
  mechelangelo_lidar_driver \
  mechelangelo_imu_driver \
  mechelangelo_teleop \
  mechelangelo_description \
  mechelangelo_gazebo \
  mechelangelo_bringup
```

## Generate The Doxygen Handover Docs

Install Doxygen and Graphviz if needed:

```bash
sudo apt install doxygen graphviz
```

Generate the documentation from this repository root:

```bash
cd ~/ros2_ws/src/MECHelangelo
doxygen docs/Doxyfile
```

Open the generated handover page:

```bash
xdg-open docs/doxygen/html/index.html
```

The Doxygen main page starts with the robot behaviour and architecture, then walks through each package and important file.

## Run Simulation

After building and sourcing the workspace:

```bash
ros2 launch mechelangelo_gazebo mechelangelo_full_mimicry_sim.launch.py
```

This launches Gazebo, the simulated tracking/mimicry camera pipeline, the behaviour node, and the named arm pose bridge. The default launch uses `empty_gallery.launch.py`; pass `gazebo_launch_file:=mechelangelo.launch.py` if you want the fuller gallery world.

## Run The Physical Robot

Use `Run instructions/Physical_Robot_Run_Instructions.md` for the full safety and startup procedure. The short version is:

```bash
# On the Pi 4, from the original camera/servo handover folder:
python3 pose_debug_v8.py

# For servo identification or joint testing only:
python3 servo_test.py

# On the Pi 5, after sourcing the ROS 2 workspace:
ros2 launch mechelangelo_bringup physical_autonomous.launch.py
```

Run the first hardware test with the wheels off the ground and the emergency stop within reach.

## Notes For Future Maintainers

- `physical_autonomous.launch.py` is the current physical bringup file in this workspace.
- `robot.launch.py` appears to be an older generic bringup file and currently references `mechelangelo.urdf.xacro`, which is not present.
- Keep the Pi 4 hardware run procedure aligned with `Run instructions/Physical_Robot_Run_Instructions.md`: use `pose_debug_v8.py` for camera/arm runtime and `servo_test.py` for servo testing.
- `scratch/` contains useful historical context, but it is not the active system.
- Doxygen input is limited to `src/`, `docs/`, and `README.md`, so removed external handover material is not required to build the docs.

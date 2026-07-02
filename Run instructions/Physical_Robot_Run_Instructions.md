# Physical Robot Run Instructions

These instructions are for running the physical MECHelangelo robot using the Pi 5 main ROS computer and the Pi 4 camera/arm computer.

---

## 1. First check battery health and wiring

Before powering the robot:

1. Check the battery health and make sure it is charged.
2. Check the wiring of all major components.
3. Confirm there are no loose wires, exposed conductors, or electronics touching metal parts of the robot.
4. Confirm the switch is off before connecting the battery.

The robot power switch is marked with red tape. Power is on when the switch is flicked towards the red tape.

---

## 2. Installing the battery

With the switch off, place the battery on the bottom plywood plate of the robot.

The best way to do this is to move the battery in from the rear of the robot, as this is the clearest path through the wires.

Be careful of:

- power cables
- LiDAR wiring
- motor driver wiring
- buck converters
- any loose jumper wires

Sit the battery so that its edge is close to the edges of the black 3D printed mounts for the castor wheels.

Remove the screws from the positive and negative battery terminals.

Using the power cables fed through to the bottom of the robot:

1. Connect the black ground wire to the negative battery terminal using the ring terminal.
2. Reuse the original battery screw to clamp the ring terminal.
3. Hand tighten until secure.
4. Connect the red power wire to the positive battery terminal using the same method.

Before turning on the robot, check the wiring again for safety issues such as:

- exposed wiring that could touch other wires or metal
- motor drivers touching metal components
- buck converters touching metal components
- incorrectly wired components
- loose power or ground connections
- LiDAR or Pi wiring under tension

Then flick the switch at the bottom of the robot towards the side marked with red tape and watch the robot power on.

Both Pis will need a small amount of time to connect to the hotspot.

If this is the first time powering on the system, connect both Pis to the correct hotspot/network before continuing.

---

## 3. Pi access details

### Pi 5

The Pi 5 is the main ROS 2 and Docker computer.

```text
Hostname/device name: pi
Username: pi
Password: pi
Terminal prompt: pi@pi:~ $
```

The Pi 5 may still be linked to Andrew's personal Raspberry Pi Connect account. Future groups should not rely on that account for handover access.

Preferred access method:

```bash
ssh pi@pi.local
```

Password:

```text
pi
```

If `pi.local` does not work, find the Pi 5 IP address from the hotspot/router device list or from the Pi 5 terminal:

```bash
hostname -I
```

Then SSH into it using:

```bash
ssh pi@<PI5_IP_ADDRESS>
```

Password:

```text
pi
```

Raspberry Pi Connect can also be used, but only after the Pi 5 has been removed from Andrew's personal Connect account and linked to the new group's Raspberry Pi account.

---

### Pi 4

The Pi 4 is the camera, arm, and servo computer.

SSH into the Pi 4 with:

```bash
ssh pi@172.20.10.3
```

Password:

```text
raspberry
```

The main Pi 4 files used were:

```text
servo_test.py
pose_debug_v8.py
```

`servo_test.py` is for identifying servos and testing joint angles.

`pose_debug_v8.py` is the main debug/run file for connecting to the arms and camera, running the camera, and sending servo signals.

---

## 4. Pi 5 code update

Once the robot is connected to the network, access the Pi 5 and update the code.

```bash
cd ~/git/MECHelangelo
git fetch
git pull
```

The Pi 5 runs ROS 2 through Docker. Docker is necessary because the Pi is running Raspberry Pi OS Desktop rather than Ubuntu 22.04 directly.

---

## 5. Enter the ROS Docker container on the Pi 5

Start the Docker container:

```bash
docker start ros2_humble_mechelangelo
docker exec -it ros2_humble_mechelangelo bash
```

If you open a new terminal or a new Pi Connect tab and want to enter the Docker container again, only repeat the `exec` command:

```bash
docker exec -it ros2_humble_mechelangelo bash
```

---

## 6. Source the ROS workspace

In every Docker terminal, source the ROS environment and workspace before running ROS commands:

```bash
source /opt/ros/humble/setup.bash
cd /home/pi/ros2_ws
source install/setup.bash
```

If a command fails because ROS packages cannot be found, the terminal probably has not been sourced.

---

## 7. Build the Pi 5 workspace

Inside Docker:

```bash
cd /home/pi/ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-skip sense-hat sensehat_ros

source /home/pi/ros2_ws/install/setup.bash
```

The build currently skips:

```text
sense-hat
sensehat_ros
```

These are Sense HAT packages from the Sense HAT GitHub page and are not the actual custom IMU driver used by the robot. If they are present in the workspace, they can cause the build to fail, so they are skipped.

If those packages are no longer in the workspace, skipping them is harmless.

There are repeated `source` and `cd` commands in these instructions because they make the process safer to copy directly into the terminal.

---

## 8. Export ROS domain

If using RViz or another ROS terminal from a laptop, make sure the ROS domain is known and consistent.

On the Pi 5 Docker terminal:

```bash
export ROS_DOMAIN_ID=1
```

On the laptop/RViz terminal, also use:

```bash
export ROS_DOMAIN_ID=1
```

This allows the laptop to subscribe to the robot's published topics for debugging.

---

## 9. Run the physical robot launch file on Pi 5

Inside the Pi 5 Docker container:

```bash
ros2 launch mechelangelo_bringup physical_autonomous.launch.py
```

Alternative launch file:

```bash
ros2 launch mechelangelo_bringup demo.launch.py
```

Use the physical autonomous launch for the real robot unless specifically testing the demo launch.

---

# 10. Pi 4 run instructions

The Pi 4 controls the camera/pose debug code and sends servo signals to the arms.

SSH into the Pi 4:

```bash
ssh pi@172.20.10.3
```

Password:

```text
raspberry
```

Go to the home directory and activate the Python virtual environment:

```bash
cd ~
source servo-env/bin/activate
```

Once the virtual environment is active, the terminal should show something like:

```bash
(servo-env) pi@raspberrypi:~ $
```

Check that the main files are present:

```bash
ls
```

You should see:

```text
servo_test.py
pose_debug_v8.py
```

---

## 10.1 Servo test file

Use this file only when identifying and id'ing servos or testing joint angles.

```bash
python3 servo_test.py
```

This file is useful for:

- checking which servo ID belongs to which joint
- testing whether a servo is responding
- testing individual joint angles
- debugging servo serial connection issues
- setting the id of servos

Do not run `servo_test.py` at the same time as `pose_debug_v8.py`, because both may try to use the same servo serial connection.

---

## 10.2 Main Pi 4 camera and arm file

For the full physical robot run, use:

```bash
python3 pose_debug_v8.py
```

This is the main Pi 4 run file. It is responsible for:

- connecting to the arms
- running the camera
- reading the body/pose tracking
- sending servo signals
- supporting the human interaction behaviour

Leave this running while the Pi 5 physical ROS launch file is running.

---

## 10.3 Pi 4 serial/device checks

If the servos do not connect, check which serial devices are available:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/
```

The servo adapters may appear as:

```text
/dev/ttyACM0
/dev/ttyACM1
```

or possibly:

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

If the script crashes with a missing serial port error, check the port name inside `servo_test.py` or `pose_debug_v8.py` and update it to the correct device.

Using `/dev/serial/by-id/` is more reliable than hardcoding `/dev/ttyUSB0` or `/dev/ttyACM0`.

---

## 10.4 Pi 4 camera/server check

If `pose_debug_v8.py` exposes a camera or state server, test it from the Pi 5 with:

```bash
curl http://172.20.10.3:5000/state
```

If this returns a JSON state, the Pi 5 can read the Pi 4 state correctly.

If it does not respond, check that:

- `pose_debug_v8.py` is still running
- the Pi 4 IP address is still `172.20.10.3`
- both Pis are on the same hotspot/network
- the correct port is being used
- no Python error occurred in the Pi 4 terminal

---

# 11. Pure code instructions for physical robot

## Pi 5

### Fresh git pull

```bash
cd ~/git/MECHelangelo
git fetch
git pull
```

### Start ROS Docker

```bash
docker start ros2_humble_mechelangelo
docker exec -it ros2_humble_mechelangelo bash
```

### Source workspace

```bash
source /opt/ros/humble/setup.bash
cd /home/pi/ros2_ws
source install/setup.bash
```

### Colcon build

```bash
cd /home/pi/ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-skip sense-hat sensehat_ros

source /home/pi/ros2_ws/install/setup.bash
```

### Export ROS domain

```bash
export ROS_DOMAIN_ID=1
```

### Run physical code

```bash
ros2 launch mechelangelo_bringup physical_autonomous.launch.py
```

Alternative:

```bash
ros2 launch mechelangelo_bringup demo.launch.py
```

### For a new Pi 5 terminal

```bash
docker exec -it ros2_humble_mechelangelo bash
```

Then source the workspace again:

```bash
source /opt/ros/humble/setup.bash
cd /home/pi/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=1
```

Then run whatever ROS command is needed.

---

## Pi 4

SSH into the Pi 4:

```bash
ssh pi@172.20.10.3
```

Password:

```text
raspberry
```

Activate the Python environment:

```bash
cd ~
source servo-env/bin/activate
```

For servo ID and joint-angle testing:

```bash
python3 servo_test.py
```

For normal robot operation with the camera and arms:

```bash
python3 pose_debug_v8.py
```

---

# 12. Recommended startup order

Use this order when running the full physical robot:

1. Check wiring and battery.
2. Turn the robot on.
3. Wait for both Pis to connect to the hotspot.
4. SSH into the Pi 4.
5. Activate the Pi 4 virtual environment.
6. Run the Pi 4 camera/arm code:

```bash
python3 pose_debug_v8.py
```

7. SSH into the Pi 5 or access it directly.
8. Pull latest code if needed.
9. Start/enter Docker.
10. Source the ROS workspace.
11. Build if needed.
12. Export ROS domain.
13. Run the Pi 5 physical launch:

```bash
ros2 launch mechelangelo_bringup physical_autonomous.launch.py
```

14. Keep both terminals open while the robot is running.

---

# 13. Simulation Instructions

The simulation of the robot in Gazebo has been made into a single launch file.

On your laptop, assuming the Git repo and ROS workspace have been set up correctly, run the following commands to ensure a clean startup:

```bash
cd ~/ros2_ws/
colcon build

source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

killall gzserver gzclient

ros2 launch mechelangelo_gazebo mechelangelo_full_mimicry_sim.launch.py
```

This opens the empty room environment with the robot spawned inside, along with a human.

The robot begins its behaviour code and explores the room. When it faces the human, it moves towards the human as intended. When it is approximately 1.5 m away, the robot stops and activates the laptop camera. The arms in Gazebo move to the poses made by the user for the interaction duration set in the code.

Stand far enough back from the laptop camera so it can track your skeleton correctly.

When the robot finishes the interaction period, it returns to autonomous mode. It then ignores human detection for the set period, currently about 10 seconds, before searching again in autonomous mode.

---

# 14. Launch files and packages for testing

## Empty gallery with robot

```bash
ros2 launch mechelangelo_gazebo empty_gallery.launch.py
```

## Behaviour code

```bash
ros2 run mechelangelo_behaviour mechelangelo_behaviour
```

## Human detection publisher

```bash
ros2 topic pub --once /human_detected std_msgs/msg/Bool "{data: true}"
```

## Teleop

```bash
ros2 run mechelangelo_teleop teleop_keyboard
```

---

# 15. Launch file notes

The launch files:

```text
environment.launch.py
environment_arm.launch.py
mechelangelo_launch.py
```

include a gallery world with obstacles.

The empty room is closer to the physical environment that the robot is likely to operate in. The client showed no strong interest in having objects in the room with the robot, but the obstacle environments may still be useful for testing.


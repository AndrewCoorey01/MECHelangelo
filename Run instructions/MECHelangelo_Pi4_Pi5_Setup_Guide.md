# MECHelangelo Pi 4 / Pi 5 Setup and Run Guide

This document is a handover guide for future groups working on the MECHelangelo robot. It explains how the Raspberry Pi 5 and Raspberry Pi 4 are used, how to access them, how to set up the GitHub repository, how to install Docker and ROS 2 dependencies, and how to run the physical robot.

---

## 1. System Overview

The robot uses two Raspberry Pis.

| Device | Access method | Main job |
|---|---|---|
| **Pi 5** | SSH recommended. Raspberry Pi Connect optional after relinking. | Main ROS 2 and Docker robot computer. |
| **Pi 4** | SSH. | Camera and perception computer. |

The system works like this:

```text
Pi 4 camera detects human / gestures
        ↓
Pi 4 exposes state on port 5000
        ↓
Pi 5 reads that state through the bridge
        ↓
Pi 5 ROS 2 behaviour node reacts
        ↓
Pi 5 sends /cmd_vel to the robot base
```

The Pi 5 handles the main robot stack:

- Docker
- ROS 2 Humble container
- LiDAR driver
- IMU driver
- ThunderBorg base driver
- dual ultrasonic sensor node
- autonomous behaviour node
- Pi 4 camera bridge

The Pi 4 handles:

- camera/perception code
- human/gesture state detection
- `/state` web endpoint on port `5000`

---

## 2. Important Access Note

The Pi 5 was previously connected to **Andrew's personal Raspberry Pi Connect account** during development. Future groups should **not rely on Andrew's account** for access.

Future groups should either:

1. **SSH into the Pi 5**. This is the recommended handover method.
2. **Remove/relink Raspberry Pi Connect** so the Pi 5 is connected to the future group's own Raspberry Pi account.

---

## 3. Login Details

### Pi 5 — Main Robot Computer

The Pi 5 was intentionally set up with simple credentials to reduce confusion during handover.

```text
Hostname / device name: pi
Username: pi
Password: pi
Terminal prompt: pi@pi:~ $
```

The Pi 5 shown in Raspberry Pi Connect was:

```text
Name: pi
Model: Raspberry Pi 5 rev 1.1, 16 GB
Operating system: Raspberry Pi OS 13 Trixie
Architecture: aarch64
```

### Pi 4 — Camera Computer

The Pi 4 is accessed through SSH.

```bash
ssh pi@172.20.10.3
```

Password:

```text
raspberry
```

The Pi 4 camera state URL is:

```text
http://172.20.10.3:5000/state
```

From the Pi 5, test the Pi 4 connection with:

```bash
curl http://172.20.10.3:5000/state
```

Expected response format:

```json
{
  "right_confirmed": false,
  "left_confirmed": false,
  "turn_cmd": "STOP",
  "locked": false
}
```

The values will change depending on what the camera sees.

---

## 4. Recommended Pi 5 Access Method — SSH

SSH is the best handover method because it does not depend on Andrew's personal Raspberry Pi Connect account.

### 4.1 Enable SSH on the Pi 5

If using the Pi 5 desktop directly:

```text
Raspberry Pi menu → Preferences → Control Centre → Interfaces → SSH → Enable
```

Or from the terminal:

```bash
sudo raspi-config
```

Then go to:

```text
3 Interface Options → I1 SSH → Yes
```

Reboot after enabling SSH:

```bash
sudo reboot
```

### 4.2 Find the Pi 5 IP Address

On the Pi 5 terminal:

```bash
hostname -I
```

This will print one or more IP addresses.

Example:

```text
172.20.10.8
```

### 4.3 SSH Into the Pi 5

From a laptop on the same Wi-Fi or hotspot:

```bash
ssh pi@pi.local
```

Password:

```text
pi
```

If `pi.local` does not work, use the IP address instead:

```bash
ssh pi@<PI5_IP_ADDRESS>
```

Example:

```bash
ssh pi@172.20.10.8
```

Password:

```text
pi
```

Once connected, the terminal should show:

```bash
pi@pi:~ $
```

---

## 5. Optional Pi 5 Access Method — Transfer Raspberry Pi Connect

Only use Raspberry Pi Connect after linking the Pi 5 to the future group's own Raspberry Pi account.

### 5.1 Remove the Pi 5 From Andrew's Raspberry Pi Connect Account

Andrew should do this before handover if possible:

```text
1. Go to connect.raspberrypi.com.
2. Sign in to Andrew's Raspberry Pi account.
3. Open the Devices tab.
4. Select the device named pi.
5. Open Settings.
6. Delete/remove the device.
```

After this, the future group can link the Pi 5 to their own Raspberry Pi account.

### 5.2 Link the Pi 5 to a New Raspberry Pi Connect Account

On the Pi 5 terminal:

```bash
rpi-connect status
```

If Connect is off:

```bash
rpi-connect on
```

Then start sign-in:

```bash
rpi-connect signin
```

This prints a verification link. Open that link in a browser, sign in with the future group's Raspberry Pi account, and link the device.

After linking, check:

```bash
rpi-connect status
```

If remote shell is disabled:

```bash
rpi-connect shell on
```

If screen sharing is disabled:

```bash
rpi-connect vnc on
```

If Raspberry Pi Connect is not installed:

```bash
sudo apt update
sudo apt install rpi-connect-lite
rpi-connect signin
```

---

## 6. Flashing or Resetting Either Pi

Use Raspberry Pi Imager if either Pi needs to be fully reset.

### Recommended Pi 5 Imaging Settings

```text
Hostname: pi
Username: pi
Password: pi
Enable SSH: yes
Wi-Fi: same network/hotspot as Pi 4
OS: Raspberry Pi OS Desktop, 64-bit
```

### Recommended Pi 4 Imaging Settings

```text
Username: pi
Password: raspberry
Enable SSH: yes
Wi-Fi: same network/hotspot as Pi 5
Preferred IP: 172.20.10.3
```

After first boot, update both Pis:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y git curl wget nano vim htop net-tools python3-pip python3-venv
sudo reboot
```

---

## 7. GitHub Setup

Create the workspace folder:

```bash
mkdir -p ~/ros2_ws/src
mkdir -p ~/git
```

### 7.1 Simple HTTPS Clone

Use this if the group only needs to pull code:

```bash
cd ~/ros2_ws/src
git clone https://github.com/AndrewCoorey01/MECHelangelo.git
```

### 7.2 SSH Clone for Pushing Changes

Use this if the group needs to push back to GitHub.

Generate an SSH key:

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub
```

Copy the printed public key into GitHub:

```text
GitHub → Settings → SSH and GPG keys → New SSH key
```

Then clone:

```bash
cd ~/ros2_ws/src
git clone git@github.com:AndrewCoorey01/MECHelangelo.git
```

---

## 8. Pi 5 Hardware Checks

### 8.1 Check I2C

The Pi 5 uses I2C for the ThunderBorg and Sense HAT IMU.

Enable I2C:

```bash
sudo raspi-config
```

Then:

```text
Interface Options → I2C → Enable
```

Install tools:

```bash
sudo apt install -y i2c-tools python3-smbus
```

Check devices:

```bash
i2cdetect -y 1
```

Expected devices from the working setup:

```text
ThunderBorg: 0x15
Sense HAT / IMU-related devices: 0x1c, 0x5c, 0x5f, 0x6a
```

If `0x15` is missing, check:

- ThunderBorg power
- I2C wiring
- SDA/SCL connections
- common ground
- loose connectors

### 8.2 Check LiDAR

The YDLIDAR X4 should appear as a USB serial device.

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/
```

The LiDAR was usually:

```text
/dev/ttyUSB0
```

If the LiDAR fails to connect, check:

```bash
dmesg | grep -i tty
lsusb
ls -l /dev/ttyUSB*
```

If the port changes, update the LiDAR config file or use a stable `/dev/serial/by-id/` path.

### 8.3 Check Ultrasonic Sensors

The ultrasonic sensors are used to slow and stop the robot during human approach.

Known confirmed wiring from the project:

```text
Sensor 1 TRIG: GPIO17, physical pin 11
Sensor 1 ECHO: GPIO22, physical pin 15
```

The HC-SR04 echo line is 5 V, so it must go through a voltage divider before entering the Pi GPIO.

The working divider used:

```text
1 kΩ + 1.8 kΩ voltage divider
```

Target behaviour:

```text
Slow at about 1.6 m
Stop at about 1.5 m
```

Check the final `dual_ultrasonic` node or launch/config file before rewiring the second ultrasonic sensor, because the exact second sensor pin record should be verified from the code.

### 8.4 Check Power

The robot previously had undervoltage issues, so always check this if sensors disappear or the Pi behaves strangely.

```bash
vcgencmd get_throttled
dmesg | grep -i voltage
```

Good result:

```text
throttled=0x0
```

Bad examples:

```text
0x50000
0x50005
hwmon undervoltage
```

If undervoltage appears, check:

- 5 V buck converter
- battery voltage
- inline fuse
- common ground
- Pi 5 power wiring
- loose connections

---

## 9. Install Docker on the Pi 5

Run this on the **Pi 5 host**, not inside Docker.

```bash
sudo apt update
sudo apt install -y ca-certificates curl

sudo install -m 0755 -d /etc/apt/keyrings

sudo curl -fsSL https://download.docker.com/linux/debian/gpg \
  -o /etc/apt/keyrings/docker.asc

sudo chmod a+r /etc/apt/keyrings/docker.asc

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update

sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Allow the `pi` user to run Docker:

```bash
sudo usermod -aG docker pi
sudo reboot
```

After reboot:

```bash
docker --version
docker run hello-world
```

---

## 10. Create the ROS 2 Docker Container on the Pi 5

The project used this container name:

```text
ros2_humble_mechelangelo
```

Pull the ROS 2 Humble image:

```bash
docker pull osrf/ros:humble-desktop
```

Create the container:

```bash
docker run -it \
  --name ros2_humble_mechelangelo \
  --privileged \
  --network host \
  --ipc host \
  -v /home/pi:/home/pi \
  osrf/ros:humble-desktop \
  bash
```

These flags matter:

```text
--privileged          allows access to USB, I2C, GPIO and serial devices
--network host        allows ROS 2 networking and Pi 4 bridge communication
--ipc host            avoids some ROS 2 shared-memory issues
-v /home/pi:/home/pi  makes the Pi files visible inside Docker
```

After the container has been created, start it later with:

```bash
docker start -ai ros2_humble_mechelangelo
```

Or open another shell into it:

```bash
docker exec -it ros2_humble_mechelangelo bash
```

---

## 11. Install ROS 2 Dependencies Inside Docker

Inside the Docker container:

```bash
apt update

apt install -y \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  build-essential \
  cmake \
  nano \
  vim \
  i2c-tools \
  python3-smbus \
  python3-serial \
  python3-numpy
```

Install useful ROS packages:

```bash
apt install -y \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-tf-transformations \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-std-msgs
```

Set up rosdep:

```bash
rosdep init || true
rosdep update
```

---

## 12. Build the Pi 5 ROS 2 Workspace

Inside Docker:

```bash
source /opt/ros/humble/setup.bash
cd /home/pi/ros2_ws
```

If the repo is not already there:

```bash
cd /home/pi/ros2_ws/src
git clone https://github.com/AndrewCoorey01/MECHelangelo.git
```

Build:

```bash
cd /home/pi/ros2_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-skip sense-hat sensehat_ros mechelangelo_gazebo
```

Source the workspace:

```bash
source /home/pi/ros2_ws/install/setup.bash
```

Recommended Docker `.bashrc` setup:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/pi/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
```

---

## 13. ROS 2 Environment Variables

Use these in every Pi 5 ROS terminal:

```bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
```

The project used:

```text
ROS_DOMAIN_ID=1
```

This must match if using RViz from another computer.

---

## 14. Pi 4 Camera Setup

SSH into the Pi 4:

```bash
ssh pi@172.20.10.3
```

Password:

```text
raspberry
```

Install dependencies:

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-pip \
  python3-venv \
  python3-opencv \
  python3-flask \
  python3-numpy
```

If using the Raspberry Pi camera stack:

```bash
sudo apt install -y python3-picamera2
```

Clone the repo:

```bash
mkdir -p ~/git
cd ~/git
git clone https://github.com/AndrewCoorey01/MECHelangelo.git
```

Run the Pi 4 camera/perception server from the relevant project folder.

The important output is that this URL must work:

```text
http://172.20.10.3:5000/state
```

Test from Pi 5:

```bash
curl http://172.20.10.3:5000/state
```

Do not continue until this works.

---

## 15. Running the Robot

### Step 1 — Start the Pi 4 Camera Server

On the Pi 4:

```bash
ssh pi@172.20.10.3
```

Password:

```text
raspberry
```

Go to the camera/perception code folder:

```bash
cd ~/git/MECHelangelo/src/mechelangelo_perception
```

Run the camera state server script used by the final project.

Then check from the Pi 5:

```bash
curl http://172.20.10.3:5000/state
```

Do not continue until this works.

### Step 2 — Start Pi 5 Docker

On the Pi 5 through SSH:

```bash
ssh pi@pi.local
```

or:

```bash
ssh pi@<PI5_IP_ADDRESS>
```

Password:

```text
pi
```

Start the Docker container:

```bash
docker start -ai ros2_humble_mechelangelo
```

Inside Docker:

```bash
source /opt/ros/humble/setup.bash
source /home/pi/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
```

Launch the robot:

```bash
ros2 launch mechelangelo_bringup physical_autonomous.launch.py
```

---

## 16. Useful ROS Checks

Inside the Pi 5 Docker container:

```bash
ros2 topic list
```

Important topics:

```text
/scan
/scan_filtered
/imu
/cmd_vel
/human_detected
/human_tracking
/ultrasonic
/arm/right_pose
/arm/left_pose
/arm/turn
/arm/locked
```

Check LiDAR:

```bash
ros2 topic echo /scan --once
```

Check IMU:

```bash
ros2 topic hz /imu
```

Check human detection:

```bash
ros2 topic echo /human_detected
ros2 topic echo /human_tracking
```

Check motor command output:

```bash
ros2 topic echo /cmd_vel
```

---

## 17. RViz From Laptop

On the laptop:

```bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
rviz2
```

Useful RViz displays:

```text
LaserScan: /scan
LaserScan: /scan_filtered
TF
RobotModel
```

If RViz cannot see topics:

```bash
ros2 topic list
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

Make sure:

- the laptop and Pi 5 are on the same network
- the Docker container was created with `--network host`
- `ROS_DOMAIN_ID=1` on both machines
- `ROS_LOCALHOST_ONLY=0` on both machines

---

## 18. Rebuilding After Code Changes

Inside Docker on the Pi 5:

```bash
cd /home/pi/ros2_ws/src/MECHelangelo
git pull
```

Then:

```bash
cd /home/pi/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-skip sense-hat sensehat_ros mechelangelo_gazebo

source install/setup.bash
```

Relaunch:

```bash
ros2 launch mechelangelo_bringup physical_autonomous.launch.py
```

---

## 19. Copying KNN Pose Files From Pi 4 to Pi 5

The camera/mimicry setup used:

```text
pose_knn_left.json
pose_knn_right.json
```

Copy from Pi 4 to Pi 5:

```bash
scp pi@172.20.10.3:/home/pi/pose_knn_left.json \
  /home/pi/ros2_ws/src/MECHelangelo/src/mechelangelo_perception/config/

scp pi@172.20.10.3:/home/pi/pose_knn_right.json \
  /home/pi/ros2_ws/src/MECHelangelo/src/mechelangelo_perception/config/
```

Password:

```text
raspberry
```

Then rebuild if needed.

---

## 20. Common Troubleshooting

### Pi 5 Docker Cannot See Hardware

The container must have been created with:

```bash
--privileged
--network host
--ipc host
-v /home/pi:/home/pi
```

If not, remove and recreate it:

```bash
docker stop ros2_humble_mechelangelo
docker rm ros2_humble_mechelangelo
```

Then recreate it using the command in Section 10.

### Pi 5 Cannot Read Pi 4 Camera State

From Pi 5:

```bash
curl http://172.20.10.3:5000/state
```

If this fails, check:

- Pi 4 camera server is running
- Pi 4 IP is still `172.20.10.3`
- Pi 4 and Pi 5 are on the same network
- correct port is being used
- hotspot/router is working

### LiDAR Not Connecting

Check:

```bash
lsusb
ls -l /dev/ttyUSB*
dmesg | grep -i tty
```

The LiDAR was usually:

```text
/dev/ttyUSB0
```

If it changes, update the config file or use `/dev/serial/by-id/`.

### ThunderBorg Not Found

Check:

```bash
i2cdetect -y 1
```

Expected:

```text
0x15
```

If missing, check:

- I2C is enabled
- ThunderBorg power
- SDA/SCL wiring
- common ground
- loose connectors

### IMU Not Publishing

Check:

```bash
i2cdetect -y 1
ros2 topic list | grep imu
ros2 topic hz /imu
```

If devices disappear, check undervoltage:

```bash
vcgencmd get_throttled
dmesg | grep -i voltage
```

### Robot Does Not Move

Check whether commands are being produced:

```bash
ros2 topic echo /cmd_vel
```

If `/cmd_vel` changes but wheels do not move, check:

- ThunderBorg power
- motor wiring
- battery voltage
- fuse
- base driver messages

If `/cmd_vel` is always zero, check:

- behaviour node may be stopped
- LiDAR may think an obstacle is too close
- ultrasonic may be forcing stop
- Pi 4 bridge may be sending locked/human state

---

## 21. Final Demonstration Checklist

```text
1. Battery charged.
2. Pi 5 has stable 5 V power.
3. Pi 4 and Pi 5 are on the same network.
4. Pi 5 accessible through SSH.
5. Pi 4 accessible with: ssh pi@172.20.10.3
6. Pi 4 camera server is running.
7. Pi 5 can curl http://172.20.10.3:5000/state.
8. Docker container starts on Pi 5.
9. ROS_DOMAIN_ID=1.
10. ROS_LOCALHOST_ONLY=0.
11. LiDAR appears as /dev/ttyUSB0 or correct by-id port.
12. i2cdetect shows ThunderBorg at 0x15.
13. IMU topic publishes.
14. Ultrasonic topic publishes.
15. /cmd_vel publishes.
16. First movement test is done with wheels lifted.
17. Robot is placed on ground only after commands look correct.
```

---

## 22. Quick Command Summary

### Pi 4

```bash
ssh pi@172.20.10.3
```

Password:

```text
raspberry
```

Test camera state:

```bash
curl http://172.20.10.3:5000/state
```

### Pi 5

SSH:

```bash
ssh pi@pi.local
```

or:

```bash
ssh pi@<PI5_IP_ADDRESS>
```

Password:

```text
pi
```

Start Docker:

```bash
docker start -ai ros2_humble_mechelangelo
```

Run robot:

```bash
source /opt/ros/humble/setup.bash
source /home/pi/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0

ros2 launch mechelangelo_bringup physical_autonomous.launch.py
```

Rebuild:

```bash
cd /home/pi/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-skip sense-hat sensehat_ros mechelangelo_gazebo

source install/setup.bash
```

Hardware checks:

```bash
i2cdetect -y 1
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/
vcgencmd get_throttled
dmesg | grep -i voltage
```

---

## 23. Security Note

The simple passwords were useful during development and handover, but they should be changed if the robot is connected to a public, shared, or university-wide network.

To change the password on either Pi:

```bash
passwd
```


Snapshot of mechelangelo_behaviour taken before IMU-confirmed rotation was added.

Active version differences:
- ALIGNING state used open-loop time integration (target_angle_ -= turn_cmd * dt)
  which caused overshooting on the physical robot because cmd rad/s does not
  translate linearly to actual motion.
- No IMU subscriber existed in the node.

What changed in the active version:
- Added /imu subscription (sensor_msgs/msg/Imu) to track actual yaw rotation
- ALIGNING now records yaw at entry and computes remaining_angle from real IMU delta
- Falls back to open-loop estimation if IMU has not published yet

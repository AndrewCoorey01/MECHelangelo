mechelangelo_perception — VERSION 1 (original)
================================================
Saved: 2026-05-29

This directory is a frozen copy of the mechelangelo_perception package
as it existed before the v2 refactor.  It is kept here for reference only
and is NOT built by colcon (no COLCON_IGNORE needed — it lives in scratch/).

WHAT V1 DID
-----------
Single script (pose_tracking_human_ros.py) that ran standalone with
`python3 pose_tracking_human_ros.py --camera <sim|pi|usb>`.

It combined everything in one process:
  - Human detection and lock-on (for approach / following)
  - Skeleton angle extraction (for arm mimicry)
  - MQTT publishing of arm angles AND robot movement commands
  - ROS publisher for /human_tracking
  - Flask MJPEG debug stream on :5000

The package was NOT registered as ros2 run entry points (setup.py had
empty console_scripts).  The script had to be run directly with python3.

DIFFERENCES FROM V2 (active package)
--------------------------------------
  - No --mode argument (always did tracking + mimicry together)
  - No direct /arm_pose ROS publisher (arm angles only went via MQTT)
  - Not usable with `ros2 run mechelangelo_perception pose_tracking_human_ros`
  - No launch/ directory or launch files
  - mqtt_bridge.py was not registered as an entry point either

FILES
-----
  mechelangelo_perception/pose_tracking_human_ros.py  — main script (v1)
  mechelangelo_perception/mqtt_bridge.py              — MQTT→ROS bridge (v1)
  mechelangelo_perception/pose_debug.py               — standalone Pi debug (v1)
  package.xml, setup.py, setup.cfg                    — as-was build config

ACTIVE VERSION
--------------
  src/mechelangelo_perception/

/**
 * @file behaviour.hpp
 * @brief Declaration of MechelangeloBehaviour — the navigation and
 *        human-interaction state machine for the MECHelangelo gallery robot.
 *
 * @details
 * This header defines:
 * - `NavigationState` — the top-level FSM states.
 * - `LaserSegment`    — a Cartesian cluster of neighbouring LiDAR returns.
 * - `MechelangeloBehaviour` — the ROS 2 node class.
 *
 * The node runs at 10 Hz (100 ms wall-clock timer).  All tuning constants
 * (`kForwardSpeed`, `kDvdOpenClearanceDistance`, etc.) are defined as
 * `static constexpr` in `behaviour.cpp` to keep this header clean.
 *
 * ### Operating modes
 *
 * **DVD-bounce exploration** (`blindAutonomous()`): Cycles through
 * SEARCHING → ALIGNING → MOVING → STOPPED.  After stopping at a wall the
 * node picks a random heading from the longest clear arc that falls in the
 * preferred 55°–150° side-bounce band, avoiding the wall it just hit.
 *
 * **Human interaction** (`HUMAN_DETECTED`): Activated when the camera node
 * publishes a `/human_tracking` message with `detected == 1.0`.  A fused
 * camera/LiDAR tracker maintains a persistent target lock and a
 * dynamic-window arc planner navigates toward the visitor.  Once the 1.5 m
 * arm-clearance bubble is free the robot enters a 30-second mimicry session
 * then returns to DVD exploration with a 10-second detection cooldown.
 */

#ifndef BEHAVIOUR_HPP
#define BEHAVIOUR_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <random>
#include <vector>

/**
 * @brief Top-level navigation states for the MECHelangelo behaviour FSM.
 *
 * The normal DVD-bounce exploration cycle visits the states in the order:
 * SEARCHING → ALIGNING → MOVING → STOPPED → SEARCHING …
 *
 * HUMAN_DETECTED interrupts this cycle whenever the camera node reports a
 * locked visitor.  The node returns to SEARCHING once the interaction session
 * completes (or the visitor is lost beyond the recovery timeout).
 */
enum class NavigationState
{
    /** @brief Robot is stationary, computing the next safe exploration heading
     *         from the current LiDAR scan using DVD-bounce logic. */
    SEARCHING,

    /** @brief Robot is rotating in-place to face the chosen heading.
     *         Rotation is IMU-confirmed when `/imu` data is available;
     *         falls back to open-loop time integration otherwise. */
    ALIGNING,

    /** @brief Robot is driving forward toward the chosen heading.
     *         Continuously checks the front arc for blocking obstacles. */
    MOVING,

    /** @brief Robot has stopped at an obstacle or wall.
     *         Holds position for ~3 seconds (30 control loops) then
     *         transitions back to SEARCHING. */
    STOPPED,

    /** @brief Camera has locked onto a human visitor.
     *         A fused camera/LiDAR tracker and arc planner control the robot
     *         until the interaction session ends or the visitor is lost. */
    HUMAN_DETECTED
};

/**
 * @brief A contiguous cluster of neighbouring LiDAR returns extracted from
 *        a `sensor_msgs/LaserScan`.
 *
 * Segments are built by `buildLaserSegments()` which joins consecutive scan
 * points whose Cartesian gap is smaller than `kSegmentJoinDistance` and
 * rejects clusters with fewer than `kSegmentMinPoints` points or a Cartesian
 * length shorter than `kSegmentMinLength`.  This mirrors the earlier
 * `LaserProcessing::countSegments()` approach and is used both for obstacle
 * detection and for human cluster association.
 */
struct LaserSegment
{
    /** @brief Index of the first beam in the raw scan array. */
    int start_index = 0;

    /** @brief Index of the last beam in the raw scan array (inclusive). */
    int end_index = 0;

    /** @brief Total number of valid points in this segment. */
    int point_count = 0;

    /** @brief Cartesian position of the first point (robot frame, metres). */
    geometry_msgs::msg::Point start_point;

    /** @brief Cartesian position of the last point (robot frame, metres). */
    geometry_msgs::msg::Point end_point;

    /** @brief Cartesian midpoint of the segment (robot frame, metres). */
    geometry_msgs::msg::Point midpoint;

    /** @brief Minimum range (metres) across all points in this segment. */
    double min_range = 0.0;

    /** @brief Bearing to the midpoint (radians, robot frame).
     *         Positive angles are counter-clockwise from the robot forward axis. */
    double midpoint_angle = 0.0;

    /** @brief Straight-line Cartesian length of the segment (metres). */
    double length = 0.0;
};

/**
 * @brief ROS 2 node implementing the MECHelangelo navigation and
 *        human-interaction behaviour.
 *
 * @details
 * Inherits from `rclcpp::Node` and is spun by `main()` in
 * `behaviour_physical.cpp`.  The class is constructed once, then `run()` is
 * called with a simulation flag that selects the appropriate parameters via
 * the `stop_distance_m` ROS parameter (default 1.5 m for simulation,
 * 0.75 m for the physical robot).
 *
 * Public interface is deliberately minimal — the node self-manages via
 * its timer and subscriber callbacks.
 *
 * @note All heavy implementation logic, auxiliary enums
 * (`HumanMotionPhase`, `HumanTargetConfidence`, `ProactiveAvoidanceStage`,
 * `DeterministicEscapeStage`), and file-scope state variables lives in
 * `behaviour.cpp` so that this header remains a clean interface boundary.
 */
class MechelangeloBehaviour : public rclcpp::Node
{
public:
    /**
     * @brief Construct the node, declare ROS parameters, create all
     *        publishers/subscribers, and start the 100 ms control timer.
     *
     * @details
     * On construction the node:
     * - Declares `stop_distance_m` (default 1.5) and `approach_arm_pose_name`
     *   (default `"arm_down"`) parameters.
     * - Creates publishers for `/cmd_vel`, `/arm/right_pose`,
     *   `/arm/left_pose`, `/interaction_active`, `/voice_prompt`,
     *   `/scan_filtered`, and `/behaviour_obstacle_markers`.
     * - Subscribes to `/scan`, `/imu`, `/human_detected`,
     *   `/human_tracking`, `/arm/mimicry_right_pose`, and
     *   `/arm/mimicry_left_pose`.
     * - Publishes an initial `interaction_active = false` and `arm_down` to
     *   both arms.
     */
    MechelangeloBehaviour();

    /**
     * @brief Stop the node cleanly and log shutdown.
     */
    ~MechelangeloBehaviour();

    /**
     * @brief Activate the behaviour and hand execution to the ROS executor.
     *
     * @details
     * Calls `blindAutonomous()` to initialise the DVD-bounce exploration state,
     * then blocks in `rclcpp::spin()` until the node is shut down.
     *
     * @param sim_mode  `true` when running under Gazebo.  Currently only
     *                  controls the log message; the actual stop distance is
     *                  set by the `stop_distance_m` ROS parameter at launch.
     */
    void run(bool sim_mode);

private:
    // ------------------------------------------------------------------
    // Behaviour modes
    // ------------------------------------------------------------------

    /**
     * @brief Initialise the DVD-bounce exploration mode.
     *
     * @details
     * Resets the navigation state to SEARCHING, clears all interaction state,
     * zeros the safety-zone baseline, and clears obstacle markers.  Called
     * once from `run()` on startup and again each time a completed interaction
     * session returns the robot to exploration.
     */
    void blindAutonomous();

    /**
     * @brief Placeholder for a future map-based autonomous mode.
     *
     * @details Not yet implemented; logs an info message when entered.
     */
    void mappedAutonomous();

    // ------------------------------------------------------------------
    // Main control loop
    // ------------------------------------------------------------------

    /**
     * @brief 10 Hz control-loop callback driven by `control_timer_`.
     *
     * @details
     * Dispatches to the appropriate NavigationState handler and publishes the
     * resulting `geometry_msgs/Twist` on `/cmd_vel`.  Also manages the
     * interaction-active gate and arm-down keepalive.
     *
     * Guards:
     * - Returns immediately if `blind_autonomous_active_` is false.
     * - Emits a throttled warning and stops if no valid scan has arrived yet.
     */
    void controlLoop();

    // ------------------------------------------------------------------
    // ROS callbacks
    // ------------------------------------------------------------------

    /**
     * @brief Receive a raw LiDAR scan, filter it, build segments, and
     *        store results for the next control loop.
     *
     * @param msg  Incoming `sensor_msgs/LaserScan` from `/scan`.
     *
     * @details
     * Runs `filterLaserScan()` on the raw scan, stores the filtered result
     * in `latest_scan_`, builds `LaserSegment` clusters via
     * `buildLaserSegments()` and stores them in `latest_segments_`, then
     * republishes the filtered scan on `/scan_filtered` for RViz debugging.
     */
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Receive an IMU message and cache the latest orientation.
     *
     * @param msg  Incoming `sensor_msgs/Imu` from `/imu`.
     *
     * @details
     * Stores the message in `latest_imu_`, sets `imu_available_` to true on
     * the first valid message.  The orientation quaternion is consumed by
     * the ALIGNING state for IMU-confirmed rotation and by the
     * HUMAN_DETECTED state for target-point propagation.
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief Edge-trigger callback for human lock acquire / loss events.
     *
     * @param msg  `std_msgs/Bool` on `/human_detected`.
     *             `true`  = camera has acquired a human lock.
     *             `false` = camera has lost the lock.
     *
     * @details
     * On a rising edge (`true`): transitions the navigation state to
     * HUMAN_DETECTED (subject to detection cooldown), resets the human
     * motion controller, and captures a safety-zone baseline scan.
     * On a falling edge (`false`): only records the loss; the HUMAN_DETECTED
     * handler manages the recovery timeout and eventual return to SEARCHING.
     */
    void humanDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Continuous tracking heartbeat from the camera pipeline.
     *
     * @param msg  `std_msgs/Float32MultiArray` on `/human_tracking`.
     *             `data[0]` = detected (1.0 / 0.0).
     *             `data[1]` = centre_offset (−0.5 = hard left, 0 = centred, +0.5 = hard right).
     *             `data[2]` = distance_m (always −1.0 from camera; behaviour uses LiDAR).
     *
     * @details
     * Updates `human_locked_`, `human_centre_offset_`, `human_distance_m_`,
     * and `last_human_tracking_time_`.  On a rising edge (previously not
     * locked, now locked) transitions to HUMAN_DETECTED if not in cooldown.
     */
    void humanTrackingCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // ------------------------------------------------------------------
    // Movement helpers
    // ------------------------------------------------------------------

    /**
     * @brief Smoothly decelerate `twist` toward zero.
     *
     * @param[in,out] twist  Command to modify.  Linear X is reduced by
     *                       `kStopLinearDecel * kControlPeriodSeconds` per
     *                       call; angular Z by `kStopAngularDecel *
     *                       kControlPeriodSeconds`.  Stops clamped at zero.
     */
    void stopRobot(geometry_msgs::msg::Twist &twist);

    // ------------------------------------------------------------------
    // LaserScan filtering and helper functions
    // ------------------------------------------------------------------

    /**
     * @brief Two-stage noise filter for a raw `sensor_msgs/LaserScan`.
     *
     * @param raw_scan  Unfiltered scan from the YDLIDAR X4.
     * @return          Copy of `raw_scan` with noisy/isolated returns
     *                  replaced by `std::numeric_limits<float>::infinity()`.
     *
     * @details
     * **Stage 1 — local neighbour test:** A return is considered a noise
     * candidate if fewer than `kNoiseMinNeighbourCount` of the
     * `±kNoiseNeighbourWindow` surrounding beams are within
     * `kNoiseNeighbourDistance` metres in Cartesian space.
     *
     * **Stage 2 — segment extraction test:** The surviving returns are grouped
     * into `LaserSegment` clusters.  Clusters with fewer than
     * `kSegmentMinPoints` points or a Cartesian length shorter than
     * `kSegmentMinLength` are discarded (returns set to infinity).
     */
    sensor_msgs::msg::LaserScan filterLaserScan(const sensor_msgs::msg::LaserScan &raw_scan);

    /**
     * @brief Extract contiguous Cartesian clusters from a filtered scan.
     *
     * @param scan  A filtered `LaserScan` (as returned by `filterLaserScan()`).
     * @return      Vector of `LaserSegment` objects, each representing one
     *              contiguous object or wall segment detected in the scan.
     *
     * @details
     * Consecutive valid returns are joined into a segment whenever their
     * Cartesian separation is less than `kSegmentJoinDistance`.  Segments
     * shorter than `kSegmentMinLength` or with fewer than `kSegmentMinPoints`
     * points are discarded.  The midpoint, min_range, and midpoint_angle fields
     * are populated for each surviving segment.
     */
    std::vector<LaserSegment> buildLaserSegments(const sensor_msgs::msg::LaserScan &scan) const;

    /**
     * @brief Convert a single scan beam to a Cartesian point in the robot frame.
     *
     * @param scan   The scan containing angle metadata.
     * @param index  Zero-based beam index within `scan.ranges`.
     * @return       `geometry_msgs::msg::Point` with `z = 0`.
     */
    geometry_msgs::msg::Point polarToPoint(const sensor_msgs::msg::LaserScan &scan, int index) const;

    /**
     * @brief Check whether a range value is usable as a reference for the
     *        neighbour-test stage of the noise filter.
     *
     * @param scan   Scan containing `range_min` and `range_max`.
     * @param range  Range value to test.
     * @return       `true` if the value is finite, above `kMinValidRange`,
     *               and within the sensor's stated limits.
     */
    bool isRangeUsableForFiltering(const sensor_msgs::msg::LaserScan &scan, double range) const;

    /**
     * @brief Check whether a range value represents a real obstacle return.
     *
     * @param range  Range value to test.
     * @return       `true` if `range` is finite and ≥ `kMinValidRange`.
     */
    bool isRangeValid(double range) const;

    /**
     * @brief Find the heading and representative range of the longest clear
     *        arc in the current filtered scan, using DVD-bounce selection.
     *
     * @param[out] out_angle  Heading (radians, robot frame) of the chosen arc.
     * @param[out] out_range  Representative range (metres) in that direction.
     * @return                `true` if a valid heading was found;
     *                        `false` if the scan contains no usable returns.
     *
     * @details
     * Scans for arcs where all beams exceed `kDvdOpenClearanceDistance` or are
     * infinite, rejects arcs narrower than `kDvdMinSectorWidth`, trims edges
     * by `kDvdSectorEdgeMargin`, then randomly selects from candidates that
     * fall in the preferred 55°–150° side-bounce band (falling back to a
     * wider band when no preferred candidates exist).
     */
    bool getLongestRange(double &out_angle, double &out_range) const;

    /**
     * @brief Return the minimum filtered range within an angular window.
     *
     * @param start_angle  Start of the window (radians, robot frame).
     * @param end_angle    End of the window (radians, robot frame).
     * @return             Minimum valid range in metres, or
     *                     `std::numeric_limits<double>::infinity()` if no
     *                     valid return exists in the window.
     */
    double getMinimumRange(double start_angle, double end_angle) const;

    /**
     * @brief Return the minimum range in the forward ±`kFrontCheckAngle` arc.
     *
     * @return Minimum valid range ahead of the robot in metres, or
     *         `std::numeric_limits<double>::infinity()` if the arc is clear.
     */
    double getFrontRange() const;

    /**
     * @brief Convert a bearing in radians to the corresponding scan array index.
     *
     * @param angle_rad  Bearing in radians (robot frame).  Normalised to
     *                   [angle_min, angle_max] before conversion.
     * @return           Zero-based index clamped to `[0, N-1]`.
     */
    int angleToIndex(double angle_rad) const;

    /**
     * @brief Wrap an angle into [−π, +π].
     *
     * @param angle_rad  Input angle in radians.
     * @return           Equivalent angle in [−π, +π].
     */
    double normaliseAngle(double angle_rad) const;

    /**
     * @brief Test whether `angle_rad` lies inside the arc
     *        [start_angle, end_angle] (wrapping correctly around ±π).
     *
     * @param angle_rad    Bearing to test (radians).
     * @param start_angle  Arc start (radians).
     * @param end_angle    Arc end (radians).
     * @return             `true` if the bearing is inside the arc.
     */
    bool angleInsideWindow(double angle_rad, double start_angle, double end_angle) const;

    /**
     * @brief Find all `LaserSegment` clusters that intersect the forward
     *        ±`kFrontCheckAngle` cone and are closer than `stop_distance_m_`.
     *
     * @param[out] blocking_segments  Vector populated with any segments that
     *                                would block forward motion.
     * @return                        `true` if any blocking segments were found.
     */
    bool findBlockingObstaclesInFront(std::vector<LaserSegment> &blocking_segments) const;

    /**
     * @brief Test whether a segment overlaps a given angular window.
     *
     * @param segment     Segment to test (uses `start_point` and `end_point`).
     * @param start_angle Arc start (radians, robot frame).
     * @param end_angle   Arc end (radians, robot frame).
     * @return            `true` if any part of the segment subtends the window.
     */
    bool segmentOverlapsAngleWindow(const LaserSegment &segment, double start_angle, double end_angle) const;

    /**
     * @brief Debug helper: write the current longest-range calculation to
     *        the debug log file `longest_scan_debug.txt`.
     */
    void longestLaserScan();

    /**
     * @brief Return the nearest LiDAR return within a narrow cone centred on
     *        the camera-estimated human bearing.
     *
     * @param centre_offset  Normalised image offset: −0.5 = hard left,
     *                       0 = centred, +0.5 = hard right.
     * @return               Nearest valid range in metres within
     *                       ±`kHumanLidarWindow` of the estimated bearing, or
     *                       `std::numeric_limits<double>::infinity()` if none.
     */
    double getHumanLidarRange(double centre_offset) const;

    // ------------------------------------------------------------------
    // Safety-zone helpers
    // ------------------------------------------------------------------

    /**
     * @brief Snapshot the current filtered scan as the safety-zone baseline.
     *
     * @details
     * Called once on entry to HUMAN_DETECTED.  The baseline represents the
     * expected environment around the robot before the visitor enters its
     * personal space.  Subsequent calls to `isSafetyZoneViolated()` compare
     * the live scan against this baseline.
     */
    void captureSafetyZoneBaseline();

    /**
     * @brief Determine whether an unexpected object has entered the safety zone.
     *
     * @param human_bearing_rad  Current estimated human bearing (radians,
     *                           robot frame).  Beams within
     *                           ±`kSafetyZoneHumanExclusionAngle` of this
     *                           bearing are excluded from the comparison to
     *                           avoid flagging the tracked visitor.
     * @return                   `true` if any non-human return is closer than
     *                           `kSafetyZoneRadius` AND is significantly closer
     *                           than the baseline at the same bearing.
     */
    bool isSafetyZoneViolated(double human_bearing_rad) const;

    // ------------------------------------------------------------------
    // RViz marker helpers
    // ------------------------------------------------------------------

    /**
     * @brief Publish sphere markers in RViz for each blocking segment.
     *
     * @param blocking_segments  Segments to visualise (midpoints are used
     *                           as marker positions).
     */
    void publishObstacleMarkers(const std::vector<LaserSegment> &blocking_segments);

    /**
     * @brief Clear all obstacle markers from RViz by publishing an empty
     *        `DELETE_ALL` marker array.
     */
    void clearObstacleMarkers();

    // ------------------------------------------------------------------
    // ROS publishers / subscribers / timer
    // ------------------------------------------------------------------

    /** @brief Subscriber for the raw YDLIDAR X4 scan on `/scan`. */
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;

    /** @brief Subscriber for Sense HAT IMU data on `/imu`. */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    /** @brief Publisher for velocity commands on `/cmd_vel`. */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    /**
     * @brief Publisher for the noise-filtered scan on `/scan_filtered`.
     *        Subscribe to this in RViz instead of `/scan` to see what the
     *        behaviour node actually reacts to.
     */
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_publisher_;

    /**
     * @brief Publisher for RViz obstacle markers on `/behaviour_obstacle_markers`.
     *        Highlights the LiDAR segments that caused the robot to stop.
     */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_publisher_;

    /**
     * @brief Subscriber for the camera lock edge-trigger on `/human_detected`.
     *        Receives a `true` pulse when the camera acquires a human lock
     *        and a `false` pulse when the lock is lost.
     */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_detected_subscriber_;

    /**
     * @brief Subscriber for the continuous tracking heartbeat on `/human_tracking`.
     *        Message format: `[detected (0/1), centre_offset (−0.5…+0.5), distance_m]`.
     *        `distance_m` is always −1.0 from the camera; the behaviour uses
     *        LiDAR for all range decisions.
     */
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr human_tracking_subscriber_;

    /** @brief 100 ms wall-clock timer driving `controlLoop()`. */
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ------------------------------------------------------------------
    // Cached sensor data
    // ------------------------------------------------------------------

    /** @brief Most recent noise-filtered `LaserScan` (populated by `laserScanCallback()`). */
    sensor_msgs::msg::LaserScan latest_scan_;

    /** @brief Segment list derived from `latest_scan_` (populated by `laserScanCallback()`). */
    std::vector<LaserSegment> latest_segments_;

    /** @brief Most recent IMU message (populated by `imuCallback()`). */
    sensor_msgs::msg::Imu latest_imu_;

    /** @brief Last Twist command that was published on `/cmd_vel`.
     *         Used by the human-target propagator to estimate base motion. */
    geometry_msgs::msg::Twist current_twist_;

    // ------------------------------------------------------------------
    // Human tracking data
    // ------------------------------------------------------------------

    /** @brief `true` while the camera reports a detected / locked human. */
    bool human_locked_;

    /**
     * @brief Normalised horizontal image offset of the detected human.
     *        −0.5 = person at left edge, 0 = centred, +0.5 = right edge.
     *        Updated by `humanTrackingCallback()`.
     */
    double human_centre_offset_;

    /**
     * @brief Camera-reported distance to the human (metres).
     *        Always −1.0 because the Pi camera has no depth sensor.
     *        The behaviour uses LiDAR for all range decisions.
     */
    double human_distance_m_;

    /** @brief ROS time of the most recent `/human_tracking` message.
     *         Used to detect tracking timeout in HUMAN_DETECTED. */
    rclcpp::Time last_human_tracking_time_;

    // ------------------------------------------------------------------
    // Behaviour state
    // ------------------------------------------------------------------

    /** @brief `true` after `blindAutonomous()` is called; enables `controlLoop()`. */
    bool blind_autonomous_active_;

    /**
     * @brief `true` while an unexpected non-human object is inside the
     *        safety zone (`kSafetyZoneRadius`).  Set by
     *        `isSafetyZoneViolated()` and checked by the HUMAN_DETECTED
     *        handler to pause arm interaction.
     */
    bool safety_zone_violated_;

    /**
     * @brief Baseline scan captured the first time the robot enters
     *        HUMAN_DETECTED.  Used by `isSafetyZoneViolated()` to
     *        distinguish the tracked visitor from unexpected intruders.
     */
    sensor_msgs::msg::LaserScan safety_zone_baseline_scan_;

    /** @brief `true` once `captureSafetyZoneBaseline()` has run for the
     *         current HUMAN_DETECTED session. */
    bool safety_zone_baseline_captured_;

    /** @brief Current navigation FSM state. */
    NavigationState current_state_;

    /**
     * @brief Target heading selected by the SEARCHING state (radians,
     *        robot frame).  Consumed and tracked by ALIGNING.
     */
    double target_angle_;

    /**
     * @brief Representative range in the direction of `target_angle_`
     *        (metres).  Stored for logging; not used for control after
     *        ALIGNING begins.
     */
    double target_range_;

    /**
     * @brief Stop-approach distance threshold (metres).
     *        Loaded from the `stop_distance_m` ROS parameter on startup.
     *        Default: 1.5 m (simulation).  Physical robot: 0.75 m.
     */
    double stop_distance_m_;

    /**
     * @brief Loop counter for the STOPPED state.
     *        Incremented each control tick; triggers transition back to
     *        SEARCHING when ≥ `kStopDurationLoops` (30 loops = 3 seconds).
     */
    int stop_counter_;

    /**
     * @brief `true` once at least one `/imu` message has been received.
     *        Enables IMU-confirmed rotation in the ALIGNING state.
     */
    bool imu_available_;

    /**
     * @brief IMU yaw recorded at the moment the ALIGNING state begins.
     *        Used as the reference for computing `yaw_turned`.
     */
    double align_start_yaw_;

    /**
     * @brief `true` once `align_start_yaw_` has been captured for the
     *        current ALIGNING cycle.  Reset to `false` whenever the robot
     *        re-enters ALIGNING from SEARCHING.
     */
    bool align_yaw_initialised_;

    // ------------------------------------------------------------------
    // Random tools
    // ------------------------------------------------------------------

    /** @brief Seeded Mersenne Twister used by the DVD-bounce heading picker. */
    std::default_random_engine random_engine_;

    /**
     * @brief Uniform real distribution over [−1, 1].
     *        Used when randomly choosing among candidate DVD-bounce headings.
     */
    std::uniform_real_distribution<double> turn_dist_;
};

#endif  // BEHAVIOUR_HPP

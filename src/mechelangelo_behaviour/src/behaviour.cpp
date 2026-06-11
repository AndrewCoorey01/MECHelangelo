#include "behaviour.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================
//  MECHELANGELO BEHAVIOUR  (simplified)
// ============================================================
// Philosophy of this rewrite:
//   * One simple state machine: SEARCHING -> ALIGNING -> MOVING -> STOPPED,
//     plus a HUMAN_DETECTED state that is now short and readable.
//   * TWO SCANS, TWO JOBS:
//       - The FILTERED scan (noise removed) is used for obstacle avoidance.
//       - The RAW scan, but only inside a narrow cone around the camera
//         bearing, is used to MEASURE THE PERSON'S DISTANCE. This is why the
//         person never gets "filtered out of themselves": we never rely on the
//         de-noised scan to see the person.
//   * Emergency stops while approaching only look at the FORWARD corridor,
//     exclude the person, and require a COHERENT CLUSTER (>=3 adjacent beams).
//     A single random dot, or anything off to the side, can no longer stop us.
// ============================================================

// ---- Control loop ----
static constexpr double kControlPeriodSeconds = 0.1;

// ---- Exploration movement tuning ----
static constexpr double kForwardSpeed = 0.26;        // m/s
static constexpr double kAlignmentTolerance = 0.10;  // rad (~5.7 deg)

// Smooth stops so the base does not snap to zero in one tick.
static constexpr double kStopLinearDecel = 0.4;   // m/s^2
static constexpr double kStopAngularDecel = 1.2;  // rad/s^2

// ---- Physical relative-turn control (gyro magnitude integration) ----
// The fused quaternion yaw is unreliable due to magnetic interference, so
// exploration turns integrate |gyro_z| over each turn instead.
static constexpr double kPhysicalTurnMaxSpeed = 0.45;     // rad/s
static constexpr double kPhysicalTurnMediumSpeed = 0.45;  // rad/s
static constexpr double kPhysicalTurnSlowSpeed = 0.45;    // rad/s
static constexpr double kPhysicalTurnMediumZone = 30.0 * M_PI / 180.0;
static constexpr double kPhysicalTurnSlowZone = 10.0 * M_PI / 180.0;
static constexpr double kGyroDeadband = 0.015;  // rad/s
static constexpr int kGyroBiasSampleCount = 8;
static constexpr double kImuFreshTimeout = 0.40;  // s
static constexpr double kMaximumTurnTime = 20.0;  // s

// Stop this far before an obstacle while exploring (ROS param 'stop_distance_m').
// 30 loops * 0.1 s = 3 s pause after a stop.
static constexpr int kStopDurationLoops = 30;
static constexpr double kFrontCheckAngle = 30 * M_PI / 180.0;
static constexpr double kLongestRangeTieTolerance = 0.05;  // m

// ---- Self-mask (ROS param 'self_mask_range_m') ----
// The physical robot sees its own frame/cables/body at close range. Set this to
// ~0.75 on the real robot (you measured body returns out to ~0.71 m). Leave at
// 0.0 in simulation. Stored in a file-scope global so the const helpers can read
// it without changing behaviour.hpp.
static double g_self_mask_range = 0.0;

// ---- DVD-style bounce exploration ----
static constexpr double kDvdOpenClearanceDistance = 4.5;  // m
static constexpr double kDvdMinSectorWidth = 18.0 * M_PI / 180.0;
static constexpr double kDvdSectorEdgeMargin = 8.0 * M_PI / 180.0;
static constexpr double kDvdPreferredMinTurnAngle = 55.0 * M_PI / 180.0;
static constexpr double kDvdPreferredMaxTurnAngle = 150.0 * M_PI / 180.0;
static constexpr double kDvdAvoidFrontAngle = 35.0 * M_PI / 180.0;
static constexpr double kDvdAvoidReverseAngle = 165.0 * M_PI / 180.0;

// ---- LaserScan noise suppression (used for the FILTERED scan) ----
static constexpr int kNoiseNeighbourWindow = 4;
static constexpr int kNoiseMinNeighbourCount = 2;
static constexpr double kNoiseNeighbourDistance = 0.25;  // m
static constexpr double kSegmentJoinDistance = 0.22;     // m
static constexpr int kSegmentMinPoints = 3;
static constexpr double kSegmentMinLength = 0.03;  // m

// ============================================================
//  Human approach tuning  (simple, camera bearing + LiDAR range)
// ============================================================
// /human_tracking message: data[0]=detected, data[1]=centre_offset, data[2]=unused.
// centre_offset is normalised: -0.5 = far left, 0 = centred, +0.5 = far right.
// camera_bearing = -centre_offset * FOV  (ROS: +left, -right).
static constexpr double kCameraHorizontalFov = 60.0 * M_PI / 180.0;

// Turn in place toward the person when they are off-centre by more than this
// (normalised image fraction). This is what keeps the person in frame.
static constexpr double kHumanCentreDeadZone = 0.10;
static constexpr double kHumanTurnSpeed = 0.45;  // rad/s (physical minimum)

// Forward approach.
static constexpr double kHumanApproachSpeed = 0.12;  // m/s
static constexpr double kHumanNearSpeed = 0.07;      // m/s when close
static constexpr double kHumanSlowdownRange = 2.2;   // m, slow down inside this
static constexpr double kHumanCreepSpeed = 0.06;     // m/s when range unknown
static constexpr double kHumanForwardAccel = 0.30;   // m/s^2

// Interaction distance.
static constexpr double kHumanInteractionDistance = 1.80;  // m
static constexpr double kHumanInteractionTolerance = 0.20; // m

// Person distance is measured from the RAW scan inside this cone around the
// camera bearing. Sparse leg returns are fine here because the camera has
// already confirmed a person is in that direction.
static constexpr double kHumanRangeConeHalfAngle = 12.0 * M_PI / 180.0;
static constexpr double kHumanRangeJumpReject = 0.50;  // m, reject sudden jumps
static constexpr double kHumanRangeAlpha = 0.40;       // EMA smoothing
static constexpr double kHumanRangeStaleTimeout = 0.60; // s

// Forward emergency obstacle check (FILTERED scan). Only the forward corridor
// is considered, the person cone is excluded, and a coherent multi-beam cluster
// is required so noise / side returns cannot trigger a false stop.
static constexpr double kHumanForwardEmergencyHalfAngle = 28.0 * M_PI / 180.0;
static constexpr double kHumanPersonExclusionAngle = 20.0 * M_PI / 180.0;
static constexpr double kHumanEmergencyStopRange = 1.00;  // m
static constexpr int kHumanEmergencyClusterMinPoints = 3;
static constexpr double kHumanEmergencyClusterRangeGap = 0.25;  // m

// Visual loss / interaction session timing.
static constexpr double kHumanLostTimeout = 4.0;  // s of no detection -> explore
// The bridge publishes at 10 Hz even on error. If messages stop entirely (bridge
// node dead) for longer than this, treat the person as not currently seen.
static constexpr double kHumanTrackingMsgTimeout = 1.0;  // s
static constexpr double kHumanInteractionDurationSeconds = 30.0;
static constexpr double kHumanDetectionCooldownSeconds = 10.0;

// Arm pose keepalive.
static constexpr double kArmDownRepublishPeriod = 0.50;

// ============================================================
//  File-scope state (kept out of the class to avoid behaviour.hpp edits)
// ============================================================

// IMU freshness.
static bool g_latest_imu_time_valid = false;
static rclcpp::Time g_latest_imu_receive_time;

// Gyro alignment state for a single exploration turn.
static bool g_align_gyro_calibrating = false;
static bool g_align_gyro_active = false;
static double g_align_gyro_angle = 0.0;
static double g_align_gyro_bias = 0.0;
static double g_align_gyro_bias_sum = 0.0;
static int g_align_gyro_bias_samples = 0;
static rclcpp::Time g_align_last_imu_time;
static rclcpp::Time g_align_start_time;

// Raw scan (unfiltered) snapshot for the human range cone.
static sensor_msgs::msg::LaserScan g_latest_raw_scan;
static bool g_latest_raw_scan_valid = false;
static rclcpp::Time g_latest_raw_scan_receive_time;

// Smoothed human range.
static bool g_human_range_valid = false;
static double g_human_range = -1.0;
static rclcpp::Time g_human_range_time;

// Grace against brief lock loss. The Pi 4 bridge publishes detected=0 on any
// network blip ("Cannot reach Pi 4"), which would otherwise make us abandon the
// person instantly. This remembers the last time the lock was genuinely held.
static bool g_last_human_seen_valid = false;
static rclcpp::Time g_last_human_seen_time;

// Flip left/right turning without recompiling (the camera is mounted inverted).
// Set ROS param 'human_turn_sign' to -1.0 if the robot turns AWAY from the person.
static double g_human_turn_sign = 1.0;

// Interaction session + cooldown.
static bool g_interaction_session_active = false;
static rclcpp::Time g_interaction_session_start_time;
static bool g_human_detection_cooldown_active = false;
static rclcpp::Time g_human_detection_cooldown_start_time;

// Latest forward emergency obstruction (for logging only).
static double g_safety_obstruction_bearing =
    std::numeric_limits<double>::quiet_NaN();
static double g_safety_obstruction_range =
    std::numeric_limits<double>::infinity();

// Arm / interaction publishers and mimicry forwarding.
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_right_arm_pose_publisher;
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_left_arm_pose_publisher;
static rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr g_interaction_active_publisher;
static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_right_mimicry_pose_subscriber;
static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_left_mimicry_pose_subscriber;
static rclcpp::Time g_last_arm_down_publish_time;
static bool g_arm_down_publish_time_initialised = false;
static rclcpp::Time g_last_interaction_active_publish_time;
static bool g_interaction_active_publish_time_initialised = false;
static bool g_last_interaction_active_value = false;
static std::string g_approach_arm_pose_name = "arm_down";

// ============================================================
//  Small free helpers
// ============================================================

static double normaliseAngleFree(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static const char *relativeDirectionName(double angle_rad)
{
    if (!std::isfinite(angle_rad)) return "UNKNOWN";
    const double deg = normaliseAngleFree(angle_rad) * 180.0 / M_PI;
    if (deg >= -22.5 && deg < 22.5) return "FRONT";
    if (deg >= 22.5 && deg < 67.5) return "FRONT_LEFT";
    if (deg >= 67.5 && deg < 112.5) return "LEFT";
    if (deg >= 112.5 && deg < 157.5) return "REAR_LEFT";
    if (deg >= 157.5 || deg < -157.5) return "REAR";
    if (deg >= -157.5 && deg < -112.5) return "REAR_RIGHT";
    if (deg >= -112.5 && deg < -67.5) return "RIGHT";
    return "FRONT_RIGHT";
}

static void publishArmsDown(const rclcpp::Time &now, bool force = false)
{
    if (!g_right_arm_pose_publisher || !g_left_arm_pose_publisher) return;

    const bool period_elapsed = !g_arm_down_publish_time_initialised ||
        (now - g_last_arm_down_publish_time).seconds() >= kArmDownRepublishPeriod;
    if (!force && !period_elapsed) return;

    std_msgs::msg::String pose;
    pose.data = g_approach_arm_pose_name;
    g_right_arm_pose_publisher->publish(pose);
    g_left_arm_pose_publisher->publish(pose);
    g_last_arm_down_publish_time = now;
    g_arm_down_publish_time_initialised = true;
}

static void publishInteractionActive(const rclcpp::Time &now, bool active, bool force = false)
{
    if (!g_interaction_active_publisher) return;

    const bool changed = active != g_last_interaction_active_value;
    const bool period_elapsed = !g_interaction_active_publish_time_initialised ||
        (now - g_last_interaction_active_publish_time).seconds() >= 0.50;
    if (!force && !changed && !period_elapsed) return;

    std_msgs::msg::Bool msg;
    msg.data = active;
    g_interaction_active_publisher->publish(msg);
    g_last_interaction_active_publish_time = now;
    g_last_interaction_active_publish_time_initialised = true;
    g_last_interaction_active_value = active;
}

static bool humanDetectionCooldownActive(const rclcpp::Time &now)
{
    if (!g_human_detection_cooldown_active) return false;
    return (now - g_human_detection_cooldown_start_time).seconds() <
        kHumanDetectionCooldownSeconds;
}

static double humanDetectionCooldownRemaining(const rclcpp::Time &now)
{
    if (!g_human_detection_cooldown_active) return 0.0;
    const double elapsed = (now - g_human_detection_cooldown_start_time).seconds();
    return std::max(0.0, kHumanDetectionCooldownSeconds - elapsed);
}

static void resetHumanApproach()
{
    g_human_range_valid = false;
    g_human_range = -1.0;
    g_interaction_session_active = false;
}

// Complete Claude's intended network/lock grace handling.
static void markHumanSeen(const rclcpp::Time &now)
{
    g_last_human_seen_time = now;
    g_last_human_seen_valid = true;
}

static void clearHumanSeenGrace()
{
    g_last_human_seen_valid = false;
}

// ============================================================
//  Constructor / lifecycle
// ============================================================

MechelangeloBehaviour::MechelangeloBehaviour()
: Node("mechelangelo_behaviour"),
  human_locked_(false),
  human_centre_offset_(0.0),
  human_distance_m_(-1.0),
  blind_autonomous_active_(false),
  safety_zone_violated_(false),
  safety_zone_baseline_captured_(false),
  current_state_(NavigationState::SEARCHING),
  target_angle_(0.0),
  target_range_(0.0),
  stop_distance_m_(1.5),
  stop_counter_(0),
  imu_available_(false),
  align_start_yaw_(0.0),
  align_yaw_initialised_(false),
  random_engine_(std::random_device{}()),
  turn_dist_(-1.0, 1.0)
{
    this->declare_parameter("stop_distance_m", 1.5);
    stop_distance_m_ = this->get_parameter("stop_distance_m").as_double();

    this->declare_parameter("self_mask_range_m", 0.0);
    g_self_mask_range = this->get_parameter("self_mask_range_m").as_double();

    this->declare_parameter("approach_arm_pose_name", "arm_down");
    g_approach_arm_pose_name = this->get_parameter("approach_arm_pose_name").as_string();

    this->declare_parameter("human_turn_sign", 1.0);
    g_human_turn_sign =
        this->get_parameter("human_turn_sign").as_double() < 0.0 ? -1.0 : 1.0;

    RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_m_);
    RCLCPP_INFO(this->get_logger(), "Self-mask range: %.2f m (0 = sim, ~0.75 = physical)",
                g_self_mask_range);
    RCLCPP_INFO(this->get_logger(), "Non-interaction arm hold pose: %s",
                g_approach_arm_pose_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Human turn sign: %+.0f", g_human_turn_sign);

    g_right_arm_pose_publisher =
        this->create_publisher<std_msgs::msg::String>("/arm/right_pose", 10);
    g_left_arm_pose_publisher =
        this->create_publisher<std_msgs::msg::String>("/arm/left_pose", 10);
    g_interaction_active_publisher =
        this->create_publisher<std_msgs::msg::Bool>(
            "/interaction_active",
            rclcpp::QoS(1).reliable().transient_local());

    // Forward mimicry arm poses to the real arm topics only during interaction.
    g_right_mimicry_pose_subscriber =
        this->create_subscription<std_msgs::msg::String>(
            "/arm/mimicry_right_pose", 10,
            [](const std_msgs::msg::String::SharedPtr msg)
            {
                if (g_interaction_session_active && g_right_arm_pose_publisher)
                    g_right_arm_pose_publisher->publish(*msg);
            });
    g_left_mimicry_pose_subscriber =
        this->create_subscription<std_msgs::msg::String>(
            "/arm/mimicry_left_pose", 10,
            [](const std_msgs::msg::String::SharedPtr msg)
            {
                if (g_interaction_session_active && g_left_arm_pose_publisher)
                    g_left_arm_pose_publisher->publish(*msg);
            });

    g_last_arm_down_publish_time = this->now();
    g_last_interaction_active_publish_time = this->now();

    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&MechelangeloBehaviour::laserScanCallback, this, std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SensorDataQoS(),
        std::bind(&MechelangeloBehaviour::imuCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan_filtered", rclcpp::SensorDataQoS());

    obstacle_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/behaviour_obstacle_markers", 10);

    human_detected_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/human_detected", 10,
        std::bind(&MechelangeloBehaviour::humanDetectedCallback, this, std::placeholders::_1));

    human_tracking_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/human_tracking", 10,
        std::bind(&MechelangeloBehaviour::humanTrackingCallback, this, std::placeholders::_1));

    last_human_tracking_time_ = this->now();

    control_timer_ = this->create_wall_timer(
        100ms, std::bind(&MechelangeloBehaviour::controlLoop, this));

    publishInteractionActive(this->now(), false, true);
    publishArmsDown(this->now(), true);
    RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been started.");
}

MechelangeloBehaviour::~MechelangeloBehaviour()
{
    RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been stopped.");
}

void MechelangeloBehaviour::run(bool sim_mode)
{
    RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node is running.");
    if (sim_mode)
    {
        RCLCPP_INFO(this->get_logger(), "Running in simulation mode.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Running in real robot mode.");
        if (g_self_mask_range <= 0.01)
        {
            RCLCPP_WARN(this->get_logger(),
                "self_mask_range_m is 0 on the physical robot. The LiDAR will see "
                "the robot's own body. Set self_mask_range_m:=0.75 in the launch file.");
        }
    }

    blindAutonomous();
    rclcpp::spin(shared_from_this());
}

void MechelangeloBehaviour::blindAutonomous()
{
    RCLCPP_INFO(this->get_logger(), "Executing blind autonomous behaviour.");
    blind_autonomous_active_ = true;
    safety_zone_violated_ = false;
    safety_zone_baseline_captured_ = false;
    current_state_ = NavigationState::SEARCHING;
    target_angle_ = 0.0;
    target_range_ = 0.0;
    stop_counter_ = 0;
    align_yaw_initialised_ = false;
    g_align_gyro_calibrating = false;
    g_align_gyro_active = false;
    g_align_gyro_angle = 0.0;
    g_align_gyro_bias = 0.0;
    g_align_gyro_bias_sum = 0.0;
    g_align_gyro_bias_samples = 0;
    resetHumanApproach();
    clearHumanSeenGrace();
    clearObstacleMarkers();
}

void MechelangeloBehaviour::mappedAutonomous()
{
    RCLCPP_INFO(this->get_logger(), "Executing mapped autonomous behaviour.");
}

// ============================================================
//  Main control loop
// ============================================================

void MechelangeloBehaviour::controlLoop()
{
    if (!blind_autonomous_active_) return;

    geometry_msgs::msg::Twist twist;
    const rclcpp::Time now = this->now();

    // Interaction gate + arm keepalive.
    publishInteractionActive(now, g_interaction_session_active);
    if (!g_interaction_session_active) publishArmsDown(now);

    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Waiting for valid filtered LaserScan data...");
        stopRobot(twist);
        current_twist_ = twist;
        cmd_vel_publisher_->publish(twist);
        return;
    }

    switch (current_state_)
    {
    // --------------------------------------------------------
    case NavigationState::SEARCHING:
    {
        stopRobot(twist);
        clearObstacleMarkers();

        double longest_angle = 0.0;
        double longest_range = 0.0;
        if (!getLongestRange(longest_angle, longest_range))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "SEARCHING: No valid filtered LaserScan range found.");
            break;
        }

        target_angle_ = longest_angle;
        target_range_ = longest_range;
        g_align_gyro_calibrating = false;
        g_align_gyro_active = false;
        g_align_gyro_angle = 0.0;
        g_align_gyro_bias_sum = 0.0;
        g_align_gyro_bias_samples = 0;

        RCLCPP_INFO(this->get_logger(),
            "SEARCHING: Selected heading %.1f deg, range %.2f m",
            target_angle_ * 180.0 / M_PI, target_range_);
        current_state_ = NavigationState::ALIGNING;
        break;
    }

    // --------------------------------------------------------
    case NavigationState::ALIGNING:
    {
        clearObstacleMarkers();
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;

        const bool imu_fresh = imu_available_ && g_latest_imu_time_valid &&
            (now - g_latest_imu_receive_time).seconds() <= kImuFreshTimeout;

        if (!imu_fresh)
        {
            g_align_gyro_calibrating = false;
            g_align_gyro_active = false;
            g_align_gyro_angle = 0.0;
            g_align_gyro_bias_sum = 0.0;
            g_align_gyro_bias_samples = 0;
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "ALIGNING (gyro): IMU stale. Holding motors at zero.");
            break;
        }

        const double target_magnitude = std::fabs(target_angle_);
        if (target_magnitude <= kAlignmentTolerance)
        {
            g_align_gyro_calibrating = false;
            g_align_gyro_active = false;
            g_align_gyro_angle = 0.0;
            current_state_ = NavigationState::MOVING;
            RCLCPP_INFO(this->get_logger(),
                "ALIGNING (gyro): Already within tolerance. Moving.");
            break;
        }

        if (!g_align_gyro_calibrating && !g_align_gyro_active)
        {
            g_align_gyro_calibrating = true;
            g_align_gyro_angle = 0.0;
            g_align_gyro_bias = 0.0;
            g_align_gyro_bias_sum = 0.0;
            g_align_gyro_bias_samples = 0;
            g_align_last_imu_time = now;
            g_align_start_time = now;
            RCLCPP_INFO(this->get_logger(),
                "ALIGNING (gyro): Calibrating bias before %.1f deg turn.",
                target_angle_ * 180.0 / M_PI);
            break;
        }

        if (g_align_gyro_calibrating)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "ALIGNING (gyro): Bias calibration %d/%d.",
                g_align_gyro_bias_samples, kGyroBiasSampleCount);
            break;
        }

        const double turn_elapsed = (now - g_align_start_time).seconds();
        if (turn_elapsed > kMaximumTurnTime)
        {
            // A stalled turn must not disable the whole robot. Abandon this
            // heading and pick a new one instead of killing autonomy forever.
            g_align_gyro_active = false;
            g_align_gyro_calibrating = false;
            RCLCPP_WARN(this->get_logger(),
                "ALIGNING (gyro): Turn timed out after %.1f s (target %.1f deg, "
                "measured %.1f deg). Picking a new heading.",
                turn_elapsed, target_angle_ * 180.0 / M_PI,
                g_align_gyro_angle * 180.0 / M_PI);
            current_state_ = NavigationState::SEARCHING;
            break;
        }

        const double remaining = target_magnitude - g_align_gyro_angle;
        if (remaining <= kAlignmentTolerance)
        {
            g_align_gyro_active = false;
            g_align_gyro_calibrating = false;
            RCLCPP_INFO(this->get_logger(),
                "ALIGNING (gyro): Turn complete. Moving.");
            current_state_ = NavigationState::MOVING;
            break;
        }

        double speed = kPhysicalTurnSlowSpeed;
        if (remaining > kPhysicalTurnMediumZone) speed = kPhysicalTurnMaxSpeed;
        else if (remaining > kPhysicalTurnSlowZone) speed = kPhysicalTurnMediumSpeed;

        twist.angular.z = std::copysign(speed, target_angle_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "ALIGNING (gyro): target %.1f, measured %.1f, remaining %.1f deg, cmd %.2f rad/s",
            target_angle_ * 180.0 / M_PI, g_align_gyro_angle * 180.0 / M_PI,
            remaining * 180.0 / M_PI, twist.angular.z);
        break;
    }

    // --------------------------------------------------------
    case NavigationState::MOVING:
    {
        std::vector<LaserSegment> blocking_segments;
        const bool blocked = findBlockingObstaclesInFront(blocking_segments);
        const double front_range = getFrontRange();

        if (blocked || front_range <= stop_distance_m_)
        {
            if (blocking_segments.empty())
            {
                LaserSegment fallback;
                fallback.point_count = 1;
                fallback.min_range = front_range;
                fallback.midpoint.x = std::isfinite(front_range) ? front_range : stop_distance_m_;
                blocking_segments.push_back(fallback);
            }
            publishObstacleMarkers(blocking_segments);
            RCLCPP_WARN(this->get_logger(),
                "MOVING: Blocking obstacle in front. Front range = %.2f m. Stopping.",
                front_range);
            stopRobot(twist);
            stop_counter_ = 0;
            current_state_ = NavigationState::STOPPED;
            break;
        }

        clearObstacleMarkers();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "MOVING: Driving forward. Front range = %.2f m", front_range);
        twist.linear.x = kForwardSpeed;
        twist.angular.z = 0.0;
        break;
    }

    // --------------------------------------------------------
    case NavigationState::STOPPED:
    {
        stopRobot(twist);
        stop_counter_++;
        if (stop_counter_ >= kStopDurationLoops)
        {
            RCLCPP_INFO(this->get_logger(), "STOPPED: Pause complete. Searching.");
            stop_counter_ = 0;
            clearObstacleMarkers();
            current_state_ = NavigationState::SEARCHING;
        }
        break;
    }

    // --------------------------------------------------------
    case NavigationState::HUMAN_DETECTED:
    {
        const double time_since_tracking = (now - last_human_tracking_time_).seconds();

        // The bridge publishes at 10 Hz even on error (detected=0 when it cannot
        // reach the Pi 4 camera). Count the person as seen only if the lock is
        // set AND a fresh message arrived; stale messages (bridge dead) count as
        // not seen. A real observation refreshes the grace timer.
        const bool seen_now =
            human_locked_ && time_since_tracking <= kHumanTrackingMsgTimeout;
        if (seen_now)
        {
            markHumanSeen(now);
        }

        const double since_seen = g_last_human_seen_valid
            ? (now - g_last_human_seen_time).seconds()
            : std::numeric_limits<double>::infinity();

        // 1) Finish a timed interaction, or end it early if the person has
        // clearly left during it.
        if (g_interaction_session_active)
        {
            const double elapsed =
                (now - g_interaction_session_start_time).seconds();
            if (elapsed >= kHumanInteractionDurationSeconds ||
                since_seen > kHumanLostTimeout)
            {
                g_interaction_session_active = false;
                g_human_detection_cooldown_active = true;
                g_human_detection_cooldown_start_time = now;
                human_locked_ = false;
                resetHumanApproach();
                clearHumanSeenGrace();
                clearObstacleMarkers();
                stopRobot(twist);
                current_state_ = NavigationState::SEARCHING;
                publishInteractionActive(now, false, true);
                publishArmsDown(now, true);
                RCLCPP_INFO(this->get_logger(),
                    "HUMAN: Interaction ended (%s). Returning to exploration (%.0f s cooldown).",
                    elapsed >= kHumanInteractionDurationSeconds ? "completed" : "person left",
                    kHumanDetectionCooldownSeconds);
                break;
            }

            // Hold the interaction pose. Arms follow mimicry via the subscribers.
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "HUMAN: Interacting. %.1f/%.0f s (last seen %.1f s ago).",
                elapsed, kHumanInteractionDurationSeconds, since_seen);
            break;
        }

        // 2) Not currently seen. A brief loss (camera occlusion, or a network
        // blip) must not make us drive off and abandon the person. Hold position
        // and wait. Only give up after the timeout.
        if (!seen_now)
        {
            if (since_seen <= kHumanLostTimeout)
            {
                stopRobot(twist);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "HUMAN: Lost lock for %.1f s. Holding position, waiting to reacquire.",
                    since_seen);
                break;
            }

            RCLCPP_WARN(this->get_logger(),
                "HUMAN: Person not seen for %.1f s. Returning to exploration.",
                since_seen);
            stopRobot(twist);
            human_locked_ = false;
            resetHumanApproach();
            clearHumanSeenGrace();
            clearObstacleMarkers();
            current_state_ = NavigationState::SEARCHING;
            publishInteractionActive(now, false, true);
            break;
        }

        // Person is currently seen. Bearing comes from the camera's coarse
        // LEFT/RIGHT/CENTRED signal (the bridge smooths it to about +/-0.25).
        // human_turn_sign flips left/right without recompiling if the camera is
        // mounted inverted.
        const double camera_bearing =
            g_human_turn_sign * (-human_centre_offset_ * kCameraHorizontalFov);

        // 3) Measure the person's distance from the RAW scan in a narrow cone
        // around the camera bearing. The person never gets filtered out of
        // their own measurement because this does not use the de-noised scan.
        double raw_range = std::numeric_limits<double>::infinity();
        if (g_latest_raw_scan_valid && !g_latest_raw_scan.ranges.empty() &&
            g_latest_raw_scan.angle_increment != 0.0)
        {
            std::vector<double> cone;
            for (int i = 0; i < static_cast<int>(g_latest_raw_scan.ranges.size()); ++i)
            {
                const double angle = g_latest_raw_scan.angle_min +
                    static_cast<double>(i) * g_latest_raw_scan.angle_increment;
                if (std::fabs(normaliseAngle(angle - camera_bearing)) >
                        kHumanRangeConeHalfAngle)
                    continue;
                const double r = g_latest_raw_scan.ranges[i];
                if (!std::isfinite(r) || r <= g_self_mask_range) continue;
                if (g_latest_raw_scan.range_max > 0.0 && r > g_latest_raw_scan.range_max)
                    continue;
                cone.push_back(r);
            }
            if (!cone.empty())
            {
                std::sort(cone.begin(), cone.end());
                // 20th percentile: close to the nearest body part but robust to
                // a single spurious near return.
                const std::size_t idx = static_cast<std::size_t>(
                    std::floor(0.20 * static_cast<double>(cone.size() - 1)));
                raw_range = cone[idx];
            }
        }

        // Smooth the range and reject impossible jumps.
        if (std::isfinite(raw_range))
        {
            if (!g_human_range_valid)
            {
                g_human_range = raw_range;
                g_human_range_valid = true;
                g_human_range_time = now;
            }
            else if (std::fabs(raw_range - g_human_range) < kHumanRangeJumpReject)
            {
                g_human_range = (1.0 - kHumanRangeAlpha) * g_human_range +
                                kHumanRangeAlpha * raw_range;
                g_human_range_time = now;
            }
            // A larger jump is intentionally ignored rather than replacing the
            // established person range with a single unrelated return.
        }
        else if (g_human_range_valid &&
                 (now - g_human_range_time).seconds() > kHumanRangeStaleTimeout)
        {
            g_human_range_valid = false;
        }

        const bool have_range = g_human_range_valid;
        const double human_range = have_range ? g_human_range : -1.0;

        // 4) Off-centre -> turn in place toward the person (keeps them in frame).
        if (std::fabs(human_centre_offset_) > kHumanCentreDeadZone)
        {
            twist.linear.x = 0.0;
            twist.angular.z = std::copysign(kHumanTurnSpeed, camera_bearing);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "HUMAN: Centring. offset=%.2f cmd=%.2f rad/s range=%.2f m.",
                human_centre_offset_, twist.angular.z, human_range);
            break;
        }

        // 5) Centred. Forward emergency check (filtered scan, forward corridor,
        // person excluded, coherent cluster required). Side/random returns
        // cannot trigger this.
        if (isSafetyZoneViolated(camera_bearing))
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "HUMAN: HOLD. Forward obstacle %s at %+.1f deg / %.2f m.",
                relativeDirectionName(g_safety_obstruction_bearing),
                g_safety_obstruction_bearing * 180.0 / M_PI,
                g_safety_obstruction_range);
            break;
        }

        // 6) Arrived at interaction distance -> stop and start interaction.
        if (have_range &&
            human_range <= kHumanInteractionDistance + kHumanInteractionTolerance)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            g_interaction_session_active = true;
            g_interaction_session_start_time = now;
            publishInteractionActive(now, true, true);
            RCLCPP_INFO(this->get_logger(),
                "HUMAN: Reached interaction distance %.2f m. Starting interaction.",
                human_range);
            break;
        }

        // 7) Approach. Slow down when close; creep slowly if range is unknown
        // (camera still sees the person and the forward corridor is clear).
        double desired_speed;
        if (!have_range)
            desired_speed = kHumanCreepSpeed;
        else if (human_range <= kHumanSlowdownRange)
            desired_speed = kHumanNearSpeed;
        else
            desired_speed = kHumanApproachSpeed;

        const double max_step = kHumanForwardAccel * kControlPeriodSeconds;
        twist.linear.x = current_twist_.linear.x +
            std::clamp(desired_speed - current_twist_.linear.x, -max_step, max_step);
        twist.angular.z = 0.0;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "HUMAN: Approaching. range=%s offset=%.2f cmd=%.2f m/s.",
            have_range ? std::to_string(human_range).c_str() : "unknown",
            human_centre_offset_, twist.linear.x);
        break;
    }

    // --------------------------------------------------------
    default:
    {
        RCLCPP_WARN(this->get_logger(), "Unknown navigation state. Returning to SEARCHING.");
        stopRobot(twist);
        clearObstacleMarkers();
        current_state_ = NavigationState::SEARCHING;
        break;
    }
    }

    current_twist_ = twist;
    cmd_vel_publisher_->publish(twist);
}

// ============================================================
//  Motion helpers
// ============================================================

void MechelangeloBehaviour::stopRobot(geometry_msgs::msg::Twist &twist)
{
    const double linear_step = kStopLinearDecel * kControlPeriodSeconds;
    const double angular_step = kStopAngularDecel * kControlPeriodSeconds;

    auto ramp = [](double value, double max_step)
    {
        if (std::fabs(value) <= max_step) return 0.0;
        return value - std::copysign(max_step, value);
    };

    twist.linear.x = ramp(current_twist_.linear.x, linear_step);
    twist.linear.y = ramp(current_twist_.linear.y, linear_step);
    twist.linear.z = ramp(current_twist_.linear.z, linear_step);
    twist.angular.x = ramp(current_twist_.angular.x, angular_step);
    twist.angular.y = ramp(current_twist_.angular.y, angular_step);
    twist.angular.z = ramp(current_twist_.angular.z, angular_step);
}

// ============================================================
//  Callbacks
// ============================================================

void MechelangeloBehaviour::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    g_latest_raw_scan = *msg;
    g_latest_raw_scan_valid = true;
    g_latest_raw_scan_receive_time = this->now();

    const sensor_msgs::msg::LaserScan filtered = filterLaserScan(*msg);
    latest_scan_ = filtered;
    latest_segments_ = buildLaserSegments(filtered);
    filtered_scan_publisher_->publish(filtered);
}

void MechelangeloBehaviour::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    latest_imu_ = *msg;
    imu_available_ = true;

    const rclcpp::Time now = this->now();
    g_latest_imu_receive_time = now;
    g_latest_imu_time_valid = true;

    if (current_state_ != NavigationState::ALIGNING) return;

    if (g_align_gyro_calibrating)
    {
        g_align_gyro_bias_sum += msg->angular_velocity.z;
        g_align_gyro_bias_samples++;
        g_align_last_imu_time = now;

        if (g_align_gyro_bias_samples >= kGyroBiasSampleCount)
        {
            g_align_gyro_bias = g_align_gyro_bias_sum /
                static_cast<double>(g_align_gyro_bias_samples);
            g_align_gyro_angle = 0.0;
            g_align_start_time = now;
            g_align_last_imu_time = now;
            g_align_gyro_calibrating = false;
            g_align_gyro_active = true;
            RCLCPP_INFO(this->get_logger(),
                "ALIGNING (gyro): Bias calibration complete. bias=%.6f rad/s.",
                g_align_gyro_bias);
        }
        return;
    }

    if (!g_align_gyro_active) return;

    const double dt = (now - g_align_last_imu_time).seconds();
    g_align_last_imu_time = now;
    if (dt <= 0.0 || dt > 0.20) return;

    double corrected = msg->angular_velocity.z - g_align_gyro_bias;
    if (std::fabs(corrected) < kGyroDeadband) corrected = 0.0;
    g_align_gyro_angle += std::fabs(corrected) * dt;
}

void MechelangeloBehaviour::humanDetectedCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
    {
        // /human_tracking owns the continuous lock state. A false edge must
        // not erase grace because it may represent a brief Pi 4/network drop.
        return;
    }

    const rclcpp::Time now = this->now();

    if (humanDetectionCooldownActive(now))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Human detection ignored during cooldown. Remaining %.1f s.",
            humanDetectionCooldownRemaining(now));
        return;
    }

    g_human_detection_cooldown_active = false;

    // The edge trigger can arrive one control tick before /human_tracking.
    // Seed grace now so the state holds safely rather than instantly returning
    // to SEARCHING with no valid last-seen timestamp.
    markHumanSeen(now);

    RCLCPP_WARN(this->get_logger(),
        "Manual human detection trigger received. Entering human mode with %.1f s lock-loss grace.",
        kHumanLostTimeout);

    current_state_ = NavigationState::HUMAN_DETECTED;
}

void MechelangeloBehaviour::humanTrackingCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 3)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Invalid /human_tracking message. Expected [detected, centre_offset, distance_m].");
        return;
    }

    const rclcpp::Time now = this->now();
    const bool detected = msg->data[0] > 0.5F;

    if (detected && humanDetectionCooldownActive(now))
    {
        human_locked_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Camera human detection ignored during cooldown. Remaining %.1f s.",
            humanDetectionCooldownRemaining(now));
        return;
    }

    if (g_human_detection_cooldown_active &&
        !humanDetectionCooldownActive(now))
    {
        g_human_detection_cooldown_active = false;
    }

    human_locked_ = detected;
    human_centre_offset_ = static_cast<double>(msg->data[1]);
    // data[2] is preserved for compatibility; real range comes from LiDAR.
    human_distance_m_ = static_cast<double>(msg->data[2]);
    last_human_tracking_time_ = now;

    if (detected)
    {
        // Refresh the grace timestamp in the subscription callback itself,
        // closing the race with the next 10 Hz control-loop tick.
        markHumanSeen(now);

        if (current_state_ != NavigationState::HUMAN_DETECTED)
            resetHumanApproach();

        current_state_ = NavigationState::HUMAN_DETECTED;
    }
    // detected=false intentionally does not clear g_last_human_seen_valid.
    // HUMAN_DETECTED stops and waits for kHumanLostTimeout before abandoning.
}

// ============================================================
//  LaserScan filtering (neighbour test + segment extraction)
// ============================================================

sensor_msgs::msg::LaserScan MechelangeloBehaviour::filterLaserScan(
    const sensor_msgs::msg::LaserScan &raw_scan)
{
    sensor_msgs::msg::LaserScan neighbour_filtered = raw_scan;
    sensor_msgs::msg::LaserScan final_filtered = raw_scan;

    if (raw_scan.ranges.empty() || raw_scan.angle_increment == 0.0)
        return final_filtered;

    const int n = static_cast<int>(raw_scan.ranges.size());
    std::vector<double> x(n, std::numeric_limits<double>::quiet_NaN());
    std::vector<double> y(n, std::numeric_limits<double>::quiet_NaN());
    std::vector<bool> usable(n, false);

    for (int i = 0; i < n; ++i)
    {
        const double range = raw_scan.ranges[i];
        if (!isRangeUsableForFiltering(raw_scan, range))
        {
            neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
            final_filtered.ranges[i] = std::numeric_limits<float>::infinity();
            continue;
        }
        const double angle = raw_scan.angle_min +
            static_cast<double>(i) * raw_scan.angle_increment;
        x[i] = range * std::cos(angle);
        y[i] = range * std::sin(angle);
        usable[i] = true;
    }

    // Stage 1: drop isolated points with too few near neighbours.
    for (int i = 0; i < n; ++i)
    {
        if (!usable[i])
        {
            neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
            continue;
        }
        int close = 0;
        const int s = std::max(0, i - kNoiseNeighbourWindow);
        const int e = std::min(n - 1, i + kNoiseNeighbourWindow);
        for (int j = s; j <= e; ++j)
        {
            if (j == i || !usable[j]) continue;
            if (std::hypot(x[i] - x[j], y[i] - y[j]) <= kNoiseNeighbourDistance)
                close++;
        }
        if (close < kNoiseMinNeighbourCount)
            neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
    }

    // Stage 2: keep only points belonging to a real segment.
    const std::vector<LaserSegment> segments = buildLaserSegments(neighbour_filtered);
    std::fill(final_filtered.ranges.begin(), final_filtered.ranges.end(),
              std::numeric_limits<float>::infinity());
    for (const LaserSegment &seg : segments)
        for (int i = seg.start_index; i <= seg.end_index; ++i)
            if (i >= 0 && i < n && std::isfinite(neighbour_filtered.ranges[i]))
                final_filtered.ranges[i] = neighbour_filtered.ranges[i];

    return final_filtered;
}

std::vector<LaserSegment> MechelangeloBehaviour::buildLaserSegments(
    const sensor_msgs::msg::LaserScan &scan) const
{
    std::vector<LaserSegment> segments;
    if (scan.ranges.empty() || scan.angle_increment == 0.0) return segments;

    const int n = static_cast<int>(scan.ranges.size());
    bool active = false;
    LaserSegment cur;
    geometry_msgs::msg::Point prev;

    auto finish = [&]()
    {
        if (!active) return;
        cur.midpoint.x = 0.5 * (cur.start_point.x + cur.end_point.x);
        cur.midpoint.y = 0.5 * (cur.start_point.y + cur.end_point.y);
        cur.midpoint.z = 0.0;
        cur.length = std::hypot(cur.end_point.x - cur.start_point.x,
                                cur.end_point.y - cur.start_point.y);
        cur.midpoint_angle = std::atan2(cur.midpoint.y, cur.midpoint.x);
        if (cur.point_count >= kSegmentMinPoints && cur.length >= kSegmentMinLength)
            segments.push_back(cur);
        active = false;
        cur = LaserSegment();
    };

    for (int i = 0; i < n; ++i)
    {
        const double range = scan.ranges[i];
        if (!isRangeUsableForFiltering(scan, range))
        {
            finish();
            continue;
        }

        const geometry_msgs::msg::Point p = polarToPoint(scan, i);
        if (!active)
        {
            active = true;
            cur = LaserSegment();
            cur.start_index = i;
            cur.end_index = i;
            cur.point_count = 1;
            cur.start_point = p;
            cur.end_point = p;
            cur.min_range = range;
            prev = p;
            continue;
        }

        if (std::hypot(p.x - prev.x, p.y - prev.y) <= kSegmentJoinDistance)
        {
            cur.end_index = i;
            cur.end_point = p;
            cur.point_count++;
            cur.min_range = std::min(cur.min_range, range);
            prev = p;
        }
        else
        {
            finish();
            active = true;
            cur = LaserSegment();
            cur.start_index = i;
            cur.end_index = i;
            cur.point_count = 1;
            cur.start_point = p;
            cur.end_point = p;
            cur.min_range = range;
            prev = p;
        }
    }
    finish();
    return segments;
}

geometry_msgs::msg::Point MechelangeloBehaviour::polarToPoint(
    const sensor_msgs::msg::LaserScan &scan, int index) const
{
    geometry_msgs::msg::Point point;
    if (index < 0 || index >= static_cast<int>(scan.ranges.size())) return point;
    const double angle = scan.angle_min +
        static_cast<double>(index) * scan.angle_increment;
    const double range = scan.ranges[index];
    point.x = range * std::cos(angle);
    point.y = range * std::sin(angle);
    point.z = 0.0;
    return point;
}

bool MechelangeloBehaviour::isRangeUsableForFiltering(
    const sensor_msgs::msg::LaserScan &scan, double range) const
{
    if (!std::isfinite(range)) return false;
    if (range <= g_self_mask_range) return false;
    if (scan.range_min > 0.0 && range < scan.range_min) return false;
    if (scan.range_max > 0.0 && range > scan.range_max) return false;
    return true;
}

bool MechelangeloBehaviour::isRangeValid(double range) const
{
    return std::isfinite(range) && range > g_self_mask_range;
}

// ============================================================
//  Range queries
// ============================================================

double MechelangeloBehaviour::getMinimumRange(double start_angle, double end_angle) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
        return std::numeric_limits<double>::infinity();

    double min_range = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
    {
        const double angle = latest_scan_.angle_min +
            static_cast<double>(i) * latest_scan_.angle_increment;
        if (!angleInsideWindow(angle, start_angle, end_angle)) continue;
        const double range = latest_scan_.ranges[i];
        if (isRangeValid(range) && range < min_range) min_range = range;
    }
    return min_range;
}

double MechelangeloBehaviour::getFrontRange() const
{
    return getMinimumRange(-kFrontCheckAngle, kFrontCheckAngle);
}

bool MechelangeloBehaviour::getLongestRange(double &out_angle, double &out_range) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
        return false;

    const int scan_count = static_cast<int>(latest_scan_.ranges.size());
    const double angle_step = std::fabs(latest_scan_.angle_increment);
    if (scan_count <= 0 || angle_step <= 0.0) return false;

    std::vector<bool> open(scan_count, false);
    for (int i = 0; i < scan_count; ++i)
    {
        const double range = latest_scan_.ranges[i];
        if (std::isinf(range)) open[i] = true;
        else if (std::isfinite(range) && range >= kDvdOpenClearanceDistance)
            open[i] = true;
    }

    auto angleForIndex = [&](int index)
    {
        return normaliseAngle(latest_scan_.angle_min +
            static_cast<double>(index) * latest_scan_.angle_increment);
    };
    auto absAngleForIndex = [&](int index)
    {
        return std::fabs(angleForIndex(index));
    };
    auto rangeForLog = [&](int index)
    {
        const double r = latest_scan_.ranges[index];
        if (std::isinf(r)) return std::numeric_limits<double>::infinity();
        if (std::isfinite(r)) return r;
        return 0.0;
    };
    auto isPreferred = [&](int index)
    {
        const double a = absAngleForIndex(index);
        return a >= kDvdPreferredMinTurnAngle &&
               a <= kDvdPreferredMaxTurnAngle;
    };
    auto isFallback = [&](int index)
    {
        const double a = absAngleForIndex(index);
        return a >= kDvdAvoidFrontAngle && a <= kDvdAvoidReverseAngle;
    };

    std::vector<int> pref_left;
    std::vector<int> pref_right;
    std::vector<int> fb_left;
    std::vector<int> fb_right;
    std::vector<int> fb_all;

    auto collect = [&](int start_index, int count)
    {
        if (count <= 0) return;
        if (static_cast<double>(count) * angle_step < kDvdMinSectorWidth)
            return;
        int margin = static_cast<int>(std::ceil(kDvdSectorEdgeMargin / angle_step));
        margin = std::min(margin, std::max(0, (count - 1) / 2));
        for (int o = margin; o < count - margin; ++o)
        {
            const int index = (start_index + o) % scan_count;
            const double a = angleForIndex(index);
            if (isPreferred(index))
                (a >= 0.0 ? pref_left : pref_right).push_back(index);
            if (isFallback(index))
            {
                fb_all.push_back(index);
                (a >= 0.0 ? fb_left : fb_right).push_back(index);
            }
        }
    };

    const bool all_open = std::all_of(
        open.begin(), open.end(), [](bool v) { return v; });
    if (all_open)
    {
        collect(0, scan_count);
    }
    else
    {
        for (int s = 0; s < scan_count; ++s)
        {
            if (!open[s]) continue;
            if (open[(s - 1 + scan_count) % scan_count]) continue;
            int count = 0;
            while (count < scan_count && open[(s + count) % scan_count])
                count++;
            collect(s, count);
        }
    }

    static thread_local std::mt19937 rng(std::random_device{}());
    auto pick = [&](const std::vector<int> &c)
    {
        std::uniform_int_distribution<int> d(0, static_cast<int>(c.size()) - 1);
        return c[d(rng)];
    };
    auto pickBalanced = [&](const std::vector<int> &l, const std::vector<int> &r)
    {
        if (!l.empty() && !r.empty())
        {
            std::uniform_int_distribution<int> sd(0, 1);
            return sd(rng) == 0 ? pick(l) : pick(r);
        }
        return !l.empty() ? pick(l) : pick(r);
    };

    int selected = -1;
    if (!pref_left.empty() || !pref_right.empty())
        selected = pickBalanced(pref_left, pref_right);
    else if (!fb_left.empty() || !fb_right.empty())
        selected = pickBalanced(fb_left, fb_right);
    else if (!fb_all.empty())
        selected = pick(fb_all);

    if (selected >= 0)
    {
        out_angle = angleForIndex(selected);
        out_range = rangeForLog(selected);
        return true;
    }

    // Fallback: longest finite return, steering toward the middle of the tie band.
    double max_range = 0.0;
    int max_index = -1;
    for (int i = 0; i < scan_count; ++i)
    {
        const double r = latest_scan_.ranges[i];
        if (isRangeValid(r) && r > max_range)
        {
            max_range = r;
            max_index = i;
        }
    }
    if (max_index < 0) return false;

    const double tie = std::max(
        g_self_mask_range,
        max_range - kLongestRangeTieTolerance);
    std::vector<bool> near(scan_count, false);
    for (int i = 0; i < scan_count; ++i)
        near[i] = isRangeValid(latest_scan_.ranges[i]) &&
                  latest_scan_.ranges[i] >= tie;

    int best_start = max_index;
    int best_count = 0;
    for (int s = 0; s < scan_count; ++s)
    {
        if (!near[s]) continue;
        if (near[(s - 1 + scan_count) % scan_count]) continue;
        int count = 0;
        while (count < scan_count && near[(s + count) % scan_count])
            count++;
        if (count > best_count)
        {
            best_count = count;
            best_start = s;
        }
    }
    if (best_count == 0)
    {
        best_count = 1;
        best_start = max_index;
    }

    const double mid = std::fmod(
        static_cast<double>(best_start) +
            0.5 * static_cast<double>(std::max(0, best_count - 1)),
        static_cast<double>(scan_count));
    out_angle = normaliseAngle(
        latest_scan_.angle_min + mid * latest_scan_.angle_increment);
    out_range = max_range;
    return true;
}

int MechelangeloBehaviour::angleToIndex(double angle_rad) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
        return -1;
    double a = std::clamp(
        angle_rad,
        static_cast<double>(latest_scan_.angle_min),
        static_cast<double>(latest_scan_.angle_max));
    int index = static_cast<int>(
        std::round((a - latest_scan_.angle_min) / latest_scan_.angle_increment));
    return std::clamp(
        index, 0, static_cast<int>(latest_scan_.ranges.size()) - 1);
}

double MechelangeloBehaviour::normaliseAngle(double angle_rad) const
{
    while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
    return angle_rad;
}

bool MechelangeloBehaviour::angleInsideWindow(
    double angle_rad, double start_angle, double end_angle) const
{
    const double a = normaliseAngle(angle_rad);
    const double s = normaliseAngle(start_angle);
    const double e = normaliseAngle(end_angle);
    if (s <= e) return a >= s && a <= e;
    return a >= s || a <= e;
}

bool MechelangeloBehaviour::segmentOverlapsAngleWindow(
    const LaserSegment &segment, double start_angle, double end_angle) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
        return false;
    for (int i = segment.start_index; i <= segment.end_index; ++i)
    {
        if (i < 0 || i >= static_cast<int>(latest_scan_.ranges.size()))
            continue;
        const double angle = latest_scan_.angle_min +
            static_cast<double>(i) * latest_scan_.angle_increment;
        if (angleInsideWindow(angle, start_angle, end_angle)) return true;
    }
    return false;
}

bool MechelangeloBehaviour::findBlockingObstaclesInFront(
    std::vector<LaserSegment> &blocking_segments) const
{
    blocking_segments.clear();
    for (const LaserSegment &seg : latest_segments_)
        if (seg.min_range <= stop_distance_m_ &&
            segmentOverlapsAngleWindow(seg, -kFrontCheckAngle, kFrontCheckAngle))
            blocking_segments.push_back(seg);
    return !blocking_segments.empty();
}

// ============================================================
//  Markers
// ============================================================

void MechelangeloBehaviour::publishObstacleMarkers(
    const std::vector<LaserSegment> &blocking_segments)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = latest_scan_.header.frame_id.empty()
        ? "base_link" : latest_scan_.header.frame_id;
    clear_marker.header.stamp = this->get_clock()->now();
    clear_marker.ns = "behaviour_blocking_obstacles";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 1;
    for (const LaserSegment &seg : blocking_segments)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = latest_scan_.header.frame_id.empty()
            ? "base_link" : latest_scan_.header.frame_id;
        m.header.stamp = this->get_clock()->now();
        m.ns = "behaviour_blocking_obstacles";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = seg.midpoint.x;
        m.pose.position.y = seg.midpoint.y;
        m.pose.position.z = 0.15;
        m.pose.orientation.w = 1.0;
        const double w = std::clamp(seg.length + 0.15, 0.20, 0.80);
        m.scale.x = w;
        m.scale.y = w;
        m.scale.z = 0.30;
        m.color.a = 0.75F;
        m.color.r = 1.0F;
        m.color.g = 0.15F;
        m.color.b = 0.0F;
        m.lifetime.sec = 0;
        m.lifetime.nanosec = 400000000;
        marker_array.markers.push_back(m);
    }
    obstacle_marker_publisher_->publish(marker_array);
}

void MechelangeloBehaviour::clearObstacleMarkers()
{
    if (!obstacle_marker_publisher_) return;
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = latest_scan_.header.frame_id.empty()
        ? "base_link" : latest_scan_.header.frame_id;
    clear_marker.header.stamp = this->get_clock()->now();
    clear_marker.ns = "behaviour_blocking_obstacles";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    obstacle_marker_publisher_->publish(marker_array);
}

void MechelangeloBehaviour::longestLaserScan()
{
    double a = 0.0;
    double r = 0.0;
    if (!getLongestRange(a, r))
    {
        RCLCPP_WARN(this->get_logger(),
            "No valid filtered laser scan for longest scan.");
        return;
    }
    RCLCPP_INFO(this->get_logger(),
        "Selected heading: range = %.2f m at %.2f deg",
        r, a * 180.0 / M_PI);
}

double MechelangeloBehaviour::getHumanLidarRange(double centre_offset) const
{
    const double bearing = -centre_offset * kCameraHorizontalFov;
    return getMinimumRange(
        bearing - kHumanRangeConeHalfAngle,
        bearing + kHumanRangeConeHalfAngle);
}

void MechelangeloBehaviour::captureSafetyZoneBaseline()
{
    safety_zone_baseline_scan_ = latest_scan_;
    safety_zone_baseline_captured_ = true;
}

// Forward emergency check used during human approach. It only returns true if
// a coherent cluster sits inside the forward corridor, closer than the stop
// range, and outside the person's exclusion cone. Side returns and single dots
// are ignored.
bool MechelangeloBehaviour::isSafetyZoneViolated(double human_bearing_rad) const
{
    g_safety_obstruction_bearing =
        std::numeric_limits<double>::quiet_NaN();
    g_safety_obstruction_range =
        std::numeric_limits<double>::infinity();

    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
        return false;

    int coherent = 0;
    int prev_index = -1000;
    double prev_range = std::numeric_limits<double>::infinity();
    double nearest_range = std::numeric_limits<double>::infinity();
    double nearest_bearing = std::numeric_limits<double>::quiet_NaN();

    auto resetRun = [&]()
    {
        coherent = 0;
        prev_index = -1000;
        prev_range = std::numeric_limits<double>::infinity();
        nearest_range = std::numeric_limits<double>::infinity();
        nearest_bearing = std::numeric_limits<double>::quiet_NaN();
    };

    for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
    {
        const double angle = latest_scan_.angle_min +
            static_cast<double>(i) * latest_scan_.angle_increment;

        // Forward corridor only.
        if (std::fabs(normaliseAngle(angle)) > kHumanForwardEmergencyHalfAngle)
        {
            resetRun();
            continue;
        }

        // Exclude the person.
        if (std::fabs(normaliseAngle(angle - human_bearing_rad)) <=
            kHumanPersonExclusionAngle)
        {
            resetRun();
            continue;
        }

        const double range = latest_scan_.ranges[i];
        if (!std::isfinite(range) || range <= g_self_mask_range ||
            range >= kHumanEmergencyStopRange)
        {
            resetRun();
            continue;
        }

        const bool adjacent = (i == prev_index + 1) &&
            std::isfinite(prev_range) &&
            std::fabs(range - prev_range) <= kHumanEmergencyClusterRangeGap;

        if (adjacent)
        {
            coherent++;
        }
        else
        {
            coherent = 1;
            nearest_range = range;
            nearest_bearing = angle;
        }

        if (range < nearest_range)
        {
            nearest_range = range;
            nearest_bearing = angle;
        }
        prev_index = i;
        prev_range = range;

        if (coherent >= kHumanEmergencyClusterMinPoints)
        {
            g_safety_obstruction_range = nearest_range;
            g_safety_obstruction_bearing = nearest_bearing;
            return true;
        }
    }
    return false;
}

// ============================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MechelangeloBehaviour>();
    // true = simulation, false = real robot
    node->run(false);
    rclcpp::shutdown();
    return 0;
}
// /////////////////////////////////////////////////////////////////////////
// ///DVD style movemenmt

// #include "behaviour.hpp"

// #include <algorithm>
// #include <chrono>
// #include <cmath>
// #include <fstream>
// #include <iomanip>
// #include <limits>
// #include <functional>
// #include <memory>
// #include <random>
// #include <string>
// #include <vector>

// using namespace std::chrono_literals;

// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif

// // ------------------------------------------------------
// // Behaviour constants
// // ------------------------------------------------------

// // Control loop runs every 100 ms.
// static constexpr double kControlPeriodSeconds = 0.1;

// // Movement tuning.
// static constexpr double kForwardSpeed = 0.26;       // m/s
// static constexpr double kTurnSpeed = 0.6;           // rad/s
// static constexpr double kAngleGain = 0.8;           // proportional turning gain
// static constexpr double kAlignmentTolerance = 0.10; // radians, about 5.7 degrees

// // Smooth commanded stops so the physical base does not snap from motion to zero
// // in one control tick. At the 100 ms control period, these remove roughly
// // 0.04 m/s and 0.12 rad/s from the command each loop.
// static constexpr double kStopLinearDecel = 0.4;  // m/s^2
// static constexpr double kStopAngularDecel = 1.2; // rad/s^2

// // Stop this far before a real obstacle/wall.
// // Loaded from the ROS parameter 'stop_distance_m'.
// // Default: 1.5 m (simulation). Physical robot: set to 0.75 in the launch file.

// // 30 loops x 0.1 s = 3 seconds.
// static constexpr int kStopDurationLoops = 30;

// // Ignore returns too close to the robot body / lidar blind spot.
// static constexpr double kMinValidRange = 0.5; // m

// // Front scan window used while moving forward.
// static constexpr double kFrontCheckAngle = 30 * M_PI / 180.0; // +/- 30 degrees

// // When several neighbouring beams are effectively tied for the longest
// // distance, steer toward the middle of that opening instead of whichever beam
// // happens to appear first in the scan array.
// static constexpr double kLongestRangeTieTolerance = 0.05; // m

// // ------------------------------------------------------
// // DVD-style bounce exploration tuning
// // ------------------------------------------------------
// // The autonomous gallery behaviour is intended to move like a DVD screensaver:
// // drive straight until the front is blocked, stop at a safe distance, then pick
// // a new safe side-bounce angle instead of always chasing the longest wall.
// //
// // A beam is considered open if it is infinity OR farther than this clearance.
// // Infinity is useful in large rooms because it means the LiDAR did not hit
// // anything within range.
// static constexpr double kDvdOpenClearanceDistance = 4.5; // m

// // Ignore very narrow open gaps. This prevents the robot from aiming through
// // thin laser cracks between obstacles.
// static constexpr double kDvdMinSectorWidth = 18.0 * M_PI / 180.0; // radians

// // Trim candidate angles away from the edge of an open sector.
// static constexpr double kDvdSectorEdgeMargin = 8.0 * M_PI / 180.0; // radians

// // Preferred bounce band after the robot stops at a wall.
// // These angles are relative to the robot's current forward direction:
// //   0 deg   = back into the wall/obstacle it just stopped for
// //   +/-90   = side-bounce
// //   +/-180  = drive exactly back along the previous path
// // The robot randomly chooses inside this safe side band when possible.
// static constexpr double kDvdPreferredMinTurnAngle = 55.0 * M_PI / 180.0;  // radians
// static constexpr double kDvdPreferredMaxTurnAngle = 150.0 * M_PI / 180.0; // radians

// // Fallback band used if the preferred side-bounce band has no safe candidates.
// // This still avoids the front wall and avoids exact reverse.
// static constexpr double kDvdAvoidFrontAngle = 35.0 * M_PI / 180.0;  // radians
// static constexpr double kDvdAvoidReverseAngle = 165.0 * M_PI / 180.0; // radians

// // ------------------------------------------------------
// // LaserScan noise suppression constants
// // ------------------------------------------------------
// // The filter is based on the previous LaserProcessing::countSegments()
// // approach: valid objects form segments of neighbouring points. Random
// // dots are usually isolated or only one/two points, so they get replaced
// // with infinity and will not stop the robot.

// // Stage 1: local neighbour test.
// static constexpr int kNoiseNeighbourWindow = 4;          // check +/- 4 beams
// static constexpr int kNoiseMinNeighbourCount = 2;        // require at least 2 close neighbours
// static constexpr double kNoiseNeighbourDistance = 0.22;  // m in local XY space

// // Stage 2: segment extraction test.
// static constexpr double kSegmentJoinDistance = 0.18;     // m max gap between consecutive points
// static constexpr int kSegmentMinPoints = 4;              // reject tiny speckle clusters
// static constexpr double kSegmentMinLength = 0.05;        // m reject near-zero length segments

// // ------------------------------------------------------
// // Human tracking tuning
// // ------------------------------------------------------
// // /human_tracking message format:
// // data[0] = detected, data[1] = centre_offset, data[2] = distance_m
// // centre_offset is normalised image offset from centre: -0.5 left, 0 centre, +0.5 right.
// static constexpr double kHumanTargetDistance = 1.5;     // m
// static constexpr double kHumanDistanceTolerance = 0.15; // m
// static constexpr double kHumanMaxForwardSpeed = 0.22;   // m/s
// static constexpr double kHumanMaxReverseSpeed = 0.12;   // m/s
// static constexpr double kHumanMaxTurnSpeed = 0.7;       // rad/s
// static constexpr double kHumanTurnGain = 1.8;           // image offset to angular speed
// static constexpr double kHumanForwardGain = 0.35;       // distance error to linear speed
// static constexpr double kHumanCentreDeadZone = 0.06;    // normalised image width
// static constexpr double kHumanLostTimeout = 1.0;        // seconds

// // LiDAR validation for human distance.
// static constexpr double kCameraHorizontalFov = 60.0 * M_PI / 180.0;
// static constexpr double kHumanLidarWindow = 10.0 * M_PI / 180.0;
// static constexpr double kLidarCameraMaxDisagreement = 0.4;
// static constexpr double kHumanLidarStopDistance = 1.5;
// static constexpr double kHumanLidarStopTolerance = 0.15;

// // Safety zone.
// static constexpr double kSafetyZoneRadius = 1.5;
// static constexpr double kSafetyZoneIntruderThreshold = 0.3;
// static constexpr double kSafetyZoneHumanExclusionAngle = 25.0 * M_PI / 180.0;

// MechelangeloBehaviour::MechelangeloBehaviour()
// : Node("mechelangelo_behaviour"),
//   human_locked_(false),
//   human_centre_offset_(0.0),
//   human_distance_m_(-1.0),
//   blind_autonomous_active_(false),
//   safety_zone_violated_(false),
//   safety_zone_baseline_captured_(false),
//   current_state_(NavigationState::SEARCHING),
//   target_angle_(0.0),
//   target_range_(0.0),
//   stop_distance_m_(1.5),
//   stop_counter_(0),
//   imu_available_(false),
//   align_start_yaw_(0.0),
//   align_yaw_initialised_(false),
//   random_engine_(std::random_device{}()),
//   turn_dist_(-1.0, 1.0)
// {
//     this->declare_parameter("stop_distance_m", 1.5);
//     stop_distance_m_ = this->get_parameter("stop_distance_m").as_double();

//     RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_m_);

//     laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         rclcpp::SensorDataQoS(),
//         std::bind(&MechelangeloBehaviour::laserScanCallback, this, std::placeholders::_1));

//     imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
//         "/imu",
//         rclcpp::SensorDataQoS(),
//         std::bind(&MechelangeloBehaviour::imuCallback, this, std::placeholders::_1));

//     cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
//         "/cmd_vel",
//         10);

//     filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
//         "/scan_filtered",
//         rclcpp::SensorDataQoS());

//     obstacle_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//         "/behaviour_obstacle_markers",
//         10);

//     human_detected_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
//         "/human_detected",
//         10,
//         std::bind(&MechelangeloBehaviour::humanDetectedCallback, this, std::placeholders::_1));

//     human_tracking_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
//         "/human_tracking",
//         10,
//         std::bind(&MechelangeloBehaviour::humanTrackingCallback, this, std::placeholders::_1));

//     last_human_tracking_time_ = this->now();

//     control_timer_ = this->create_wall_timer(
//         100ms,
//         std::bind(&MechelangeloBehaviour::controlLoop, this));

//     RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been started.");
// }

// MechelangeloBehaviour::~MechelangeloBehaviour()
// {
//     RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been stopped.");
// }

// void MechelangeloBehaviour::run(bool sim_mode)
// {
//     RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node is running.");

//     if (sim_mode)
//     {
//         RCLCPP_INFO(this->get_logger(), "Running in simulation mode.");
//     }
//     else
//     {
//         RCLCPP_INFO(this->get_logger(), "Running in real robot mode.");
//     }

//     blindAutonomous();
//     rclcpp::spin(shared_from_this());
// }

// void MechelangeloBehaviour::blindAutonomous()
// {
//     RCLCPP_INFO(this->get_logger(), "Executing blind autonomous behaviour.");

//     blind_autonomous_active_ = true;
//     safety_zone_violated_ = false;
//     safety_zone_baseline_captured_ = false;
//     current_state_ = NavigationState::SEARCHING;
//     target_angle_ = 0.0;
//     target_range_ = 0.0;
//     stop_counter_ = 0;
//     align_yaw_initialised_ = false;
//     clearObstacleMarkers();
// }

// void MechelangeloBehaviour::mappedAutonomous()
// {
//     RCLCPP_INFO(this->get_logger(), "Executing mapped autonomous behaviour.");
// }

// void MechelangeloBehaviour::controlLoop()
// {
//     if (!blind_autonomous_active_)
//     {
//         return;
//     }

//     geometry_msgs::msg::Twist twist;

//     // Safety: wait until valid LaserScan data exists.
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         RCLCPP_WARN_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             2000,
//             "Waiting for valid filtered LaserScan data...");

//         stopRobot(twist);
//         current_twist_ = twist;
//         cmd_vel_publisher_->publish(twist);
//         return;
//     }

//     switch (current_state_)
//     {
//     case NavigationState::SEARCHING:
//     {
//         stopRobot(twist);
//         clearObstacleMarkers();

//         double longest_angle = 0.0;
//         double longest_range = 0.0;

//         if (!getLongestRange(longest_angle, longest_range))
//         {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 2000,
//                 "SEARCHING: No valid filtered LaserScan range found.");
//             break;
//         }

//         target_angle_ = longest_angle;
//         target_range_ = longest_range;
//         align_yaw_initialised_ = false;

//         RCLCPP_INFO(
//             this->get_logger(),
//             "SEARCHING: Selected exploration heading at %.2f deg, representative range %.2f m",
//             target_angle_ * 180.0 / M_PI,
//             target_range_);

//         current_state_ = NavigationState::ALIGNING;
//         break;
//     }

//     case NavigationState::ALIGNING:
//     {
//         clearObstacleMarkers();
//         twist.linear.x = 0.0;

//         if (!imu_available_)
//         {
//             // IMU not yet publishing — fall back to open-loop time integration.
//             if (std::fabs(target_angle_) <= kAlignmentTolerance)
//             {
//                 stopRobot(twist);
//                 if (std::fabs(twist.angular.z) <= 1e-6)
//                 {
//                     RCLCPP_INFO(this->get_logger(),
//                         "ALIGNING (open-loop): Aligned. Starting forward movement.");
//                     current_state_ = NavigationState::MOVING;
//                 }
//                 break;
//             }

//             const double turn_cmd = std::clamp(
//                 target_angle_ * kAngleGain, -kTurnSpeed, kTurnSpeed);
//             twist.angular.z = turn_cmd;
//             target_angle_ -= turn_cmd * kControlPeriodSeconds;
//             target_angle_ = normaliseAngle(target_angle_);

//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//                 "ALIGNING: No IMU data yet — using open-loop time estimate. "
//                 "Remaining %.2f deg", target_angle_ * 180.0 / M_PI);
//             break;
//         }

//         // IMU-confirmed alignment: compare actual yaw turned vs. required angle.
//         tf2::Quaternion q;
//         tf2::fromMsg(latest_imu_.orientation, q);
//         const double current_yaw = tf2::getYaw(q);

//         if (!align_yaw_initialised_)
//         {
//             align_start_yaw_ = current_yaw;
//             align_yaw_initialised_ = true;
//         }

//         // How much has the robot actually rotated since ALIGNING began.
//         const double yaw_turned = normaliseAngle(current_yaw - align_start_yaw_);

//         // How many degrees still remain.
//         const double remaining_angle = normaliseAngle(target_angle_ - yaw_turned);

//         if (std::fabs(remaining_angle) <= kAlignmentTolerance)
//         {
//             stopRobot(twist);
//             if (std::fabs(twist.angular.z) <= 1e-6)
//             {
//                 RCLCPP_INFO(this->get_logger(),
//                     "ALIGNING: IMU confirmed rotation. Turned %.2f deg (target %.2f deg). Starting forward movement.",
//                     yaw_turned * 180.0 / M_PI,
//                     target_angle_ * 180.0 / M_PI);
//                 current_state_ = NavigationState::MOVING;
//             }
//             break;
//         }

//         const double turn_cmd = std::clamp(
//             remaining_angle * kAngleGain, -kTurnSpeed, kTurnSpeed);
//         twist.angular.z = turn_cmd;

//         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//             "ALIGNING: IMU-confirmed. Target %.2f deg, turned %.2f deg, remaining %.2f deg, cmd %.2f rad/s",
//             target_angle_ * 180.0 / M_PI,
//             yaw_turned * 180.0 / M_PI,
//             remaining_angle * 180.0 / M_PI,
//             twist.angular.z);

//         break;
//     }

//     case NavigationState::MOVING:
//     {
//         std::vector<LaserSegment> blocking_segments;
//         const bool blocked_by_segment = findBlockingObstaclesInFront(blocking_segments);
//         const double front_range = getFrontRange();

//         // Segment-based blocking is the main decision. This prevents a single
//         // random dot from stopping the robot because the dot will not survive
//         // the neighbour + segment filter.
//         if (blocked_by_segment || front_range <= stop_distance_m_)
//         {
//             if (blocking_segments.empty())
//             {
//                 // Fallback marker if the range check caught something but no
//                 // segment was available. This should be rare after filtering.
//                 LaserSegment fallback;
//                 fallback.point_count = 1;
//                 fallback.min_range = front_range;
//                 fallback.midpoint.x = std::isfinite(front_range) ? front_range : stop_distance_m_;
//                 fallback.midpoint.y = 0.0;
//                 fallback.midpoint.z = 0.0;
//                 blocking_segments.push_back(fallback);
//             }

//             publishObstacleMarkers(blocking_segments);

//             RCLCPP_WARN(
//                 this->get_logger(),
//                 "MOVING: Blocking obstacle detected in front. Front range = %.2f m. Stopping.",
//                 front_range);

//             stopRobot(twist);
//             stop_counter_ = 0;
//             current_state_ = NavigationState::STOPPED;
//             break;
//         }

//         clearObstacleMarkers();

//         if (std::isinf(front_range))
//         {
//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "MOVING: Front is clear after filtering. Driving forward.");
//         }
//         else
//         {
//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "MOVING: Driving forward. Filtered front range = %.2f m",
//                 front_range);
//         }

//         twist.linear.x = kForwardSpeed;
//         twist.angular.z = 0.0;
//         break;
//     }

//     case NavigationState::STOPPED:
//     {
//         stopRobot(twist);
//         stop_counter_++;

//         if (stop_counter_ >= kStopDurationLoops)
//         {
//             RCLCPP_INFO(
//                 this->get_logger(),
//                 "STOPPED: Pause complete. Searching for next direction.");

//             stop_counter_ = 0;
//             clearObstacleMarkers();
//             current_state_ = NavigationState::SEARCHING;
//         }
//         else
//         {
//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "STOPPED: Pausing %.1f / %.1f seconds",
//                 stop_counter_ * kControlPeriodSeconds,
//                 kStopDurationLoops * kControlPeriodSeconds);
//         }

//         break;
//     }

//     case NavigationState::HUMAN_DETECTED:
//     {
//         const double time_since_tracking =
//             (this->now() - last_human_tracking_time_).seconds();

//         if (!human_locked_ || time_since_tracking > kHumanLostTimeout)
//         {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: Human lost. Returning to blind autonomous search.");

//             stopRobot(twist);
//             human_locked_ = false;
//             safety_zone_violated_ = false;
//             safety_zone_baseline_captured_ = false;
//             clearObstacleMarkers();
//             current_state_ = NavigationState::SEARCHING;
//             break;
//         }

//         if (!safety_zone_baseline_captured_)
//         {
//             captureSafetyZoneBaseline();
//         }

//         const double human_bearing_rad = -human_centre_offset_ * kCameraHorizontalFov;
//         const bool zone_now_violated = isSafetyZoneViolated(human_bearing_rad);

//         if (zone_now_violated != safety_zone_violated_)
//         {
//             safety_zone_violated_ = zone_now_violated;

//             if (safety_zone_violated_)
//             {
//                 RCLCPP_WARN(
//                     this->get_logger(),
//                     "SAFETY ZONE: Object detected within %.1f m. Interaction paused.",
//                     kSafetyZoneRadius);
//             }
//             else
//             {
//                 RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Clear. Resuming interaction.");
//             }
//         }

//         if (safety_zone_violated_)
//         {
//             stopRobot(twist);
//             break;
//         }

//         if (std::fabs(human_centre_offset_) <= kHumanCentreDeadZone)
//         {
//             twist.angular.z = 0.0;
//         }
//         else
//         {
//             twist.angular.z = std::clamp(
//                 -kHumanTurnGain * human_centre_offset_,
//                 -kHumanMaxTurnSpeed,
//                 kHumanMaxTurnSpeed);
//         }

//         const double human_lidar_range = getHumanLidarRange(human_centre_offset_);
//         const bool lidar_distance_valid = std::isfinite(human_lidar_range);

//         if (!lidar_distance_valid)
//         {
//             twist.linear.x = 0.0;

//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: No valid filtered LiDAR return near human bearing. Turning only.");
//         }
//         else
//         {
//             if (human_distance_m_ > 0.0 && std::isfinite(human_distance_m_))
//             {
//                 const double disagreement = std::fabs(human_distance_m_ - human_lidar_range);

//                 if (disagreement > kLidarCameraMaxDisagreement)
//                 {
//                     RCLCPP_WARN_THROTTLE(
//                         this->get_logger(),
//                         *this->get_clock(),
//                         1000,
//                         "HUMAN_DETECTED: Camera/LiDAR distance disagreement. Camera=%.2f m, LiDAR=%.2f m",
//                         human_distance_m_,
//                         human_lidar_range);
//                 }
//             }

//             const double distance_error = human_lidar_range - kHumanLidarStopDistance;

//             if (std::fabs(distance_error) <= kHumanLidarStopTolerance)
//             {
//                 twist.linear.x = 0.0;

//                 RCLCPP_INFO_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     1000,
//                     "HUMAN_DETECTED: LiDAR validated target distance %.2f m. Ready for arm mimicry.",
//                     human_lidar_range);
//             }
//             else
//             {
//                 twist.linear.x = std::clamp(
//                     kHumanForwardGain * distance_error,
//                     -kHumanMaxReverseSpeed,
//                     kHumanMaxForwardSpeed);
//             }

//             if (human_lidar_range < kHumanLidarStopDistance && twist.linear.x > 0.0)
//             {
//                 twist.linear.x = 0.0;
//             }
//         }

//         RCLCPP_INFO_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             500,
//             "HUMAN_DETECTED: offset=%.2f distance=%.2f cmd linear=%.2f angular=%.2f",
//             human_centre_offset_,
//             human_distance_m_,
//             twist.linear.x,
//             twist.angular.z);

//         break;
//     }

//     default:
//     {
//         RCLCPP_WARN(this->get_logger(), "Unknown navigation state. Returning to SEARCHING.");
//         stopRobot(twist);
//         clearObstacleMarkers();
//         current_state_ = NavigationState::SEARCHING;
//         break;
//     }
//     }

//     current_twist_ = twist;
//     cmd_vel_publisher_->publish(twist);
// }

// void MechelangeloBehaviour::stopRobot(geometry_msgs::msg::Twist &twist)
// {
//     const double linear_step = kStopLinearDecel * kControlPeriodSeconds;
//     const double angular_step = kStopAngularDecel * kControlPeriodSeconds;

//     auto rampTowardZero = [](double value, double max_step)
//     {
//         if (std::fabs(value) <= max_step)
//         {
//             return 0.0;
//         }

//         return value - std::copysign(max_step, value);
//     };

//     twist.linear.x = rampTowardZero(current_twist_.linear.x, linear_step);
//     twist.linear.y = rampTowardZero(current_twist_.linear.y, linear_step);
//     twist.linear.z = rampTowardZero(current_twist_.linear.z, linear_step);

//     twist.angular.x = rampTowardZero(current_twist_.angular.x, angular_step);
//     twist.angular.y = rampTowardZero(current_twist_.angular.y, angular_step);
//     twist.angular.z = rampTowardZero(current_twist_.angular.z, angular_step);
// }

// void MechelangeloBehaviour::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//     const sensor_msgs::msg::LaserScan filtered_scan = filterLaserScan(*msg);

//     latest_scan_ = filtered_scan;
//     latest_segments_ = buildLaserSegments(filtered_scan);

//     filtered_scan_publisher_->publish(filtered_scan);
// }

// void MechelangeloBehaviour::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//     latest_imu_ = *msg;
//     imu_available_ = true;
// }

// void MechelangeloBehaviour::humanDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     if (!msg->data)
//     {
//         return;
//     }

//     RCLCPP_WARN(
//         this->get_logger(),
//         "Manual human detection trigger received. Interrupting autonomous behaviour.");

//     current_state_ = NavigationState::HUMAN_DETECTED;
// }

// void MechelangeloBehaviour::humanTrackingCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
// {
//     if (msg->data.size() < 3)
//     {
//         RCLCPP_WARN_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             2000,
//             "Received invalid /human_tracking message. Expected [detected, centre_offset, distance_m].");
//         return;
//     }

//     human_locked_ = msg->data[0] > 0.5F;
//     human_centre_offset_ = static_cast<double>(msg->data[1]);
//     human_distance_m_ = static_cast<double>(msg->data[2]);
//     last_human_tracking_time_ = this->now();

//     if (human_locked_)
//     {
//         current_state_ = NavigationState::HUMAN_DETECTED;
//     }
// }

// sensor_msgs::msg::LaserScan MechelangeloBehaviour::filterLaserScan(
//     const sensor_msgs::msg::LaserScan &raw_scan)
// {
//     sensor_msgs::msg::LaserScan neighbour_filtered = raw_scan;
//     sensor_msgs::msg::LaserScan final_filtered = raw_scan;

//     if (raw_scan.ranges.empty() || raw_scan.angle_increment == 0.0)
//     {
//         return final_filtered;
//     }

//     const int scan_count = static_cast<int>(raw_scan.ranges.size());
//     std::vector<double> x_points(scan_count, std::numeric_limits<double>::quiet_NaN());
//     std::vector<double> y_points(scan_count, std::numeric_limits<double>::quiet_NaN());
//     std::vector<bool> usable(scan_count, false);

//     // Convert valid polar points to local Cartesian points.
//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = raw_scan.ranges[i];

//         if (!isRangeUsableForFiltering(raw_scan, range))
//         {
//             neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             final_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             continue;
//         }

//         const double angle = raw_scan.angle_min + static_cast<double>(i) * raw_scan.angle_increment;
//         x_points[i] = range * std::cos(angle);
//         y_points[i] = range * std::sin(angle);
//         usable[i] = true;
//     }

//     // Stage 1: suppress isolated points that do not have nearby neighbours.
//     for (int i = 0; i < scan_count; ++i)
//     {
//         if (!usable[i])
//         {
//             neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             continue;
//         }

//         int close_neighbour_count = 0;
//         const int start_index = std::max(0, i - kNoiseNeighbourWindow);
//         const int end_index = std::min(scan_count - 1, i + kNoiseNeighbourWindow);

//         for (int j = start_index; j <= end_index; ++j)
//         {
//             if (j == i || !usable[j])
//             {
//                 continue;
//             }

//             const double distance = std::hypot(x_points[i] - x_points[j], y_points[i] - y_points[j]);

//             if (distance <= kNoiseNeighbourDistance)
//             {
//                 close_neighbour_count++;
//             }
//         }

//         if (close_neighbour_count < kNoiseMinNeighbourCount)
//         {
//             neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//         }
//     }

//     // Stage 2: build LaserProcessing-style segments and only keep points
//     // that belong to a real segment.
//     const std::vector<LaserSegment> accepted_segments = buildLaserSegments(neighbour_filtered);

//     std::fill(final_filtered.ranges.begin(), final_filtered.ranges.end(), std::numeric_limits<float>::infinity());

//     for (const LaserSegment &segment : accepted_segments)
//     {
//         for (int i = segment.start_index; i <= segment.end_index; ++i)
//         {
//             if (i >= 0 && i < scan_count && std::isfinite(neighbour_filtered.ranges[i]))
//             {
//                 final_filtered.ranges[i] = neighbour_filtered.ranges[i];
//             }
//         }
//     }

//     return final_filtered;
// }

// std::vector<LaserSegment> MechelangeloBehaviour::buildLaserSegments(
//     const sensor_msgs::msg::LaserScan &scan) const
// {
//     std::vector<LaserSegment> segments;

//     if (scan.ranges.empty() || scan.angle_increment == 0.0)
//     {
//         return segments;
//     }

//     const int scan_count = static_cast<int>(scan.ranges.size());

//     bool segment_active = false;
//     LaserSegment current_segment;
//     geometry_msgs::msg::Point previous_point;

//     auto finish_segment = [&]()
//     {
//         if (!segment_active)
//         {
//             return;
//         }

//         current_segment.midpoint.x = 0.5 * (current_segment.start_point.x + current_segment.end_point.x);
//         current_segment.midpoint.y = 0.5 * (current_segment.start_point.y + current_segment.end_point.y);
//         current_segment.midpoint.z = 0.0;
//         current_segment.length = std::hypot(
//             current_segment.end_point.x - current_segment.start_point.x,
//             current_segment.end_point.y - current_segment.start_point.y);
//         current_segment.midpoint_angle = std::atan2(
//             current_segment.midpoint.y,
//             current_segment.midpoint.x);

//         if (current_segment.point_count >= kSegmentMinPoints &&
//             current_segment.length >= kSegmentMinLength)
//         {
//             segments.push_back(current_segment);
//         }

//         segment_active = false;
//         current_segment = LaserSegment();
//     };

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = scan.ranges[i];

//         if (!isRangeUsableForFiltering(scan, range))
//         {
//             finish_segment();
//             continue;
//         }

//         const geometry_msgs::msg::Point point = polarToPoint(scan, i);

//         if (!segment_active)
//         {
//             segment_active = true;
//             current_segment = LaserSegment();
//             current_segment.start_index = i;
//             current_segment.end_index = i;
//             current_segment.point_count = 1;
//             current_segment.start_point = point;
//             current_segment.end_point = point;
//             current_segment.min_range = range;
//             previous_point = point;
//             continue;
//         }

//         const double gap = std::hypot(point.x - previous_point.x, point.y - previous_point.y);

//         if (gap <= kSegmentJoinDistance)
//         {
//             current_segment.end_index = i;
//             current_segment.end_point = point;
//             current_segment.point_count++;
//             current_segment.min_range = std::min(current_segment.min_range, range);
//             previous_point = point;
//         }
//         else
//         {
//             finish_segment();

//             segment_active = true;
//             current_segment = LaserSegment();
//             current_segment.start_index = i;
//             current_segment.end_index = i;
//             current_segment.point_count = 1;
//             current_segment.start_point = point;
//             current_segment.end_point = point;
//             current_segment.min_range = range;
//             previous_point = point;
//         }
//     }

//     finish_segment();
//     return segments;
// }

// geometry_msgs::msg::Point MechelangeloBehaviour::polarToPoint(
//     const sensor_msgs::msg::LaserScan &scan,
//     int index) const
// {
//     geometry_msgs::msg::Point point;

//     if (index < 0 || index >= static_cast<int>(scan.ranges.size()))
//     {
//         return point;
//     }

//     const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
//     const double range = scan.ranges[index];

//     point.x = range * std::cos(angle);
//     point.y = range * std::sin(angle);
//     point.z = 0.0;

//     return point;
// }

// bool MechelangeloBehaviour::isRangeUsableForFiltering(
//     const sensor_msgs::msg::LaserScan &scan,
//     double range) const
// {
//     if (!std::isfinite(range))
//     {
//         return false;
//     }

//     if (range <= kMinValidRange)
//     {
//         return false;
//     }

//     if (scan.range_min > 0.0 && range < scan.range_min)
//     {
//         return false;
//     }

//     if (scan.range_max > 0.0 && range > scan.range_max)
//     {
//         return false;
//     }

//     return true;
// }

// bool MechelangeloBehaviour::isRangeValid(double range) const
// {
//     return std::isfinite(range) && range > kMinValidRange;
// }

// double MechelangeloBehaviour::getMinimumRange(double start_angle, double end_angle) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return std::numeric_limits<double>::infinity();
//     }

//     double min_range = std::numeric_limits<double>::infinity();

//     for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//     {
//         const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

//         if (!angleInsideWindow(angle, start_angle, end_angle))
//         {
//             continue;
//         }

//         const double range = latest_scan_.ranges[i];

//         if (isRangeValid(range) && range < min_range)
//         {
//             min_range = range;
//         }
//     }

//     return min_range;
// }

// double MechelangeloBehaviour::getFrontRange() const
// {
//     return getMinimumRange(-kFrontCheckAngle, kFrontCheckAngle);
// }

// bool MechelangeloBehaviour::getLongestRange(double &out_angle, double &out_range) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     const int scan_count = static_cast<int>(latest_scan_.ranges.size());
//     const double angle_step = std::fabs(latest_scan_.angle_increment);

//     if (scan_count <= 0 || angle_step <= 0.0)
//     {
//         return false;
//     }

//     // ------------------------------------------------------
//     // Step 1: classify beams as open or blocked.
//     //
//     // Open means either:
//     //   - infinity: the LiDAR did not hit anything within range, or
//     //   - a finite return farther than the clearance distance.
//     // ------------------------------------------------------
//     std::vector<bool> open_beam(scan_count, false);

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = latest_scan_.ranges[i];

//         if (std::isinf(range))
//         {
//             open_beam[i] = true;
//         }
//         else if (std::isfinite(range) && range >= kDvdOpenClearanceDistance)
//         {
//             open_beam[i] = true;
//         }
//     }

//     auto angleForIndex = [&](int index)
//     {
//         return normaliseAngle(
//             latest_scan_.angle_min + static_cast<double>(index) * latest_scan_.angle_increment);
//     };

//     auto absoluteAngleForIndex = [&](int index)
//     {
//         return std::fabs(angleForIndex(index));
//     };

//     auto rangeForLog = [&](int index)
//     {
//         const double range = latest_scan_.ranges[index];

//         if (std::isinf(range))
//         {
//             return std::numeric_limits<double>::infinity();
//         }

//         if (std::isfinite(range))
//         {
//             return range;
//         }

//         return 0.0;
//     };

//     auto isPreferredBounceAngle = [&](int index)
//     {
//         const double abs_angle = absoluteAngleForIndex(index);
//         return abs_angle >= kDvdPreferredMinTurnAngle &&
//                abs_angle <= kDvdPreferredMaxTurnAngle;
//     };

//     auto isFallbackBounceAngle = [&](int index)
//     {
//         const double abs_angle = absoluteAngleForIndex(index);
//         return abs_angle >= kDvdAvoidFrontAngle &&
//                abs_angle <= kDvdAvoidReverseAngle;
//     };

//     // ------------------------------------------------------
//     // Step 2: extract open sectors and collect safe candidate beams.
//     //
//     // Preferred candidates are side-bounce angles, roughly +/-55 to +/-150 deg.
//     // General fallback candidates simply avoid the front wall and exact reverse.
//     // Left and right candidates are stored separately so one long side of the
//     // room does not dominate every decision.
//     // ------------------------------------------------------
//     std::vector<int> preferred_left_candidates;
//     std::vector<int> preferred_right_candidates;
//     std::vector<int> fallback_left_candidates;
//     std::vector<int> fallback_right_candidates;
//     std::vector<int> fallback_all_candidates;

//     int accepted_sector_count = 0;
//     int widest_sector_count = 0;

//     auto collectCandidatesFromSector = [&](int start_index, int count)
//     {
//         if (count <= 0)
//         {
//             return;
//         }

//         const double sector_width = static_cast<double>(count) * angle_step;
//         widest_sector_count = std::max(widest_sector_count, count);

//         if (sector_width < kDvdMinSectorWidth)
//         {
//             return;
//         }

//         accepted_sector_count++;

//         int edge_margin_count = static_cast<int>(std::ceil(kDvdSectorEdgeMargin / angle_step));

//         // Never let the edge margin remove the entire sector.
//         edge_margin_count = std::min(edge_margin_count, std::max(0, (count - 1) / 2));

//         for (int offset = edge_margin_count; offset < count - edge_margin_count; ++offset)
//         {
//             const int index = (start_index + offset) % scan_count;
//             const double angle = angleForIndex(index);

//             if (isPreferredBounceAngle(index))
//             {
//                 if (angle >= 0.0)
//                 {
//                     preferred_left_candidates.push_back(index);
//                 }
//                 else
//                 {
//                     preferred_right_candidates.push_back(index);
//                 }
//             }

//             if (isFallbackBounceAngle(index))
//             {
//                 fallback_all_candidates.push_back(index);

//                 if (angle >= 0.0)
//                 {
//                     fallback_left_candidates.push_back(index);
//                 }
//                 else
//                 {
//                     fallback_right_candidates.push_back(index);
//                 }
//             }
//         }
//     };

//     const bool all_open = std::all_of(open_beam.begin(), open_beam.end(),
//         [](bool value) { return value; });

//     if (all_open)
//     {
//         collectCandidatesFromSector(0, scan_count);
//     }
//     else
//     {
//         for (int start_index = 0; start_index < scan_count; ++start_index)
//         {
//             if (!open_beam[start_index])
//             {
//                 continue;
//             }

//             const int previous_index = (start_index - 1 + scan_count) % scan_count;
//             if (open_beam[previous_index])
//             {
//                 continue;
//             }

//             int count = 0;
//             while (count < scan_count && open_beam[(start_index + count) % scan_count])
//             {
//                 count++;
//             }

//             collectCandidatesFromSector(start_index, count);
//         }
//     }

//     // ------------------------------------------------------
//     // Step 3: choose a random safe bounce angle.
//     //
//     // Preferred behaviour:
//     //   - use the side-bounce band if possible;
//     //   - choose left/right with a 50/50 coin flip when both are available;
//     //   - choose a random beam inside that side's safe candidate list.
//     // This produces a DVD-screensaver style movement instead of always going
//     // down the longest wall/side of the room.
//     // ------------------------------------------------------
//     static thread_local std::mt19937 rng(std::random_device{}());

//     auto chooseFromCandidates = [&](const std::vector<int> &candidates)
//     {
//         std::uniform_int_distribution<int> index_dist(
//             0, static_cast<int>(candidates.size()) - 1);
//         return candidates[index_dist(rng)];
//     };

//     auto chooseBalancedSide = [&](const std::vector<int> &left_candidates,
//                                   const std::vector<int> &right_candidates,
//                                   bool &used_left_side,
//                                   bool &used_right_side)
//     {
//         used_left_side = false;
//         used_right_side = false;

//         if (!left_candidates.empty() && !right_candidates.empty())
//         {
//             std::uniform_int_distribution<int> side_dist(0, 1);

//             if (side_dist(rng) == 0)
//             {
//                 used_left_side = true;
//                 return chooseFromCandidates(left_candidates);
//             }

//             used_right_side = true;
//             return chooseFromCandidates(right_candidates);
//         }

//         if (!left_candidates.empty())
//         {
//             used_left_side = true;
//             return chooseFromCandidates(left_candidates);
//         }

//         used_right_side = true;
//         return chooseFromCandidates(right_candidates);
//     };

//     int selected_index = -1;
//     bool used_preferred_bounce_band = false;
//     bool used_left_side = false;
//     bool used_right_side = false;

//     if (!preferred_left_candidates.empty() || !preferred_right_candidates.empty())
//     {
//         selected_index = chooseBalancedSide(
//             preferred_left_candidates,
//             preferred_right_candidates,
//             used_left_side,
//             used_right_side);
//         used_preferred_bounce_band = true;
//     }
//     else if (!fallback_left_candidates.empty() || !fallback_right_candidates.empty())
//     {
//         selected_index = chooseBalancedSide(
//             fallback_left_candidates,
//             fallback_right_candidates,
//             used_left_side,
//             used_right_side);
//         used_preferred_bounce_band = false;
//     }
//     else if (!fallback_all_candidates.empty())
//     {
//         selected_index = chooseFromCandidates(fallback_all_candidates);
//         used_left_side = angleForIndex(selected_index) >= 0.0;
//         used_right_side = !used_left_side;
//         used_preferred_bounce_band = false;
//     }

//     if (selected_index >= 0)
//     {
//         out_angle = angleForIndex(selected_index);
//         out_range = rangeForLog(selected_index);

//         {
//             std::ofstream log("/tmp/mechelangelo_dvd_bounce_debug.txt");
//             log << std::fixed << std::setprecision(4);
//             log << "=== DVD Bounce Heading Selection ===\n";
//             log << "Open beam rule: inf OR range >= " << kDvdOpenClearanceDistance << " m\n";
//             log << "Minimum accepted sector width: " << kDvdMinSectorWidth * 180.0 / M_PI << " deg\n";
//             log << "Sector edge margin: " << kDvdSectorEdgeMargin * 180.0 / M_PI << " deg\n";
//             log << "Preferred bounce band: +/-"
//                 << kDvdPreferredMinTurnAngle * 180.0 / M_PI << " to +/-"
//                 << kDvdPreferredMaxTurnAngle * 180.0 / M_PI << " deg\n";
//             log << "Fallback bounce band: +/-"
//                 << kDvdAvoidFrontAngle * 180.0 / M_PI << " to +/-"
//                 << kDvdAvoidReverseAngle * 180.0 / M_PI << " deg\n\n";

//             log << "Accepted open sectors: " << accepted_sector_count << "\n";
//             log << "Widest open sector beams: " << widest_sector_count << "\n";
//             log << "Preferred left candidates: " << preferred_left_candidates.size() << "\n";
//             log << "Preferred right candidates: " << preferred_right_candidates.size() << "\n";
//             log << "Fallback candidates: " << fallback_all_candidates.size() << "\n\n";

//             log << "=== Result ===\n";
//             log << "Selected behaviour: "
//                 << (used_preferred_bounce_band ? "preferred random side-bounce" : "fallback random safe bounce")
//                 << "\n";
//             log << "Selected side: "
//                 << (used_left_side ? "left/positive" : (used_right_side ? "right/negative" : "unknown"))
//                 << "\n";
//             log << "Selected index: " << selected_index << "\n";
//             log << "Selected angle: " << out_angle * 180.0 / M_PI << " deg\n";
//             log << "Representative range: " << out_range << " m\n\n";

//             log << "=== Laser Scan Values ===\n";
//             for (int i = 0; i < scan_count; ++i)
//             {
//                 const double angle_deg = angleForIndex(i) * 180.0 / M_PI;
//                 log << "  [" << std::setw(4) << i << "]  angle="
//                     << std::setw(9) << angle_deg << " deg  range="
//                     << latest_scan_.ranges[i] << " m  open="
//                     << (open_beam[i] ? "yes" : "no") << "  preferred="
//                     << (isPreferredBounceAngle(i) ? "yes" : "no") << "\n";
//             }
//         }

//         return true;
//     }

//     // ------------------------------------------------------
//     // Step 4: fallback to the old longest finite scan if the room is too
//     // cluttered for a safe random bounce heading.
//     // ------------------------------------------------------
//     double max_finite_range = 0.0;
//     int max_finite_index = -1;

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = latest_scan_.ranges[i];

//         if (isRangeValid(range) && range > max_finite_range)
//         {
//             max_finite_range = range;
//             max_finite_index = i;
//         }
//     }

//     if (max_finite_index < 0)
//     {
//         return false;
//     }

//     const double tied_range_threshold =
//         std::max(kMinValidRange, max_finite_range - kLongestRangeTieTolerance);

//     std::vector<bool> near_longest(scan_count, false);

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = latest_scan_.ranges[i];
//         near_longest[i] = isRangeValid(range) && range >= tied_range_threshold;
//     }

//     int best_start_index = max_finite_index;
//     int best_count = 0;

//     for (int start_index = 0; start_index < scan_count; ++start_index)
//     {
//         if (!near_longest[start_index])
//         {
//             continue;
//         }

//         const int previous_index = (start_index - 1 + scan_count) % scan_count;
//         if (near_longest[previous_index])
//         {
//             continue;
//         }

//         int count = 0;
//         while (count < scan_count && near_longest[(start_index + count) % scan_count])
//         {
//             count++;
//         }

//         if (count > best_count)
//         {
//             best_count = count;
//             best_start_index = start_index;
//         }
//     }

//     if (best_count == 0)
//     {
//         best_count = 1;
//         best_start_index = max_finite_index;
//     }

//     const double best_mid_index = std::fmod(
//         static_cast<double>(best_start_index) +
//             0.5 * static_cast<double>(std::max(0, best_count - 1)),
//         static_cast<double>(scan_count));

//     out_angle = normaliseAngle(latest_scan_.angle_min + best_mid_index * latest_scan_.angle_increment);
//     out_range = max_finite_range;

//     {
//         std::ofstream log("/tmp/mechelangelo_dvd_bounce_debug.txt");
//         log << std::fixed << std::setprecision(4);
//         log << "=== DVD Bounce Heading Selection ===\n";
//         log << "Selected behaviour: fallback longest finite scan\n";
//         log << "Longest finite range: " << out_range << " m\n";
//         log << "Selected angle: " << out_angle * 180.0 / M_PI << " deg\n";
//     }

//     return true;
// }

// int MechelangeloBehaviour::angleToIndex(double angle_rad) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return -1;
//     }

//     double capped_angle = angle_rad;

//     if (capped_angle < latest_scan_.angle_min)
//     {
//         capped_angle = latest_scan_.angle_min;
//     }

//     if (capped_angle > latest_scan_.angle_max)
//     {
//         capped_angle = latest_scan_.angle_max;
//     }

//     int index = static_cast<int>(
//         std::round((capped_angle - latest_scan_.angle_min) / latest_scan_.angle_increment));

//     index = std::clamp(index, 0, static_cast<int>(latest_scan_.ranges.size()) - 1);
//     return index;
// }

// double MechelangeloBehaviour::normaliseAngle(double angle_rad) const
// {
//     while (angle_rad > M_PI)
//     {
//         angle_rad -= 2.0 * M_PI;
//     }

//     while (angle_rad < -M_PI)
//     {
//         angle_rad += 2.0 * M_PI;
//     }

//     return angle_rad;
// }

// bool MechelangeloBehaviour::angleInsideWindow(
//     double angle_rad,
//     double start_angle,
//     double end_angle) const
// {
//     const double angle = normaliseAngle(angle_rad);
//     const double start = normaliseAngle(start_angle);
//     const double end = normaliseAngle(end_angle);

//     if (start <= end)
//     {
//         return angle >= start && angle <= end;
//     }

//     return angle >= start || angle <= end;
// }

// bool MechelangeloBehaviour::segmentOverlapsAngleWindow(
//     const LaserSegment &segment,
//     double start_angle,
//     double end_angle) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     for (int i = segment.start_index; i <= segment.end_index; ++i)
//     {
//         if (i < 0 || i >= static_cast<int>(latest_scan_.ranges.size()))
//         {
//             continue;
//         }

//         const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

//         if (angleInsideWindow(angle, start_angle, end_angle))
//         {
//             return true;
//         }
//     }

//     return false;
// }

// bool MechelangeloBehaviour::findBlockingObstaclesInFront(
//     std::vector<LaserSegment> &blocking_segments) const
// {
//     blocking_segments.clear();

//     for (const LaserSegment &segment : latest_segments_)
//     {
//         if (segment.min_range <= stop_distance_m_ &&
//             segmentOverlapsAngleWindow(segment, -kFrontCheckAngle, kFrontCheckAngle))
//         {
//             blocking_segments.push_back(segment);
//         }
//     }

//     return !blocking_segments.empty();
// }

// void MechelangeloBehaviour::publishObstacleMarkers(
//     const std::vector<LaserSegment> &blocking_segments)
// {
//     visualization_msgs::msg::MarkerArray marker_array;

//     visualization_msgs::msg::Marker clear_marker;
//     clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
//     clear_marker.header.stamp = this->get_clock()->now();
//     clear_marker.ns = "behaviour_blocking_obstacles";
//     clear_marker.id = 0;
//     clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//     marker_array.markers.push_back(clear_marker);

//     int marker_id = 1;

//     for (const LaserSegment &segment : blocking_segments)
//     {
//         visualization_msgs::msg::Marker marker;
//         marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
//         marker.header.stamp = this->get_clock()->now();
//         marker.ns = "behaviour_blocking_obstacles";
//         marker.id = marker_id++;
//         marker.type = visualization_msgs::msg::Marker::CYLINDER;
//         marker.action = visualization_msgs::msg::Marker::ADD;

//         marker.pose.position.x = segment.midpoint.x;
//         marker.pose.position.y = segment.midpoint.y;
//         marker.pose.position.z = 0.15;
//         marker.pose.orientation.x = 0.0;
//         marker.pose.orientation.y = 0.0;
//         marker.pose.orientation.z = 0.0;
//         marker.pose.orientation.w = 1.0;

//         // Make marker size scale slightly with the observed segment length.
//         const double marker_width = std::clamp(segment.length + 0.15, 0.20, 0.80);
//         marker.scale.x = marker_width;
//         marker.scale.y = marker_width;
//         marker.scale.z = 0.30;

//         // Red/orange transparent marker for blocking obstacle.
//         marker.color.a = 0.75F;
//         marker.color.r = 1.0F;
//         marker.color.g = 0.15F;
//         marker.color.b = 0.0F;

//         marker.lifetime.sec = 0;
//         marker.lifetime.nanosec = 400000000; // 0.4 s

//         marker_array.markers.push_back(marker);
//     }

//     obstacle_marker_publisher_->publish(marker_array);
// }

// void MechelangeloBehaviour::clearObstacleMarkers()
// {
//     if (!obstacle_marker_publisher_)
//     {
//         return;
//     }

//     visualization_msgs::msg::MarkerArray marker_array;
//     visualization_msgs::msg::Marker clear_marker;
//     clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
//     clear_marker.header.stamp = this->get_clock()->now();
//     clear_marker.ns = "behaviour_blocking_obstacles";
//     clear_marker.id = 0;
//     clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//     marker_array.markers.push_back(clear_marker);
//     obstacle_marker_publisher_->publish(marker_array);
// }

// void MechelangeloBehaviour::longestLaserScan()
// {
//     double longest_angle = 0.0;
//     double longest_range = 0.0;

//     if (!getLongestRange(longest_angle, longest_range))
//     {
//         RCLCPP_WARN(
//             this->get_logger(),
//             "No valid filtered laser scan data available for longest scan calculation.");
//         return;
//     }

//     RCLCPP_INFO(
//         this->get_logger(),
//         "Selected exploration heading: Representative range = %.2f m at Angle = %.2f degrees",
//         longest_range,
//         longest_angle * 180.0 / M_PI);
// }

// double MechelangeloBehaviour::getHumanLidarRange(double centre_offset) const
// {
//     const double estimated_human_angle = -centre_offset * kCameraHorizontalFov;
//     const double start_angle = estimated_human_angle - kHumanLidarWindow;
//     const double end_angle = estimated_human_angle + kHumanLidarWindow;

//     return getMinimumRange(start_angle, end_angle);
// }

// void MechelangeloBehaviour::captureSafetyZoneBaseline()
// {
//     safety_zone_baseline_scan_ = latest_scan_;
//     safety_zone_baseline_captured_ = true;
//     RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Filtered background baseline captured.");
// }

// bool MechelangeloBehaviour::isSafetyZoneViolated(double human_bearing_rad) const
// {
//     if (!safety_zone_baseline_captured_)
//     {
//         return false;
//     }

//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     if (safety_zone_baseline_scan_.ranges.size() != latest_scan_.ranges.size())
//     {
//         return false;
//     }

//     for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//     {
//         const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
//         const double angle_diff = normaliseAngle(angle - human_bearing_rad);

//         // Exclude the window around the tracked human so they do not self-trigger.
//         if (std::fabs(angle_diff) <= kSafetyZoneHumanExclusionAngle)
//         {
//             continue;
//         }

//         const double current_range = latest_scan_.ranges[i];

//         if (!std::isfinite(current_range) || current_range <= kMinValidRange)
//         {
//             continue;
//         }

//         if (current_range >= kSafetyZoneRadius)
//         {
//             continue;
//         }

//         const double baseline_range = std::isfinite(safety_zone_baseline_scan_.ranges[i])
//             ? safety_zone_baseline_scan_.ranges[i]
//             : kSafetyZoneRadius;

//         if (current_range < (baseline_range - kSafetyZoneIntruderThreshold))
//         {
//             return true;
//         }
//     }

//     return false;
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<MechelangeloBehaviour>();

//     // true = simulation mode
//     // false = real robot mode
//     node->run(true);

//     rclcpp::shutdown();
//     return 0;
// }

//^^^DVD CODE ABOVE AND BELOW IS THE REPOSITIONING CODE FOR HANDLING HUMANS CLOSE TO THE WALL
/////

/////////////////////////////////////////////////////////////////////////
///DVD style movemenmt

// #include "behaviour.hpp"

// #include <algorithm>
// #include <chrono>
// #include <cmath>
// #include <fstream>
// #include <iomanip>
// #include <limits>
// #include <functional>
// #include <memory>
// #include <random>
// #include <string>
// #include <vector>

// using namespace std::chrono_literals;

// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif

// // ------------------------------------------------------
// // Behaviour constants
// // ------------------------------------------------------

// // Control loop runs every 100 ms.
// static constexpr double kControlPeriodSeconds = 0.1;

// // Movement tuning.
// static constexpr double kForwardSpeed = 0.26;       // m/s
// static constexpr double kTurnSpeed = 0.6;           // rad/s
// static constexpr double kAngleGain = 0.8;           // proportional turning gain
// static constexpr double kAlignmentTolerance = 0.10; // radians, about 5.7 degrees

// // Smooth commanded stops so the physical base does not snap from motion to zero
// // in one control tick. At the 100 ms control period, these remove roughly
// // 0.04 m/s and 0.12 rad/s from the command each loop.
// static constexpr double kStopLinearDecel = 0.4;  // m/s^2
// static constexpr double kStopAngularDecel = 1.2; // rad/s^2

// // Stop this far before a real obstacle/wall.
// // Loaded from the ROS parameter 'stop_distance_m'.
// // Default: 1.5 m (simulation). Physical robot: set to 0.75 in the launch file.

// // 30 loops x 0.1 s = 3 seconds.
// static constexpr int kStopDurationLoops = 30;

// // Ignore returns too close to the robot body / lidar blind spot.
// static constexpr double kMinValidRange = 0.5; // m

// // Front scan window used while moving forward.
// static constexpr double kFrontCheckAngle = 30 * M_PI / 180.0; // +/- 30 degrees

// // When several neighbouring beams are effectively tied for the longest
// // distance, steer toward the middle of that opening instead of whichever beam
// // happens to appear first in the scan array.
// static constexpr double kLongestRangeTieTolerance = 0.05; // m

// // ------------------------------------------------------
// // DVD-style bounce exploration tuning
// // ------------------------------------------------------
// // The autonomous gallery behaviour is intended to move like a DVD screensaver:
// // drive straight until the front is blocked, stop at a safe distance, then pick
// // a new safe side-bounce angle instead of always chasing the longest wall.
// //
// // A beam is considered open if it is infinity OR farther than this clearance.
// // Infinity is useful in large rooms because it means the LiDAR did not hit
// // anything within range.
// static constexpr double kDvdOpenClearanceDistance = 4.5; // m

// // Ignore very narrow open gaps. This prevents the robot from aiming through
// // thin laser cracks between obstacles.
// static constexpr double kDvdMinSectorWidth = 18.0 * M_PI / 180.0; // radians

// // Trim candidate angles away from the edge of an open sector.
// static constexpr double kDvdSectorEdgeMargin = 8.0 * M_PI / 180.0; // radians

// // Preferred bounce band after the robot stops at a wall.
// // These angles are relative to the robot's current forward direction:
// //   0 deg   = back into the wall/obstacle it just stopped for
// //   +/-90   = side-bounce
// //   +/-180  = drive exactly back along the previous path
// // The robot randomly chooses inside this safe side band when possible.
// static constexpr double kDvdPreferredMinTurnAngle = 55.0 * M_PI / 180.0;  // radians
// static constexpr double kDvdPreferredMaxTurnAngle = 150.0 * M_PI / 180.0; // radians

// // Fallback band used if the preferred side-bounce band has no safe candidates.
// // This still avoids the front wall and avoids exact reverse.
// static constexpr double kDvdAvoidFrontAngle = 35.0 * M_PI / 180.0;  // radians
// static constexpr double kDvdAvoidReverseAngle = 165.0 * M_PI / 180.0; // radians

// // ------------------------------------------------------
// // LaserScan noise suppression constants
// // ------------------------------------------------------
// // The filter is based on the previous LaserProcessing::countSegments()
// // approach: valid objects form segments of neighbouring points. Random
// // dots are usually isolated or only one/two points, so they get replaced
// // with infinity and will not stop the robot.

// // Stage 1: local neighbour test.
// static constexpr int kNoiseNeighbourWindow = 4;          // check +/- 4 beams
// static constexpr int kNoiseMinNeighbourCount = 2;        // require at least 2 close neighbours
// static constexpr double kNoiseNeighbourDistance = 0.22;  // m in local XY space

// // Stage 2: segment extraction test.
// static constexpr double kSegmentJoinDistance = 0.18;     // m max gap between consecutive points
// static constexpr int kSegmentMinPoints = 4;              // reject tiny speckle clusters
// static constexpr double kSegmentMinLength = 0.05;        // m reject near-zero length segments

// // ------------------------------------------------------
// // Human tracking tuning
// // ------------------------------------------------------
// // /human_tracking message format:
// // data[0] = detected, data[1] = centre_offset, data[2] = distance_m
// // centre_offset is normalised image offset from centre: -0.5 left, 0 centre, +0.5 right.
// static constexpr double kHumanTargetDistance = 1.5;     // m
// static constexpr double kHumanDistanceTolerance = 0.15; // m
// static constexpr double kHumanMaxForwardSpeed = 0.22;   // m/s
// static constexpr double kHumanMaxReverseSpeed = 0.12;   // m/s
// static constexpr double kHumanMaxTurnSpeed = 0.7;       // rad/s
// static constexpr double kHumanTurnGain = 1.8;           // image offset to angular speed
// static constexpr double kHumanForwardGain = 0.35;       // distance error to linear speed
// static constexpr double kHumanCentreDeadZone = 0.06;    // normalised image width
// static constexpr double kHumanLostTimeout = 1.0;        // seconds

// // LiDAR validation for human distance.
// static constexpr double kCameraHorizontalFov = 60.0 * M_PI / 180.0;
// static constexpr double kHumanLidarWindow = 10.0 * M_PI / 180.0;
// static constexpr double kLidarCameraMaxDisagreement = 0.4;
// static constexpr double kHumanLidarStopDistance = 1.5;
// static constexpr double kHumanLidarStopTolerance = 0.15;

// // Safety zone.
// static constexpr double kSafetyZoneRadius = 1.5;
// static constexpr double kSafetyZoneIntruderThreshold = 0.3;
// static constexpr double kSafetyZoneHumanExclusionAngle = 25.0 * M_PI / 180.0;

// // ------------------------------------------------------
// // Human interaction repositioning tuning
// // ------------------------------------------------------
// // The robot needs a clear space around itself before arm interaction.
// // If the human is visible but the 1.5 m arm bubble is blocked by a wall or
// // obstacle, the robot does not drive straight at the human. Instead it samples
// // short forward/arc moves and chooses the one predicted to clear the bubble
// // while still keeping the human in view.
// static constexpr double kInteractionBubbleRadius = 1.5;                 // m, arm movement clearance around robot
// static constexpr double kInteractionBubbleSafetyMargin = 0.10;          // m, extra buffer added to the bubble check
// static constexpr double kInteractionHumanExclusionAngle = 25.0 * M_PI / 180.0; // ignore tracked human cone
// static constexpr int kInteractionAllowedBlockedBeams = 3;               // tolerate a few filtered/noisy beams
// static constexpr double kInteractionRepositionLookahead = 0.55;         // m, predicted short move distance
// static constexpr double kInteractionRepositionSpeed = 0.10;             // m/s, slow reposition speed
// static constexpr double kInteractionRepositionTurnGain = 1.2;           // rad/s per rad target heading
// static constexpr double kInteractionPathHalfWidth = 0.45;               // m, collision corridor half-width
// static constexpr double kInteractionPathForwardBuffer = 0.25;           // m, extra forward collision buffer
// static constexpr double kInteractionMaxCandidateAngle = 75.0 * M_PI / 180.0;

// MechelangeloBehaviour::MechelangeloBehaviour()
// : Node("mechelangelo_behaviour"),
//   human_locked_(false),
//   human_centre_offset_(0.0),
//   human_distance_m_(-1.0),
//   blind_autonomous_active_(false),
//   safety_zone_violated_(false),
//   safety_zone_baseline_captured_(false),
//   current_state_(NavigationState::SEARCHING),
//   target_angle_(0.0),
//   target_range_(0.0),
//   stop_distance_m_(1.75),
//   stop_counter_(0),
//   imu_available_(false),
//   align_start_yaw_(0.0),
//   align_yaw_initialised_(false),
//   random_engine_(std::random_device{}()),
//   turn_dist_(-1.0, 1.0)
// {
//     this->declare_parameter("stop_distance_m", 1.75);
//     stop_distance_m_ = this->get_parameter("stop_distance_m").as_double();

//     RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_m_);

//     laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         rclcpp::SensorDataQoS(),
//         std::bind(&MechelangeloBehaviour::laserScanCallback, this, std::placeholders::_1));

//     imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
//         "/imu",
//         rclcpp::SensorDataQoS(),
//         std::bind(&MechelangeloBehaviour::imuCallback, this, std::placeholders::_1));

//     cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
//         "/cmd_vel",
//         10);

//     filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
//         "/scan_filtered",
//         rclcpp::SensorDataQoS());

//     obstacle_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//         "/behaviour_obstacle_markers",
//         10);

//     human_detected_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
//         "/human_detected",
//         10,
//         std::bind(&MechelangeloBehaviour::humanDetectedCallback, this, std::placeholders::_1));

//     human_tracking_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
//         "/human_tracking",
//         10,
//         std::bind(&MechelangeloBehaviour::humanTrackingCallback, this, std::placeholders::_1));

//     last_human_tracking_time_ = this->now();

//     control_timer_ = this->create_wall_timer(
//         100ms,
//         std::bind(&MechelangeloBehaviour::controlLoop, this));

//     RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been started.");
// }

// MechelangeloBehaviour::~MechelangeloBehaviour()
// {
//     RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been stopped.");
// }

// void MechelangeloBehaviour::run(bool sim_mode)
// {
//     RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node is running.");

//     if (sim_mode)
//     {
//         RCLCPP_INFO(this->get_logger(), "Running in simulation mode.");
//     }
//     else
//     {
//         RCLCPP_INFO(this->get_logger(), "Running in real robot mode.");
//     }

//     blindAutonomous();
//     rclcpp::spin(shared_from_this());
// }

// void MechelangeloBehaviour::blindAutonomous()
// {
//     RCLCPP_INFO(this->get_logger(), "Executing blind autonomous behaviour.");

//     blind_autonomous_active_ = true;
//     safety_zone_violated_ = false;
//     safety_zone_baseline_captured_ = false;
//     current_state_ = NavigationState::SEARCHING;
//     target_angle_ = 0.0;
//     target_range_ = 0.0;
//     stop_counter_ = 0;
//     align_yaw_initialised_ = false;
//     clearObstacleMarkers();
// }

// void MechelangeloBehaviour::mappedAutonomous()
// {
//     RCLCPP_INFO(this->get_logger(), "Executing mapped autonomous behaviour.");
// }

// void MechelangeloBehaviour::controlLoop()
// {
//     if (!blind_autonomous_active_)
//     {
//         return;
//     }

//     geometry_msgs::msg::Twist twist;

//     // Safety: wait until valid LaserScan data exists.
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         RCLCPP_WARN_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             2000,
//             "Waiting for valid filtered LaserScan data...");

//         stopRobot(twist);
//         current_twist_ = twist;
//         cmd_vel_publisher_->publish(twist);
//         return;
//     }

//     switch (current_state_)
//     {
//     case NavigationState::SEARCHING:
//     {
//         stopRobot(twist);
//         clearObstacleMarkers();

//         double longest_angle = 0.0;
//         double longest_range = 0.0;

//         if (!getLongestRange(longest_angle, longest_range))
//         {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 2000,
//                 "SEARCHING: No valid filtered LaserScan range found.");
//             break;
//         }

//         target_angle_ = longest_angle;
//         target_range_ = longest_range;
//         align_yaw_initialised_ = false;

//         RCLCPP_INFO(
//             this->get_logger(),
//             "SEARCHING: Selected exploration heading at %.2f deg, representative range %.2f m",
//             target_angle_ * 180.0 / M_PI,
//             target_range_);

//         current_state_ = NavigationState::ALIGNING;
//         break;
//     }

//     case NavigationState::ALIGNING:
//     {
//         clearObstacleMarkers();
//         twist.linear.x = 0.0;

//         if (!imu_available_)
//         {
//             // IMU not yet publishing — fall back to open-loop time integration.
//             if (std::fabs(target_angle_) <= kAlignmentTolerance)
//             {
//                 stopRobot(twist);
//                 if (std::fabs(twist.angular.z) <= 1e-6)
//                 {
//                     RCLCPP_INFO(this->get_logger(),
//                         "ALIGNING (open-loop): Aligned. Starting forward movement.");
//                     current_state_ = NavigationState::MOVING;
//                 }
//                 break;
//             }

//             const double turn_cmd = std::clamp(
//                 target_angle_ * kAngleGain, -kTurnSpeed, kTurnSpeed);
//             twist.angular.z = turn_cmd;
//             target_angle_ -= turn_cmd * kControlPeriodSeconds;
//             target_angle_ = normaliseAngle(target_angle_);

//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//                 "ALIGNING: No IMU data yet — using open-loop time estimate. "
//                 "Remaining %.2f deg", target_angle_ * 180.0 / M_PI);
//             break;
//         }

//         // IMU-confirmed alignment: compare actual yaw turned vs. required angle.
//         tf2::Quaternion q;
//         tf2::fromMsg(latest_imu_.orientation, q);
//         const double current_yaw = tf2::getYaw(q);

//         if (!align_yaw_initialised_)
//         {
//             align_start_yaw_ = current_yaw;
//             align_yaw_initialised_ = true;
//         }

//         // How much has the robot actually rotated since ALIGNING began.
//         const double yaw_turned = normaliseAngle(current_yaw - align_start_yaw_);

//         // How many degrees still remain.
//         const double remaining_angle = normaliseAngle(target_angle_ - yaw_turned);

//         if (std::fabs(remaining_angle) <= kAlignmentTolerance)
//         {
//             stopRobot(twist);
//             if (std::fabs(twist.angular.z) <= 1e-6)
//             {
//                 RCLCPP_INFO(this->get_logger(),
//                     "ALIGNING: IMU confirmed rotation. Turned %.2f deg (target %.2f deg). Starting forward movement.",
//                     yaw_turned * 180.0 / M_PI,
//                     target_angle_ * 180.0 / M_PI);
//                 current_state_ = NavigationState::MOVING;
//             }
//             break;
//         }

//         const double turn_cmd = std::clamp(
//             remaining_angle * kAngleGain, -kTurnSpeed, kTurnSpeed);
//         twist.angular.z = turn_cmd;

//         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//             "ALIGNING: IMU-confirmed. Target %.2f deg, turned %.2f deg, remaining %.2f deg, cmd %.2f rad/s",
//             target_angle_ * 180.0 / M_PI,
//             yaw_turned * 180.0 / M_PI,
//             remaining_angle * 180.0 / M_PI,
//             twist.angular.z);

//         break;
//     }

//     case NavigationState::MOVING:
//     {
//         std::vector<LaserSegment> blocking_segments;
//         const bool blocked_by_segment = findBlockingObstaclesInFront(blocking_segments);
//         const double front_range = getFrontRange();

//         // Segment-based blocking is the main decision. This prevents a single
//         // random dot from stopping the robot because the dot will not survive
//         // the neighbour + segment filter.
//         if (blocked_by_segment || front_range <= stop_distance_m_)
//         {
//             if (blocking_segments.empty())
//             {
//                 // Fallback marker if the range check caught something but no
//                 // segment was available. This should be rare after filtering.
//                 LaserSegment fallback;
//                 fallback.point_count = 1;
//                 fallback.min_range = front_range;
//                 fallback.midpoint.x = std::isfinite(front_range) ? front_range : stop_distance_m_;
//                 fallback.midpoint.y = 0.0;
//                 fallback.midpoint.z = 0.0;
//                 blocking_segments.push_back(fallback);
//             }

//             publishObstacleMarkers(blocking_segments);

//             RCLCPP_WARN(
//                 this->get_logger(),
//                 "MOVING: Blocking obstacle detected in front. Front range = %.2f m. Stopping.",
//                 front_range);

//             stopRobot(twist);
//             stop_counter_ = 0;
//             current_state_ = NavigationState::STOPPED;
//             break;
//         }

//         clearObstacleMarkers();

//         if (std::isinf(front_range))
//         {
//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "MOVING: Front is clear after filtering. Driving forward.");
//         }
//         else
//         {
//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "MOVING: Driving forward. Filtered front range = %.2f m",
//                 front_range);
//         }

//         twist.linear.x = kForwardSpeed;
//         twist.angular.z = 0.0;
//         break;
//     }

//     case NavigationState::STOPPED:
//     {
//         stopRobot(twist);
//         stop_counter_++;

//         if (stop_counter_ >= kStopDurationLoops)
//         {
//             RCLCPP_INFO(
//                 this->get_logger(),
//                 "STOPPED: Pause complete. Searching for next direction.");

//             stop_counter_ = 0;
//             clearObstacleMarkers();
//             current_state_ = NavigationState::SEARCHING;
//         }
//         else
//         {
//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "STOPPED: Pausing %.1f / %.1f seconds",
//                 stop_counter_ * kControlPeriodSeconds,
//                 kStopDurationLoops * kControlPeriodSeconds);
//         }

//         break;
//     }

//     case NavigationState::HUMAN_DETECTED:
//     {
//         const double time_since_tracking =
//             (this->now() - last_human_tracking_time_).seconds();

//         if (!human_locked_ || time_since_tracking > kHumanLostTimeout)
//         {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: Human lost. Returning to blind autonomous search.");

//             stopRobot(twist);
//             human_locked_ = false;
//             safety_zone_violated_ = false;
//             safety_zone_baseline_captured_ = false;
//             clearObstacleMarkers();
//             current_state_ = NavigationState::SEARCHING;
//             break;
//         }

//         if (!safety_zone_baseline_captured_)
//         {
//             captureSafetyZoneBaseline();
//         }

//         const double human_bearing_rad = -human_centre_offset_ * kCameraHorizontalFov;
//         const bool zone_now_violated = isSafetyZoneViolated(human_bearing_rad);

//         if (zone_now_violated != safety_zone_violated_)
//         {
//             safety_zone_violated_ = zone_now_violated;

//             if (safety_zone_violated_)
//             {
//                 RCLCPP_WARN(
//                     this->get_logger(),
//                     "SAFETY ZONE: Object detected within %.1f m. Interaction paused.",
//                     kSafetyZoneRadius);
//             }
//             else
//             {
//                 RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Clear. Resuming interaction.");
//             }
//         }

//         if (safety_zone_violated_)
//         {
//             stopRobot(twist);
//             break;
//         }

//         // ------------------------------------------------------
//         // Human tracking + interaction-space behaviour
//         // ------------------------------------------------------
//         // The normal human approach command keeps the person centred and moves
//         // to the target distance. The extra logic below checks whether there is
//         // enough 360-degree clearance for the arms. If not, the robot chooses a
//         // short safe arc that is predicted to improve the 1.5 m interaction
//         // bubble while still keeping the human in view.

//         const double human_keep_turn =
//             (std::fabs(human_centre_offset_) <= kHumanCentreDeadZone)
//                 ? 0.0
//                 : std::clamp(
//                     -kHumanTurnGain * human_centre_offset_,
//                     -kHumanMaxTurnSpeed,
//                     kHumanMaxTurnSpeed);

//         const double human_lidar_range = getHumanLidarRange(human_centre_offset_);
//         const bool lidar_distance_valid = std::isfinite(human_lidar_range);

//         const double estimated_human_range =
//             lidar_distance_valid
//                 ? human_lidar_range
//                 : ((human_distance_m_ > 0.0 && std::isfinite(human_distance_m_))
//                     ? human_distance_m_
//                     : kHumanTargetDistance);

//         const double human_x = estimated_human_range * std::cos(human_bearing_rad);
//         const double human_y = estimated_human_range * std::sin(human_bearing_rad);

//         struct InteractionBubbleCheck
//         {
//             int considered_beams = 0;
//             int blocked_beams = 0;
//             double min_clearance = std::numeric_limits<double>::infinity();
//             double blocked_fraction = 0.0;
//             bool clear = false;
//         };

//         auto evaluateInteractionBubbleAt = [&](double future_x, double future_y)
//         {
//             InteractionBubbleCheck check;
//             const double required_radius =
//                 kInteractionBubbleRadius + kInteractionBubbleSafetyMargin;

//             if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//             {
//                 check.clear = false;
//                 check.blocked_beams = 9999;
//                 check.blocked_fraction = 1.0;
//                 check.min_clearance = 0.0;
//                 return check;
//             }

//             for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//             {
//                 const double range = latest_scan_.ranges[i];

//                 if (!std::isfinite(range) || range <= kMinValidRange)
//                 {
//                     continue;
//                 }

//                 const double angle = latest_scan_.angle_min +
//                     static_cast<double>(i) * latest_scan_.angle_increment;
//                 const double angle_diff_to_human = normaliseAngle(angle - human_bearing_rad);

//                 // The tracked human is allowed to be in the front interaction
//                 // window. Everything else inside the arm bubble blocks interaction.
//                 if (std::fabs(angle_diff_to_human) <= kInteractionHumanExclusionAngle)
//                 {
//                     continue;
//                 }

//                 const double point_x = range * std::cos(angle);
//                 const double point_y = range * std::sin(angle);
//                 const double distance_to_future_robot =
//                     std::hypot(point_x - future_x, point_y - future_y);

//                 check.considered_beams++;
//                 check.min_clearance = std::min(check.min_clearance, distance_to_future_robot);

//                 if (distance_to_future_robot < required_radius)
//                 {
//                     check.blocked_beams++;
//                 }
//             }

//             if (check.considered_beams > 0)
//             {
//                 check.blocked_fraction = static_cast<double>(check.blocked_beams) /
//                     static_cast<double>(check.considered_beams);
//             }
//             else
//             {
//                 // No finite obstacles outside the human window means the bubble
//                 // is clear as far as LiDAR can tell.
//                 check.blocked_fraction = 0.0;
//             }

//             check.clear = check.blocked_beams <= kInteractionAllowedBlockedBeams;
//             return check;
//         };

//         auto pathToFuturePoseIsClear = [&](double heading, double move_distance)
//         {
//             if (move_distance <= 1e-3)
//             {
//                 return true;
//             }

//             const double dir_x = std::cos(heading);
//             const double dir_y = std::sin(heading);
//             const double max_forward = move_distance + kInteractionPathForwardBuffer;

//             for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//             {
//                 const double range = latest_scan_.ranges[i];

//                 if (!std::isfinite(range) || range <= kMinValidRange)
//                 {
//                     continue;
//                 }

//                 const double angle = latest_scan_.angle_min +
//                     static_cast<double>(i) * latest_scan_.angle_increment;
//                 const double angle_diff_to_human = normaliseAngle(angle - human_bearing_rad);

//                 if (std::fabs(angle_diff_to_human) <= kInteractionHumanExclusionAngle)
//                 {
//                     continue;
//                 }

//                 const double point_x = range * std::cos(angle);
//                 const double point_y = range * std::sin(angle);

//                 const double forward = point_x * dir_x + point_y * dir_y;
//                 const double lateral = -point_x * dir_y + point_y * dir_x;

//                 if (forward > 0.0 && forward < max_forward &&
//                     std::fabs(lateral) < kInteractionPathHalfWidth)
//                 {
//                     return false;
//                 }
//             }

//             return true;
//         };

//         const InteractionBubbleCheck current_bubble = evaluateInteractionBubbleAt(0.0, 0.0);
//         const bool interaction_bubble_clear = current_bubble.clear;

//         if (lidar_distance_valid &&
//             human_distance_m_ > 0.0 &&
//             std::isfinite(human_distance_m_))
//         {
//             const double disagreement = std::fabs(human_distance_m_ - human_lidar_range);

//             if (disagreement > kLidarCameraMaxDisagreement)
//             {
//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     1000,
//                     "HUMAN_DETECTED: Camera/LiDAR distance disagreement. Camera=%.2f m, LiDAR=%.2f m",
//                     human_distance_m_,
//                     human_lidar_range);
//             }
//         }

//         const double distance_error = estimated_human_range - kHumanLidarStopDistance;
//         const bool at_human_target_distance =
//             std::fabs(distance_error) <= kHumanLidarStopTolerance;

//         // If the robot is at interaction distance but the arm bubble is blocked,
//         // do not enter arm interaction. Reposition instead.
//         const bool needs_interaction_reposition = !interaction_bubble_clear;

//         if (needs_interaction_reposition)
//         {
//             struct RepositionCandidate
//             {
//                 double heading = 0.0;
//                 double move_distance = 0.0;
//                 double linear = 0.0;
//                 double angular = 0.0;
//                 double score = -std::numeric_limits<double>::infinity();
//                 InteractionBubbleCheck bubble;
//                 bool path_clear = false;
//             };

//             const std::vector<double> candidate_headings = {
//                 -75.0 * M_PI / 180.0,
//                 -60.0 * M_PI / 180.0,
//                 -45.0 * M_PI / 180.0,
//                 -30.0 * M_PI / 180.0,
//                 -15.0 * M_PI / 180.0,
//                   0.0,
//                  15.0 * M_PI / 180.0,
//                  30.0 * M_PI / 180.0,
//                  45.0 * M_PI / 180.0,
//                  60.0 * M_PI / 180.0,
//                  75.0 * M_PI / 180.0
//             };

//             RepositionCandidate best_candidate;
//             const double required_radius =
//                 kInteractionBubbleRadius + kInteractionBubbleSafetyMargin;

//             for (const double heading : candidate_headings)
//             {
//                 const double abs_heading = std::fabs(heading);

//                 // More sideways arcs move a little less in one decision step.
//                 const double move_scale = std::clamp(
//                     1.0 - 0.45 * (abs_heading / kInteractionMaxCandidateAngle),
//                     0.45,
//                     1.0);
//                 const double move_distance = kInteractionRepositionLookahead * move_scale;
//                 const double future_x = move_distance * std::cos(heading);
//                 const double future_y = move_distance * std::sin(heading);

//                 RepositionCandidate candidate;
//                 candidate.heading = heading;
//                 candidate.move_distance = move_distance;
//                 candidate.path_clear = pathToFuturePoseIsClear(heading, move_distance);
//                 candidate.bubble = evaluateInteractionBubbleAt(future_x, future_y);

//                 const double future_human_bearing = normaliseAngle(
//                     std::atan2(human_y - future_y, human_x - future_x));
//                 const double future_human_distance =
//                     std::hypot(human_x - future_x, human_y - future_y);

//                 const double human_center_score = std::clamp(
//                     1.0 - std::fabs(future_human_bearing) / (0.5 * kCameraHorizontalFov),
//                     0.0,
//                     1.0);
//                 const double human_distance_score = std::clamp(
//                     1.0 - std::fabs(future_human_distance - kHumanLidarStopDistance) / 1.0,
//                     0.0,
//                     1.0);
//                 const double clearance_score = std::clamp(
//                     candidate.bubble.min_clearance / required_radius,
//                     0.0,
//                     1.4);
//                 const double blocked_score = 1.0 - std::clamp(
//                     candidate.bubble.blocked_fraction,
//                     0.0,
//                     1.0);

//                 // Strongly reward future arm clearance, then human visibility.
//                 // Penalise aggressive turns so the reposition feels like a subtle
//                 // interaction adjustment, not a full return to exploration.
//                 candidate.score =
//                     6.0 * blocked_score +
//                     3.0 * clearance_score +
//                     2.5 * human_center_score +
//                     1.5 * human_distance_score -
//                     0.8 * (abs_heading / kInteractionMaxCandidateAngle);

//                 if (!candidate.path_clear)
//                 {
//                     candidate.score -= 8.0;
//                 }

//                 // Prefer candidates that improve the currently blocked bubble.
//                 if (candidate.bubble.blocked_beams < current_bubble.blocked_beams)
//                 {
//                     candidate.score += 2.0;
//                 }

//                 const double candidate_turn = std::clamp(
//                     candidate.heading * kInteractionRepositionTurnGain,
//                     -kHumanMaxTurnSpeed,
//                     kHumanMaxTurnSpeed);

//                 // Blend the reposition direction with the normal human-centering turn.
//                 candidate.angular = std::clamp(
//                     0.60 * candidate_turn + 0.40 * human_keep_turn,
//                     -kHumanMaxTurnSpeed,
//                     kHumanMaxTurnSpeed);

//                 candidate.linear = std::clamp(
//                     kInteractionRepositionSpeed * move_scale,
//                     0.04,
//                     kInteractionRepositionSpeed);

//                 if (candidate.score > best_candidate.score)
//                 {
//                     best_candidate = candidate;
//                 }
//             }

//             if (best_candidate.score > -1.0)
//             {
//                 twist.linear.x = best_candidate.linear;
//                 twist.angular.z = best_candidate.angular;

//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     750,
//                     "HUMAN_REPOSITION: Arm bubble blocked now (%d beams, min %.2f m). "
//                     "Best heading %.1f deg -> future blocked %d beams, min %.2f m. cmd linear=%.2f angular=%.2f",
//                     current_bubble.blocked_beams,
//                     current_bubble.min_clearance,
//                     best_candidate.heading * 180.0 / M_PI,
//                     best_candidate.bubble.blocked_beams,
//                     best_candidate.bubble.min_clearance,
//                     twist.linear.x,
//                     twist.angular.z);
//             }
//             else
//             {
//                 // If there is no safe reposition movement, at least keep the
//                 // human in view and do not enter arm interaction.
//                 twist.linear.x = 0.0;
//                 twist.angular.z = human_keep_turn;

//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     1000,
//                     "HUMAN_REPOSITION: No safe movement found to clear arm bubble. Holding and keeping human centred.");
//             }

//             break;
//         }

//         // Arm bubble is clear, so normal human approach/hold behaviour can run.
//         twist.angular.z = human_keep_turn;

//         if (!lidar_distance_valid && !(human_distance_m_ > 0.0 && std::isfinite(human_distance_m_)))
//         {
//             twist.linear.x = 0.0;

//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: No valid distance estimate. Turning only to keep human centred.");
//         }
//         else if (at_human_target_distance)
//         {
//             twist.linear.x = 0.0;

//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: Human at target distance %.2f m and 1.5 m arm bubble clear. Ready for interaction.",
//                 estimated_human_range);
//         }
//         else
//         {
//             twist.linear.x = std::clamp(
//                 kHumanForwardGain * distance_error,
//                 -kHumanMaxReverseSpeed,
//                 kHumanMaxForwardSpeed);
//         }

//         if (estimated_human_range < kHumanLidarStopDistance && twist.linear.x > 0.0)
//         {
//             twist.linear.x = 0.0;
//         }

//         RCLCPP_INFO_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             500,
//             "HUMAN_DETECTED: offset=%.2f distance=%.2f arm_clear=%s blocked=%d min_clearance=%.2f cmd linear=%.2f angular=%.2f",
//             human_centre_offset_,
//             estimated_human_range,
//             interaction_bubble_clear ? "yes" : "no",
//             current_bubble.blocked_beams,
//             current_bubble.min_clearance,
//             twist.linear.x,
//             twist.angular.z);

//         break;
//     }

//     default:
//     {
//         RCLCPP_WARN(this->get_logger(), "Unknown navigation state. Returning to SEARCHING.");
//         stopRobot(twist);
//         clearObstacleMarkers();
//         current_state_ = NavigationState::SEARCHING;
//         break;
//     }
//     }

//     current_twist_ = twist;
//     cmd_vel_publisher_->publish(twist);
// }

// void MechelangeloBehaviour::stopRobot(geometry_msgs::msg::Twist &twist)
// {
//     const double linear_step = kStopLinearDecel * kControlPeriodSeconds;
//     const double angular_step = kStopAngularDecel * kControlPeriodSeconds;

//     auto rampTowardZero = [](double value, double max_step)
//     {
//         if (std::fabs(value) <= max_step)
//         {
//             return 0.0;
//         }

//         return value - std::copysign(max_step, value);
//     };

//     twist.linear.x = rampTowardZero(current_twist_.linear.x, linear_step);
//     twist.linear.y = rampTowardZero(current_twist_.linear.y, linear_step);
//     twist.linear.z = rampTowardZero(current_twist_.linear.z, linear_step);

//     twist.angular.x = rampTowardZero(current_twist_.angular.x, angular_step);
//     twist.angular.y = rampTowardZero(current_twist_.angular.y, angular_step);
//     twist.angular.z = rampTowardZero(current_twist_.angular.z, angular_step);
// }

// void MechelangeloBehaviour::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//     const sensor_msgs::msg::LaserScan filtered_scan = filterLaserScan(*msg);

//     latest_scan_ = filtered_scan;
//     latest_segments_ = buildLaserSegments(filtered_scan);

//     filtered_scan_publisher_->publish(filtered_scan);
// }

// void MechelangeloBehaviour::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//     latest_imu_ = *msg;
//     imu_available_ = true;
// }

// void MechelangeloBehaviour::humanDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     if (!msg->data)
//     {
//         return;
//     }

//     RCLCPP_WARN(
//         this->get_logger(),
//         "Manual human detection trigger received. Interrupting autonomous behaviour.");

//     current_state_ = NavigationState::HUMAN_DETECTED;
// }

// void MechelangeloBehaviour::humanTrackingCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
// {
//     if (msg->data.size() < 3)
//     {
//         RCLCPP_WARN_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             2000,
//             "Received invalid /human_tracking message. Expected [detected, centre_offset, distance_m].");
//         return;
//     }

//     human_locked_ = msg->data[0] > 0.5F;
//     human_centre_offset_ = static_cast<double>(msg->data[1]);
//     human_distance_m_ = static_cast<double>(msg->data[2]);
//     last_human_tracking_time_ = this->now();

//     if (human_locked_)
//     {
//         current_state_ = NavigationState::HUMAN_DETECTED;
//     }
// }

// sensor_msgs::msg::LaserScan MechelangeloBehaviour::filterLaserScan(
//     const sensor_msgs::msg::LaserScan &raw_scan)
// {
//     sensor_msgs::msg::LaserScan neighbour_filtered = raw_scan;
//     sensor_msgs::msg::LaserScan final_filtered = raw_scan;

//     if (raw_scan.ranges.empty() || raw_scan.angle_increment == 0.0)
//     {
//         return final_filtered;
//     }

//     const int scan_count = static_cast<int>(raw_scan.ranges.size());
//     std::vector<double> x_points(scan_count, std::numeric_limits<double>::quiet_NaN());
//     std::vector<double> y_points(scan_count, std::numeric_limits<double>::quiet_NaN());
//     std::vector<bool> usable(scan_count, false);

//     // Convert valid polar points to local Cartesian points.
//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = raw_scan.ranges[i];

//         if (!isRangeUsableForFiltering(raw_scan, range))
//         {
//             neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             final_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             continue;
//         }

//         const double angle = raw_scan.angle_min + static_cast<double>(i) * raw_scan.angle_increment;
//         x_points[i] = range * std::cos(angle);
//         y_points[i] = range * std::sin(angle);
//         usable[i] = true;
//     }

//     // Stage 1: suppress isolated points that do not have nearby neighbours.
//     for (int i = 0; i < scan_count; ++i)
//     {
//         if (!usable[i])
//         {
//             neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             continue;
//         }

//         int close_neighbour_count = 0;
//         const int start_index = std::max(0, i - kNoiseNeighbourWindow);
//         const int end_index = std::min(scan_count - 1, i + kNoiseNeighbourWindow);

//         for (int j = start_index; j <= end_index; ++j)
//         {
//             if (j == i || !usable[j])
//             {
//                 continue;
//             }

//             const double distance = std::hypot(x_points[i] - x_points[j], y_points[i] - y_points[j]);

//             if (distance <= kNoiseNeighbourDistance)
//             {
//                 close_neighbour_count++;
//             }
//         }

//         if (close_neighbour_count < kNoiseMinNeighbourCount)
//         {
//             neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//         }
//     }

//     // Stage 2: build LaserProcessing-style segments and only keep points
//     // that belong to a real segment.
//     const std::vector<LaserSegment> accepted_segments = buildLaserSegments(neighbour_filtered);

//     std::fill(final_filtered.ranges.begin(), final_filtered.ranges.end(), std::numeric_limits<float>::infinity());

//     for (const LaserSegment &segment : accepted_segments)
//     {
//         for (int i = segment.start_index; i <= segment.end_index; ++i)
//         {
//             if (i >= 0 && i < scan_count && std::isfinite(neighbour_filtered.ranges[i]))
//             {
//                 final_filtered.ranges[i] = neighbour_filtered.ranges[i];
//             }
//         }
//     }

//     return final_filtered;
// }

// std::vector<LaserSegment> MechelangeloBehaviour::buildLaserSegments(
//     const sensor_msgs::msg::LaserScan &scan) const
// {
//     std::vector<LaserSegment> segments;

//     if (scan.ranges.empty() || scan.angle_increment == 0.0)
//     {
//         return segments;
//     }

//     const int scan_count = static_cast<int>(scan.ranges.size());

//     bool segment_active = false;
//     LaserSegment current_segment;
//     geometry_msgs::msg::Point previous_point;

//     auto finish_segment = [&]()
//     {
//         if (!segment_active)
//         {
//             return;
//         }

//         current_segment.midpoint.x = 0.5 * (current_segment.start_point.x + current_segment.end_point.x);
//         current_segment.midpoint.y = 0.5 * (current_segment.start_point.y + current_segment.end_point.y);
//         current_segment.midpoint.z = 0.0;
//         current_segment.length = std::hypot(
//             current_segment.end_point.x - current_segment.start_point.x,
//             current_segment.end_point.y - current_segment.start_point.y);
//         current_segment.midpoint_angle = std::atan2(
//             current_segment.midpoint.y,
//             current_segment.midpoint.x);

//         if (current_segment.point_count >= kSegmentMinPoints &&
//             current_segment.length >= kSegmentMinLength)
//         {
//             segments.push_back(current_segment);
//         }

//         segment_active = false;
//         current_segment = LaserSegment();
//     };

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = scan.ranges[i];

//         if (!isRangeUsableForFiltering(scan, range))
//         {
//             finish_segment();
//             continue;
//         }

//         const geometry_msgs::msg::Point point = polarToPoint(scan, i);

//         if (!segment_active)
//         {
//             segment_active = true;
//             current_segment = LaserSegment();
//             current_segment.start_index = i;
//             current_segment.end_index = i;
//             current_segment.point_count = 1;
//             current_segment.start_point = point;
//             current_segment.end_point = point;
//             current_segment.min_range = range;
//             previous_point = point;
//             continue;
//         }

//         const double gap = std::hypot(point.x - previous_point.x, point.y - previous_point.y);

//         if (gap <= kSegmentJoinDistance)
//         {
//             current_segment.end_index = i;
//             current_segment.end_point = point;
//             current_segment.point_count++;
//             current_segment.min_range = std::min(current_segment.min_range, range);
//             previous_point = point;
//         }
//         else
//         {
//             finish_segment();

//             segment_active = true;
//             current_segment = LaserSegment();
//             current_segment.start_index = i;
//             current_segment.end_index = i;
//             current_segment.point_count = 1;
//             current_segment.start_point = point;
//             current_segment.end_point = point;
//             current_segment.min_range = range;
//             previous_point = point;
//         }
//     }

//     finish_segment();
//     return segments;
// }

// geometry_msgs::msg::Point MechelangeloBehaviour::polarToPoint(
//     const sensor_msgs::msg::LaserScan &scan,
//     int index) const
// {
//     geometry_msgs::msg::Point point;

//     if (index < 0 || index >= static_cast<int>(scan.ranges.size()))
//     {
//         return point;
//     }

//     const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
//     const double range = scan.ranges[index];

//     point.x = range * std::cos(angle);
//     point.y = range * std::sin(angle);
//     point.z = 0.0;

//     return point;
// }

// bool MechelangeloBehaviour::isRangeUsableForFiltering(
//     const sensor_msgs::msg::LaserScan &scan,
//     double range) const
// {
//     if (!std::isfinite(range))
//     {
//         return false;
//     }

//     if (range <= kMinValidRange)
//     {
//         return false;
//     }

//     if (scan.range_min > 0.0 && range < scan.range_min)
//     {
//         return false;
//     }

//     if (scan.range_max > 0.0 && range > scan.range_max)
//     {
//         return false;
//     }

//     return true;
// }

// bool MechelangeloBehaviour::isRangeValid(double range) const
// {
//     return std::isfinite(range) && range > kMinValidRange;
// }

// double MechelangeloBehaviour::getMinimumRange(double start_angle, double end_angle) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return std::numeric_limits<double>::infinity();
//     }

//     double min_range = std::numeric_limits<double>::infinity();

//     for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//     {
//         const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

//         if (!angleInsideWindow(angle, start_angle, end_angle))
//         {
//             continue;
//         }

//         const double range = latest_scan_.ranges[i];

//         if (isRangeValid(range) && range < min_range)
//         {
//             min_range = range;
//         }
//     }

//     return min_range;
// }

// double MechelangeloBehaviour::getFrontRange() const
// {
//     return getMinimumRange(-kFrontCheckAngle, kFrontCheckAngle);
// }

// bool MechelangeloBehaviour::getLongestRange(double &out_angle, double &out_range) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     const int scan_count = static_cast<int>(latest_scan_.ranges.size());
//     const double angle_step = std::fabs(latest_scan_.angle_increment);

//     if (scan_count <= 0 || angle_step <= 0.0)
//     {
//         return false;
//     }

//     // ------------------------------------------------------
//     // Step 1: classify beams as open or blocked.
//     //
//     // Open means either:
//     //   - infinity: the LiDAR did not hit anything within range, or
//     //   - a finite return farther than the clearance distance.
//     // ------------------------------------------------------
//     std::vector<bool> open_beam(scan_count, false);

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = latest_scan_.ranges[i];

//         if (std::isinf(range))
//         {
//             open_beam[i] = true;
//         }
//         else if (std::isfinite(range) && range >= kDvdOpenClearanceDistance)
//         {
//             open_beam[i] = true;
//         }
//     }

//     auto angleForIndex = [&](int index)
//     {
//         return normaliseAngle(
//             latest_scan_.angle_min + static_cast<double>(index) * latest_scan_.angle_increment);
//     };

//     auto absoluteAngleForIndex = [&](int index)
//     {
//         return std::fabs(angleForIndex(index));
//     };

//     auto rangeForLog = [&](int index)
//     {
//         const double range = latest_scan_.ranges[index];

//         if (std::isinf(range))
//         {
//             return std::numeric_limits<double>::infinity();
//         }

//         if (std::isfinite(range))
//         {
//             return range;
//         }

//         return 0.0;
//     };

//     auto isPreferredBounceAngle = [&](int index)
//     {
//         const double abs_angle = absoluteAngleForIndex(index);
//         return abs_angle >= kDvdPreferredMinTurnAngle &&
//                abs_angle <= kDvdPreferredMaxTurnAngle;
//     };

//     auto isFallbackBounceAngle = [&](int index)
//     {
//         const double abs_angle = absoluteAngleForIndex(index);
//         return abs_angle >= kDvdAvoidFrontAngle &&
//                abs_angle <= kDvdAvoidReverseAngle;
//     };

//     // ------------------------------------------------------
//     // Step 2: extract open sectors and collect safe candidate beams.
//     //
//     // Preferred candidates are side-bounce angles, roughly +/-55 to +/-150 deg.
//     // General fallback candidates simply avoid the front wall and exact reverse.
//     // Left and right candidates are stored separately so one long side of the
//     // room does not dominate every decision.
//     // ------------------------------------------------------
//     std::vector<int> preferred_left_candidates;
//     std::vector<int> preferred_right_candidates;
//     std::vector<int> fallback_left_candidates;
//     std::vector<int> fallback_right_candidates;
//     std::vector<int> fallback_all_candidates;

//     int accepted_sector_count = 0;
//     int widest_sector_count = 0;

//     auto collectCandidatesFromSector = [&](int start_index, int count)
//     {
//         if (count <= 0)
//         {
//             return;
//         }

//         const double sector_width = static_cast<double>(count) * angle_step;
//         widest_sector_count = std::max(widest_sector_count, count);

//         if (sector_width < kDvdMinSectorWidth)
//         {
//             return;
//         }

//         accepted_sector_count++;

//         int edge_margin_count = static_cast<int>(std::ceil(kDvdSectorEdgeMargin / angle_step));

//         // Never let the edge margin remove the entire sector.
//         edge_margin_count = std::min(edge_margin_count, std::max(0, (count - 1) / 2));

//         for (int offset = edge_margin_count; offset < count - edge_margin_count; ++offset)
//         {
//             const int index = (start_index + offset) % scan_count;
//             const double angle = angleForIndex(index);

//             if (isPreferredBounceAngle(index))
//             {
//                 if (angle >= 0.0)
//                 {
//                     preferred_left_candidates.push_back(index);
//                 }
//                 else
//                 {
//                     preferred_right_candidates.push_back(index);
//                 }
//             }

//             if (isFallbackBounceAngle(index))
//             {
//                 fallback_all_candidates.push_back(index);

//                 if (angle >= 0.0)
//                 {
//                     fallback_left_candidates.push_back(index);
//                 }
//                 else
//                 {
//                     fallback_right_candidates.push_back(index);
//                 }
//             }
//         }
//     };

//     const bool all_open = std::all_of(open_beam.begin(), open_beam.end(),
//         [](bool value) { return value; });

//     if (all_open)
//     {
//         collectCandidatesFromSector(0, scan_count);
//     }
//     else
//     {
//         for (int start_index = 0; start_index < scan_count; ++start_index)
//         {
//             if (!open_beam[start_index])
//             {
//                 continue;
//             }

//             const int previous_index = (start_index - 1 + scan_count) % scan_count;
//             if (open_beam[previous_index])
//             {
//                 continue;
//             }

//             int count = 0;
//             while (count < scan_count && open_beam[(start_index + count) % scan_count])
//             {
//                 count++;
//             }

//             collectCandidatesFromSector(start_index, count);
//         }
//     }

//     // ------------------------------------------------------
//     // Step 3: choose a random safe bounce angle.
//     //
//     // Preferred behaviour:
//     //   - use the side-bounce band if possible;
//     //   - choose left/right with a 50/50 coin flip when both are available;
//     //   - choose a random beam inside that side's safe candidate list.
//     // This produces a DVD-screensaver style movement instead of always going
//     // down the longest wall/side of the room.
//     // ------------------------------------------------------
//     static thread_local std::mt19937 rng(std::random_device{}());

//     auto chooseFromCandidates = [&](const std::vector<int> &candidates)
//     {
//         std::uniform_int_distribution<int> index_dist(
//             0, static_cast<int>(candidates.size()) - 1);
//         return candidates[index_dist(rng)];
//     };

//     auto chooseBalancedSide = [&](const std::vector<int> &left_candidates,
//                                   const std::vector<int> &right_candidates,
//                                   bool &used_left_side,
//                                   bool &used_right_side)
//     {
//         used_left_side = false;
//         used_right_side = false;

//         if (!left_candidates.empty() && !right_candidates.empty())
//         {
//             std::uniform_int_distribution<int> side_dist(0, 1);

//             if (side_dist(rng) == 0)
//             {
//                 used_left_side = true;
//                 return chooseFromCandidates(left_candidates);
//             }

//             used_right_side = true;
//             return chooseFromCandidates(right_candidates);
//         }

//         if (!left_candidates.empty())
//         {
//             used_left_side = true;
//             return chooseFromCandidates(left_candidates);
//         }

//         used_right_side = true;
//         return chooseFromCandidates(right_candidates);
//     };

//     int selected_index = -1;
//     bool used_preferred_bounce_band = false;
//     bool used_left_side = false;
//     bool used_right_side = false;

//     if (!preferred_left_candidates.empty() || !preferred_right_candidates.empty())
//     {
//         selected_index = chooseBalancedSide(
//             preferred_left_candidates,
//             preferred_right_candidates,
//             used_left_side,
//             used_right_side);
//         used_preferred_bounce_band = true;
//     }
//     else if (!fallback_left_candidates.empty() || !fallback_right_candidates.empty())
//     {
//         selected_index = chooseBalancedSide(
//             fallback_left_candidates,
//             fallback_right_candidates,
//             used_left_side,
//             used_right_side);
//         used_preferred_bounce_band = false;
//     }
//     else if (!fallback_all_candidates.empty())
//     {
//         selected_index = chooseFromCandidates(fallback_all_candidates);
//         used_left_side = angleForIndex(selected_index) >= 0.0;
//         used_right_side = !used_left_side;
//         used_preferred_bounce_band = false;
//     }

//     if (selected_index >= 0)
//     {
//         out_angle = angleForIndex(selected_index);
//         out_range = rangeForLog(selected_index);

//         {
//             std::ofstream log("/tmp/mechelangelo_dvd_bounce_debug.txt");
//             log << std::fixed << std::setprecision(4);
//             log << "=== DVD Bounce Heading Selection ===\n";
//             log << "Open beam rule: inf OR range >= " << kDvdOpenClearanceDistance << " m\n";
//             log << "Minimum accepted sector width: " << kDvdMinSectorWidth * 180.0 / M_PI << " deg\n";
//             log << "Sector edge margin: " << kDvdSectorEdgeMargin * 180.0 / M_PI << " deg\n";
//             log << "Preferred bounce band: +/-"
//                 << kDvdPreferredMinTurnAngle * 180.0 / M_PI << " to +/-"
//                 << kDvdPreferredMaxTurnAngle * 180.0 / M_PI << " deg\n";
//             log << "Fallback bounce band: +/-"
//                 << kDvdAvoidFrontAngle * 180.0 / M_PI << " to +/-"
//                 << kDvdAvoidReverseAngle * 180.0 / M_PI << " deg\n\n";

//             log << "Accepted open sectors: " << accepted_sector_count << "\n";
//             log << "Widest open sector beams: " << widest_sector_count << "\n";
//             log << "Preferred left candidates: " << preferred_left_candidates.size() << "\n";
//             log << "Preferred right candidates: " << preferred_right_candidates.size() << "\n";
//             log << "Fallback candidates: " << fallback_all_candidates.size() << "\n\n";

//             log << "=== Result ===\n";
//             log << "Selected behaviour: "
//                 << (used_preferred_bounce_band ? "preferred random side-bounce" : "fallback random safe bounce")
//                 << "\n";
//             log << "Selected side: "
//                 << (used_left_side ? "left/positive" : (used_right_side ? "right/negative" : "unknown"))
//                 << "\n";
//             log << "Selected index: " << selected_index << "\n";
//             log << "Selected angle: " << out_angle * 180.0 / M_PI << " deg\n";
//             log << "Representative range: " << out_range << " m\n\n";

//             log << "=== Laser Scan Values ===\n";
//             for (int i = 0; i < scan_count; ++i)
//             {
//                 const double angle_deg = angleForIndex(i) * 180.0 / M_PI;
//                 log << "  [" << std::setw(4) << i << "]  angle="
//                     << std::setw(9) << angle_deg << " deg  range="
//                     << latest_scan_.ranges[i] << " m  open="
//                     << (open_beam[i] ? "yes" : "no") << "  preferred="
//                     << (isPreferredBounceAngle(i) ? "yes" : "no") << "\n";
//             }
//         }

//         return true;
//     }

//     // ------------------------------------------------------
//     // Step 4: fallback to the old longest finite scan if the room is too
//     // cluttered for a safe random bounce heading.
//     // ------------------------------------------------------
//     double max_finite_range = 0.0;
//     int max_finite_index = -1;

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = latest_scan_.ranges[i];

//         if (isRangeValid(range) && range > max_finite_range)
//         {
//             max_finite_range = range;
//             max_finite_index = i;
//         }
//     }

//     if (max_finite_index < 0)
//     {
//         return false;
//     }

//     const double tied_range_threshold =
//         std::max(kMinValidRange, max_finite_range - kLongestRangeTieTolerance);

//     std::vector<bool> near_longest(scan_count, false);

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = latest_scan_.ranges[i];
//         near_longest[i] = isRangeValid(range) && range >= tied_range_threshold;
//     }

//     int best_start_index = max_finite_index;
//     int best_count = 0;

//     for (int start_index = 0; start_index < scan_count; ++start_index)
//     {
//         if (!near_longest[start_index])
//         {
//             continue;
//         }

//         const int previous_index = (start_index - 1 + scan_count) % scan_count;
//         if (near_longest[previous_index])
//         {
//             continue;
//         }

//         int count = 0;
//         while (count < scan_count && near_longest[(start_index + count) % scan_count])
//         {
//             count++;
//         }

//         if (count > best_count)
//         {
//             best_count = count;
//             best_start_index = start_index;
//         }
//     }

//     if (best_count == 0)
//     {
//         best_count = 1;
//         best_start_index = max_finite_index;
//     }

//     const double best_mid_index = std::fmod(
//         static_cast<double>(best_start_index) +
//             0.5 * static_cast<double>(std::max(0, best_count - 1)),
//         static_cast<double>(scan_count));

//     out_angle = normaliseAngle(latest_scan_.angle_min + best_mid_index * latest_scan_.angle_increment);
//     out_range = max_finite_range;

//     {
//         std::ofstream log("/tmp/mechelangelo_dvd_bounce_debug.txt");
//         log << std::fixed << std::setprecision(4);
//         log << "=== DVD Bounce Heading Selection ===\n";
//         log << "Selected behaviour: fallback longest finite scan\n";
//         log << "Longest finite range: " << out_range << " m\n";
//         log << "Selected angle: " << out_angle * 180.0 / M_PI << " deg\n";
//     }

//     return true;
// }

// int MechelangeloBehaviour::angleToIndex(double angle_rad) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return -1;
//     }

//     double capped_angle = angle_rad;

//     if (capped_angle < latest_scan_.angle_min)
//     {
//         capped_angle = latest_scan_.angle_min;
//     }

//     if (capped_angle > latest_scan_.angle_max)
//     {
//         capped_angle = latest_scan_.angle_max;
//     }

//     int index = static_cast<int>(
//         std::round((capped_angle - latest_scan_.angle_min) / latest_scan_.angle_increment));

//     index = std::clamp(index, 0, static_cast<int>(latest_scan_.ranges.size()) - 1);
//     return index;
// }

// double MechelangeloBehaviour::normaliseAngle(double angle_rad) const
// {
//     while (angle_rad > M_PI)
//     {
//         angle_rad -= 2.0 * M_PI;
//     }

//     while (angle_rad < -M_PI)
//     {
//         angle_rad += 2.0 * M_PI;
//     }

//     return angle_rad;
// }

// bool MechelangeloBehaviour::angleInsideWindow(
//     double angle_rad,
//     double start_angle,
//     double end_angle) const
// {
//     const double angle = normaliseAngle(angle_rad);
//     const double start = normaliseAngle(start_angle);
//     const double end = normaliseAngle(end_angle);

//     if (start <= end)
//     {
//         return angle >= start && angle <= end;
//     }

//     return angle >= start || angle <= end;
// }

// bool MechelangeloBehaviour::segmentOverlapsAngleWindow(
//     const LaserSegment &segment,
//     double start_angle,
//     double end_angle) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     for (int i = segment.start_index; i <= segment.end_index; ++i)
//     {
//         if (i < 0 || i >= static_cast<int>(latest_scan_.ranges.size()))
//         {
//             continue;
//         }

//         const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

//         if (angleInsideWindow(angle, start_angle, end_angle))
//         {
//             return true;
//         }
//     }

//     return false;
// }

// bool MechelangeloBehaviour::findBlockingObstaclesInFront(
//     std::vector<LaserSegment> &blocking_segments) const
// {
//     blocking_segments.clear();

//     for (const LaserSegment &segment : latest_segments_)
//     {
//         if (segment.min_range <= stop_distance_m_ &&
//             segmentOverlapsAngleWindow(segment, -kFrontCheckAngle, kFrontCheckAngle))
//         {
//             blocking_segments.push_back(segment);
//         }
//     }

//     return !blocking_segments.empty();
// }

// void MechelangeloBehaviour::publishObstacleMarkers(
//     const std::vector<LaserSegment> &blocking_segments)
// {
//     visualization_msgs::msg::MarkerArray marker_array;

//     visualization_msgs::msg::Marker clear_marker;
//     clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
//     clear_marker.header.stamp = this->get_clock()->now();
//     clear_marker.ns = "behaviour_blocking_obstacles";
//     clear_marker.id = 0;
//     clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//     marker_array.markers.push_back(clear_marker);

//     int marker_id = 1;

//     for (const LaserSegment &segment : blocking_segments)
//     {
//         visualization_msgs::msg::Marker marker;
//         marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
//         marker.header.stamp = this->get_clock()->now();
//         marker.ns = "behaviour_blocking_obstacles";
//         marker.id = marker_id++;
//         marker.type = visualization_msgs::msg::Marker::CYLINDER;
//         marker.action = visualization_msgs::msg::Marker::ADD;

//         marker.pose.position.x = segment.midpoint.x;
//         marker.pose.position.y = segment.midpoint.y;
//         marker.pose.position.z = 0.15;
//         marker.pose.orientation.x = 0.0;
//         marker.pose.orientation.y = 0.0;
//         marker.pose.orientation.z = 0.0;
//         marker.pose.orientation.w = 1.0;

//         // Make marker size scale slightly with the observed segment length.
//         const double marker_width = std::clamp(segment.length + 0.15, 0.20, 0.80);
//         marker.scale.x = marker_width;
//         marker.scale.y = marker_width;
//         marker.scale.z = 0.30;

//         // Red/orange transparent marker for blocking obstacle.
//         marker.color.a = 0.75F;
//         marker.color.r = 1.0F;
//         marker.color.g = 0.15F;
//         marker.color.b = 0.0F;

//         marker.lifetime.sec = 0;
//         marker.lifetime.nanosec = 400000000; // 0.4 s

//         marker_array.markers.push_back(marker);
//     }

//     obstacle_marker_publisher_->publish(marker_array);
// }

// void MechelangeloBehaviour::clearObstacleMarkers()
// {
//     if (!obstacle_marker_publisher_)
//     {
//         return;
//     }

//     visualization_msgs::msg::MarkerArray marker_array;
//     visualization_msgs::msg::Marker clear_marker;
//     clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
//     clear_marker.header.stamp = this->get_clock()->now();
//     clear_marker.ns = "behaviour_blocking_obstacles";
//     clear_marker.id = 0;
//     clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//     marker_array.markers.push_back(clear_marker);
//     obstacle_marker_publisher_->publish(marker_array);
// }

// void MechelangeloBehaviour::longestLaserScan()
// {
//     double longest_angle = 0.0;
//     double longest_range = 0.0;

//     if (!getLongestRange(longest_angle, longest_range))
//     {
//         RCLCPP_WARN(
//             this->get_logger(),
//             "No valid filtered laser scan data available for longest scan calculation.");
//         return;
//     }

//     RCLCPP_INFO(
//         this->get_logger(),
//         "Selected exploration heading: Representative range = %.2f m at Angle = %.2f degrees",
//         longest_range,
//         longest_angle * 180.0 / M_PI);
// }

// double MechelangeloBehaviour::getHumanLidarRange(double centre_offset) const
// {
//     const double estimated_human_angle = -centre_offset * kCameraHorizontalFov;
//     const double start_angle = estimated_human_angle - kHumanLidarWindow;
//     const double end_angle = estimated_human_angle + kHumanLidarWindow;

//     return getMinimumRange(start_angle, end_angle);
// }

// void MechelangeloBehaviour::captureSafetyZoneBaseline()
// {
//     safety_zone_baseline_scan_ = latest_scan_;
//     safety_zone_baseline_captured_ = true;
//     RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Filtered background baseline captured.");
// }

// bool MechelangeloBehaviour::isSafetyZoneViolated(double human_bearing_rad) const
// {
//     if (!safety_zone_baseline_captured_)
//     {
//         return false;
//     }

//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     if (safety_zone_baseline_scan_.ranges.size() != latest_scan_.ranges.size())
//     {
//         return false;
//     }

//     for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//     {
//         const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
//         const double angle_diff = normaliseAngle(angle - human_bearing_rad);

//         // Exclude the window around the tracked human so they do not self-trigger.
//         if (std::fabs(angle_diff) <= kSafetyZoneHumanExclusionAngle)
//         {
//             continue;
//         }

//         const double current_range = latest_scan_.ranges[i];

//         if (!std::isfinite(current_range) || current_range <= kMinValidRange)
//         {
//             continue;
//         }

//         if (current_range >= kSafetyZoneRadius)
//         {
//             continue;
//         }

//         const double baseline_range = std::isfinite(safety_zone_baseline_scan_.ranges[i])
//             ? safety_zone_baseline_scan_.ranges[i]
//             : kSafetyZoneRadius;

//         if (current_range < (baseline_range - kSafetyZoneIntruderThreshold))
//         {
//             return true;
//         }
//     }

//     return false;
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<MechelangeloBehaviour>();

//     // true = simulation mode
//     // false = real robot mode
//     node->run(true);

//     rclcpp::shutdown();
//     return 0;
// }

// /////////////////////////////////////////////////////////////////////////
// /// DVD bounce + human interaction timer/statistics test

// #include "behaviour.hpp"

// #include <algorithm>
// #include <chrono>
// #include <cmath>
// #include <fstream>
// #include <iomanip>
// #include <limits>
// #include <functional>
// #include <memory>
// #include <random>
// #include <string>
// #include <vector>

// using namespace std::chrono_literals;

// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif

// // ------------------------------------------------------
// // Behaviour constants
// // ------------------------------------------------------

// // Control loop runs every 100 ms.
// static constexpr double kControlPeriodSeconds = 0.1;

// // Movement tuning.
// static constexpr double kForwardSpeed = 0.26;       // m/s
// static constexpr double kTurnSpeed = 0.6;           // rad/s
// static constexpr double kAngleGain = 0.8;           // proportional turning gain
// static constexpr double kAlignmentTolerance = 0.10; // radians, about 5.7 degrees

// // Smooth commanded stops so the physical base does not snap from motion to zero
// // in one control tick. At the 100 ms control period, these remove roughly
// // 0.04 m/s and 0.12 rad/s from the command each loop.
// static constexpr double kStopLinearDecel = 0.4;  // m/s^2
// static constexpr double kStopAngularDecel = 1.2; // rad/s^2

// // Stop this far before a real obstacle/wall.
// // Loaded from the ROS parameter 'stop_distance_m'.
// // Default: 1.5 m (simulation). Physical robot: set to 0.75 in the launch file.

// // 30 loops x 0.1 s = 3 seconds.
// static constexpr int kStopDurationLoops = 30;

// // Ignore returns too close to the robot body / lidar blind spot.
// static constexpr double kMinValidRange = 0.5; // m

// // Front scan window used while moving forward.
// static constexpr double kFrontCheckAngle = 30 * M_PI / 180.0; // +/- 30 degrees

// // When several neighbouring beams are effectively tied for the longest
// // distance, steer toward the middle of that opening instead of whichever beam
// // happens to appear first in the scan array.
// static constexpr double kLongestRangeTieTolerance = 0.05; // m

// // ------------------------------------------------------
// // DVD-style bounce exploration tuning
// // ------------------------------------------------------
// // The autonomous gallery behaviour is intended to move like a DVD screensaver:
// // drive straight until the front is blocked, stop at a safe distance, then pick
// // a new safe side-bounce angle instead of always chasing the longest wall.
// //
// // A beam is considered open if it is infinity OR farther than this clearance.
// // Infinity is useful in large rooms because it means the LiDAR did not hit
// // anything within range.
// static constexpr double kDvdOpenClearanceDistance = 4.5; // m

// // Ignore very narrow open gaps. This prevents the robot from aiming through
// // thin laser cracks between obstacles.
// static constexpr double kDvdMinSectorWidth = 18.0 * M_PI / 180.0; // radians

// // Trim candidate angles away from the edge of an open sector.
// static constexpr double kDvdSectorEdgeMargin = 8.0 * M_PI / 180.0; // radians

// // Preferred bounce band after the robot stops at a wall.
// // These angles are relative to the robot's current forward direction:
// //   0 deg   = back into the wall/obstacle it just stopped for
// //   +/-90   = side-bounce
// //   +/-180  = drive exactly back along the previous path
// // The robot randomly chooses inside this safe side band when possible.
// static constexpr double kDvdPreferredMinTurnAngle = 55.0 * M_PI / 180.0;  // radians
// static constexpr double kDvdPreferredMaxTurnAngle = 150.0 * M_PI / 180.0; // radians

// // Fallback band used if the preferred side-bounce band has no safe candidates.
// // This still avoids the front wall and avoids exact reverse.
// static constexpr double kDvdAvoidFrontAngle = 35.0 * M_PI / 180.0;  // radians
// static constexpr double kDvdAvoidReverseAngle = 165.0 * M_PI / 180.0; // radians

// // ------------------------------------------------------
// // LaserScan noise suppression constants
// // ------------------------------------------------------
// // The filter is based on the previous LaserProcessing::countSegments()
// // approach: valid objects form segments of neighbouring points. Random
// // dots are usually isolated or only one/two points, so they get replaced
// // with infinity and will not stop the robot.

// // Stage 1: local neighbour test.
// static constexpr int kNoiseNeighbourWindow = 4;          // check +/- 4 beams
// static constexpr int kNoiseMinNeighbourCount = 2;        // require at least 2 close neighbours
// static constexpr double kNoiseNeighbourDistance = 0.22;  // m in local XY space

// // Stage 2: segment extraction test.
// static constexpr double kSegmentJoinDistance = 0.18;     // m max gap between consecutive points
// static constexpr int kSegmentMinPoints = 4;              // reject tiny speckle clusters
// static constexpr double kSegmentMinLength = 0.05;        // m reject near-zero length segments

// // ------------------------------------------------------
// // Human tracking tuning
// // ------------------------------------------------------
// // /human_tracking message format:
// // data[0] = detected, data[1] = centre_offset, data[2] = distance_m
// // centre_offset is normalised image offset from centre: -0.5 left, 0 centre, +0.5 right.
// static constexpr double kHumanTargetDistance = 1.5;     // m
// static constexpr double kHumanDistanceTolerance = 0.15; // m
// static constexpr double kHumanMaxForwardSpeed = 0.16;   // m/s, slower approach helps keep person in camera
// static constexpr double kHumanMaxReverseSpeed = 0.12;   // m/s
// static constexpr double kHumanMaxTurnSpeed = 0.50;      // rad/s, gentler human tracking so the camera does not swing off target
// static constexpr double kHumanTurnGain = 1.8;           // image offset to angular speed
// static constexpr double kHumanForwardGain = 0.35;       // distance error to linear speed
// static constexpr double kHumanCentreDeadZone = 0.06;    // normalised image width
// static constexpr double kHumanLostTimeout = 4.0;        // seconds, allow longer brief camera loss before returning to exploration
// static constexpr double kHumanRecoveryTurnSpeed = 0.25;   // rad/s, gentle reacquire turn using last known offset
// static constexpr double kHumanRecoveryCreepSpeed = 0.00;  // m/s, keep zero while reacquiring to avoid blind motion

// // LiDAR validation for human distance.
// static constexpr double kCameraHorizontalFov = 60.0 * M_PI / 180.0;
// static constexpr double kHumanLidarWindow = 10.0 * M_PI / 180.0;
// static constexpr double kLidarCameraMaxDisagreement = 0.4;
// static constexpr double kHumanLidarStopDistance = 1.65;
// static constexpr double kHumanLidarStopTolerance = 0.20;

// // Once the robot reaches a usable interaction pose, hold it instead of
// // continuing to creep forward or dropping straight back into DVD exploration
// // when the detector flickers for a moment.
// static constexpr double kHumanInteractionHoldTimeout = 5.0;       // seconds to hold pose through brief detector loss
// static constexpr double kHumanInteractionHoldMaxOffset = 0.18;    // human must be reasonably centred to latch interaction hold
// static constexpr double kHumanRecoveryRotateOffsetThreshold = 0.15; // do not rotate during recovery if human was basically centred
// static constexpr double kHumanInteractionHoldRangeSlack = 0.25;   // extra distance slack while staying latched

// // Human interaction session timing. Once a valid interaction pose is reached,
// // hold interaction for this long, then return to DVD exploration and ignore
// // camera-triggered human interrupts for a short cooldown period.
// static constexpr double kHumanInteractionDurationSeconds = 30.0;
// static constexpr double kHumanDetectionCooldownSeconds = 10.0;

// // Safety zone.
// static constexpr double kSafetyZoneRadius = 1.5;
// static constexpr double kSafetyZoneIntruderThreshold = 0.3;
// static constexpr double kSafetyZoneHumanExclusionAngle = 25.0 * M_PI / 180.0;

// // During HUMAN_DETECTED, the robot is allowed to have a closer wall behind it
// // because the arms and human interaction are mainly front/side constrained.
// // This avoids false blocking when the base turns imperfectly and the rear of
// // the LiDAR/base drifts slightly toward the wall it just stopped near.
// static constexpr double kHumanRearSafetyRadius = 1.0; // m, rear-only during human approach/interaction
// static constexpr double kHumanRearSectorHalfAngle = 45.0 * M_PI / 180.0; // rear cone around +/-180 deg

// // ------------------------------------------------------
// // Human interaction repositioning tuning
// // ------------------------------------------------------
// // The robot needs a clear space around itself before arm interaction.
// // If the human is visible but the 1.5 m arm bubble is blocked by a wall or
// // obstacle, the robot does not drive straight at the human. Instead it samples
// // short forward/arc moves and chooses the one predicted to clear the bubble
// // while still keeping the human in view.
// static constexpr double kInteractionBubbleRadius = 1.5;                 // m, arm movement clearance around robot
// static constexpr double kInteractionBubbleSafetyMargin = 0.10;          // m, extra buffer added to the bubble check
// static constexpr double kInteractionHumanExclusionAngle = 25.0 * M_PI / 180.0; // ignore tracked human cone
// static constexpr int kInteractionAllowedBlockedBeams = 3;               // tolerate a few filtered/noisy beams
// static constexpr double kInteractionRepositionLookahead = 0.45;         // m, predicted short move distance
// static constexpr double kInteractionRepositionSpeed = 0.07;             // m/s, slow reposition creep
// static constexpr double kInteractionRepositionTurnGain = 0.65;          // rad/s per rad target heading, deliberately gentle
// static constexpr double kInteractionRepositionMaxAngularSpeed = 0.30;   // rad/s, avoid swinging the camera off the human
// static constexpr double kInteractionPathHalfWidth = 0.45;               // m, collision corridor half-width
// static constexpr double kInteractionPathForwardBuffer = 0.25;           // m, extra forward collision buffer
// static constexpr double kInteractionMaxCandidateAngle = 35.0 * M_PI / 180.0;
// static constexpr double kInteractionRecenterFirstOffset = 0.38;         // if human is near camera edge, re-centre before repositioning
// static constexpr double kInteractionViewLossOffset = 0.48;              // heavy penalty if predicted offset approaches camera edge
// static constexpr double kInteractionParallelBiasAngle = 25.0 * M_PI / 180.0; // prefer shallow wall-parallel arcs

// // Last confirmed visual human observation. Kept file-scope so this patch does not
// // require behaviour.hpp changes. Used to pause/reacquire instead of immediately
// // dropping back to DVD exploration when the camera briefly loses the person.
// static bool g_last_visible_human_valid = false;
// static double g_last_visible_human_centre_offset = 0.0;
// static double g_last_visible_human_distance_m = -1.0;
// static rclcpp::Time g_last_visible_human_time;

// // File-scope latch so this test patch does not require behaviour.hpp changes.
// // This becomes active once the robot is centred, at a usable interaction range,
// // and has a clear 1.5 m arm bubble.
// static bool g_interaction_hold_active = false;
// static rclcpp::Time g_interaction_hold_time;
// static double g_interaction_hold_range_m = -1.0;
// static double g_interaction_hold_offset = 0.0;

// // File-scope interaction session/cooldown and long-run stats. Keeping these
// // outside the class avoids requiring behaviour.hpp changes for this test.
// static bool g_interaction_session_active = false;
// static rclcpp::Time g_interaction_session_start_time;
// static bool g_human_detection_cooldown_active = false;
// static rclcpp::Time g_human_detection_cooldown_start_time;

// static int g_dvd_heading_selection_count = 0;
// static int g_human_detection_accepted_count = 0;
// static int g_human_detection_ignored_cooldown_count = 0;
// static int g_human_interaction_success_count = 0;
// static int g_human_interaction_completed_count = 0;
// static int g_human_abandoned_before_interaction_count = 0;
// static int g_human_abandoned_after_interaction_count = 0;

// static constexpr const char *kHumanStatsLogPath =
//     "/tmp/mechelangelo_human_interaction_stats.txt";

// static double normaliseAngleForFileScope(double angle_rad)
// {
//     while (angle_rad > M_PI)
//     {
//         angle_rad -= 2.0 * M_PI;
//     }

//     while (angle_rad < -M_PI)
//     {
//         angle_rad += 2.0 * M_PI;
//     }

//     return angle_rad;
// }

// static bool isRearBeamAngle(double angle_rad)
// {
//     const double angle = normaliseAngleForFileScope(angle_rad);
//     return std::fabs(angle) >= (M_PI - kHumanRearSectorHalfAngle);
// }

// static double humanModeSafetyRadiusForAngle(double angle_rad)
// {
//     return isRearBeamAngle(angle_rad) ? kHumanRearSafetyRadius : kSafetyZoneRadius;
// }

// static double humanModeInteractionBubbleRadiusForAngle(double angle_rad)
// {
//     return isRearBeamAngle(angle_rad)
//         ? kHumanRearSafetyRadius
//         : (kInteractionBubbleRadius + kInteractionBubbleSafetyMargin);
// }

// static bool humanDetectionCooldownActive(const rclcpp::Time &now)
// {
//     if (!g_human_detection_cooldown_active)
//     {
//         return false;
//     }

//     return (now - g_human_detection_cooldown_start_time).seconds() <
//         kHumanDetectionCooldownSeconds;
// }

// static double humanDetectionCooldownRemaining(const rclcpp::Time &now)
// {
//     if (!g_human_detection_cooldown_active)
//     {
//         return 0.0;
//     }

//     const double elapsed = (now - g_human_detection_cooldown_start_time).seconds();
//     return std::max(0.0, kHumanDetectionCooldownSeconds - elapsed);
// }

// static void writeHumanInteractionStatsLog(
//     const std::string &event,
//     const rclcpp::Time &now)
// {
//     std::ofstream log(kHumanStatsLogPath);
//     log << std::fixed << std::setprecision(3);
//     log << "=== Mechelangelo Human Interaction Stats ===\n";
//     log << "Last event: " << event << "\n";
//     log << "ROS time: " << now.seconds() << " s\n\n";

//     log << "DVD exploration headings selected: " << g_dvd_heading_selection_count << "\n";
//     log << "Human detections accepted: " << g_human_detection_accepted_count << "\n";
//     log << "Human detections ignored during cooldown: "
//         << g_human_detection_ignored_cooldown_count << "\n\n";

//     log << "Successful entries into interaction state: "
//         << g_human_interaction_success_count << "\n";
//     log << "Completed 30 s interactions: "
//         << g_human_interaction_completed_count << "\n";
//     log << "Abandoned before reaching interaction: "
//         << g_human_abandoned_before_interaction_count << "\n";
//     log << "Abandoned after reaching interaction: "
//         << g_human_abandoned_after_interaction_count << "\n\n";

//     log << "Interaction session active: "
//         << (g_interaction_session_active ? "yes" : "no") << "\n";
//     if (g_interaction_session_active)
//     {
//         log << "Interaction elapsed: "
//             << (now - g_interaction_session_start_time).seconds() << " / "
//             << kHumanInteractionDurationSeconds << " s\n";
//     }

//     log << "Human detection cooldown active: "
//         << (humanDetectionCooldownActive(now) ? "yes" : "no") << "\n";
//     if (humanDetectionCooldownActive(now))
//     {
//         log << "Cooldown remaining: "
//             << humanDetectionCooldownRemaining(now) << " s\n";
//     }

//     log << "\nRear safety radius during HUMAN_DETECTED: "
//         << kHumanRearSafetyRadius << " m for rear +/-"
//         << kHumanRearSectorHalfAngle * 180.0 / M_PI << " deg around 180 deg\n";
//     log << "Front/side safety radius during HUMAN_DETECTED: "
//         << kSafetyZoneRadius << " m\n";
// }

// MechelangeloBehaviour::MechelangeloBehaviour()
// : Node("mechelangelo_behaviour"),
//   human_locked_(false),
//   human_centre_offset_(0.0),
//   human_distance_m_(-1.0),
//   blind_autonomous_active_(false),
//   safety_zone_violated_(false),
//   safety_zone_baseline_captured_(false),
//   current_state_(NavigationState::SEARCHING),
//   target_angle_(0.0),
//   target_range_(0.0),
//   stop_distance_m_(1.5),
//   stop_counter_(0),
//   imu_available_(false),
//   align_start_yaw_(0.0),
//   align_yaw_initialised_(false),
//   random_engine_(std::random_device{}()),
//   turn_dist_(-1.0, 1.0)
// {
//     this->declare_parameter("stop_distance_m", 1.5);
//     stop_distance_m_ = this->get_parameter("stop_distance_m").as_double();

//     RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_m_);

//     laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         rclcpp::SensorDataQoS(),
//         std::bind(&MechelangeloBehaviour::laserScanCallback, this, std::placeholders::_1));

//     imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
//         "/imu",
//         rclcpp::SensorDataQoS(),
//         std::bind(&MechelangeloBehaviour::imuCallback, this, std::placeholders::_1));

//     cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
//         "/cmd_vel",
//         10);

//     filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
//         "/scan_filtered",
//         rclcpp::SensorDataQoS());

//     obstacle_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//         "/behaviour_obstacle_markers",
//         10);

//     human_detected_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
//         "/human_detected",
//         10,
//         std::bind(&MechelangeloBehaviour::humanDetectedCallback, this, std::placeholders::_1));

//     human_tracking_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
//         "/human_tracking",
//         10,
//         std::bind(&MechelangeloBehaviour::humanTrackingCallback, this, std::placeholders::_1));

//     last_human_tracking_time_ = this->now();
//     g_last_visible_human_time = this->now();
//     g_last_visible_human_valid = false;
//     g_last_visible_human_centre_offset = 0.0;
//     g_last_visible_human_distance_m = -1.0;

//     control_timer_ = this->create_wall_timer(
//         100ms,
//         std::bind(&MechelangeloBehaviour::controlLoop, this));

//     RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been started.");
// }

// MechelangeloBehaviour::~MechelangeloBehaviour()
// {
//     RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been stopped.");
// }

// void MechelangeloBehaviour::run(bool sim_mode)
// {
//     RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node is running.");

//     if (sim_mode)
//     {
//         RCLCPP_INFO(this->get_logger(), "Running in simulation mode.");
//     }
//     else
//     {
//         RCLCPP_INFO(this->get_logger(), "Running in real robot mode.");
//     }

//     blindAutonomous();
//     rclcpp::spin(shared_from_this());
// }

// void MechelangeloBehaviour::blindAutonomous()
// {
//     RCLCPP_INFO(this->get_logger(), "Executing blind autonomous behaviour.");

//     blind_autonomous_active_ = true;
//     safety_zone_violated_ = false;
//     safety_zone_baseline_captured_ = false;
//     current_state_ = NavigationState::SEARCHING;
//     target_angle_ = 0.0;
//     target_range_ = 0.0;
//     stop_counter_ = 0;
//     align_yaw_initialised_ = false;
//     g_interaction_hold_active = false;
//     clearObstacleMarkers();
// }

// void MechelangeloBehaviour::mappedAutonomous()
// {
//     RCLCPP_INFO(this->get_logger(), "Executing mapped autonomous behaviour.");
// }

// void MechelangeloBehaviour::controlLoop()
// {
//     if (!blind_autonomous_active_)
//     {
//         return;
//     }

//     geometry_msgs::msg::Twist twist;

//     // Safety: wait until valid LaserScan data exists.
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         RCLCPP_WARN_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             2000,
//             "Waiting for valid filtered LaserScan data...");

//         stopRobot(twist);
//         current_twist_ = twist;
//         cmd_vel_publisher_->publish(twist);
//         return;
//     }

//     switch (current_state_)
//     {
//     case NavigationState::SEARCHING:
//     {
//         stopRobot(twist);
//         clearObstacleMarkers();

//         double longest_angle = 0.0;
//         double longest_range = 0.0;

//         if (!getLongestRange(longest_angle, longest_range))
//         {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 2000,
//                 "SEARCHING: No valid filtered LaserScan range found.");
//             break;
//         }

//         target_angle_ = longest_angle;
//         target_range_ = longest_range;
//         align_yaw_initialised_ = false;

//         RCLCPP_INFO(
//             this->get_logger(),
//             "SEARCHING: Selected exploration heading at %.2f deg, representative range %.2f m",
//             target_angle_ * 180.0 / M_PI,
//             target_range_);

//         current_state_ = NavigationState::ALIGNING;
//         break;
//     }

//     case NavigationState::ALIGNING:
//     {
//         clearObstacleMarkers();
//         twist.linear.x = 0.0;

//         if (!imu_available_)
//         {
//             // IMU not yet publishing — fall back to open-loop time integration.
//             if (std::fabs(target_angle_) <= kAlignmentTolerance)
//             {
//                 stopRobot(twist);
//                 if (std::fabs(twist.angular.z) <= 1e-6)
//                 {
//                     RCLCPP_INFO(this->get_logger(),
//                         "ALIGNING (open-loop): Aligned. Starting forward movement.");
//                     current_state_ = NavigationState::MOVING;
//                 }
//                 break;
//             }

//             const double turn_cmd = std::clamp(
//                 target_angle_ * kAngleGain, -kTurnSpeed, kTurnSpeed);
//             twist.angular.z = turn_cmd;
//             target_angle_ -= turn_cmd * kControlPeriodSeconds;
//             target_angle_ = normaliseAngle(target_angle_);

//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//                 "ALIGNING: No IMU data yet — using open-loop time estimate. "
//                 "Remaining %.2f deg", target_angle_ * 180.0 / M_PI);
//             break;
//         }

//         // IMU-confirmed alignment: compare actual yaw turned vs. required angle.
//         tf2::Quaternion q;
//         tf2::fromMsg(latest_imu_.orientation, q);
//         const double current_yaw = tf2::getYaw(q);

//         if (!align_yaw_initialised_)
//         {
//             align_start_yaw_ = current_yaw;
//             align_yaw_initialised_ = true;
//         }

//         // How much has the robot actually rotated since ALIGNING began.
//         const double yaw_turned = normaliseAngle(current_yaw - align_start_yaw_);

//         // How many degrees still remain.
//         const double remaining_angle = normaliseAngle(target_angle_ - yaw_turned);

//         if (std::fabs(remaining_angle) <= kAlignmentTolerance)
//         {
//             stopRobot(twist);
//             if (std::fabs(twist.angular.z) <= 1e-6)
//             {
//                 RCLCPP_INFO(this->get_logger(),
//                     "ALIGNING: IMU confirmed rotation. Turned %.2f deg (target %.2f deg). Starting forward movement.",
//                     yaw_turned * 180.0 / M_PI,
//                     target_angle_ * 180.0 / M_PI);
//                 current_state_ = NavigationState::MOVING;
//             }
//             break;
//         }

//         const double turn_cmd = std::clamp(
//             remaining_angle * kAngleGain, -kTurnSpeed, kTurnSpeed);
//         twist.angular.z = turn_cmd;

//         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//             "ALIGNING: IMU-confirmed. Target %.2f deg, turned %.2f deg, remaining %.2f deg, cmd %.2f rad/s",
//             target_angle_ * 180.0 / M_PI,
//             yaw_turned * 180.0 / M_PI,
//             remaining_angle * 180.0 / M_PI,
//             twist.angular.z);

//         break;
//     }

//     case NavigationState::MOVING:
//     {
//         std::vector<LaserSegment> blocking_segments;
//         const bool blocked_by_segment = findBlockingObstaclesInFront(blocking_segments);
//         const double front_range = getFrontRange();

//         // Segment-based blocking is the main decision. This prevents a single
//         // random dot from stopping the robot because the dot will not survive
//         // the neighbour + segment filter.
//         if (blocked_by_segment || front_range <= stop_distance_m_)
//         {
//             if (blocking_segments.empty())
//             {
//                 // Fallback marker if the range check caught something but no
//                 // segment was available. This should be rare after filtering.
//                 LaserSegment fallback;
//                 fallback.point_count = 1;
//                 fallback.min_range = front_range;
//                 fallback.midpoint.x = std::isfinite(front_range) ? front_range : stop_distance_m_;
//                 fallback.midpoint.y = 0.0;
//                 fallback.midpoint.z = 0.0;
//                 blocking_segments.push_back(fallback);
//             }

//             publishObstacleMarkers(blocking_segments);

//             RCLCPP_WARN(
//                 this->get_logger(),
//                 "MOVING: Blocking obstacle detected in front. Front range = %.2f m. Stopping.",
//                 front_range);

//             stopRobot(twist);
//             stop_counter_ = 0;
//             current_state_ = NavigationState::STOPPED;
//             break;
//         }

//         clearObstacleMarkers();

//         if (std::isinf(front_range))
//         {
//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "MOVING: Front is clear after filtering. Driving forward.");
//         }
//         else
//         {
//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "MOVING: Driving forward. Filtered front range = %.2f m",
//                 front_range);
//         }

//         twist.linear.x = kForwardSpeed;
//         twist.angular.z = 0.0;
//         break;
//     }

//     case NavigationState::STOPPED:
//     {
//         stopRobot(twist);
//         stop_counter_++;

//         if (stop_counter_ >= kStopDurationLoops)
//         {
//             RCLCPP_INFO(
//                 this->get_logger(),
//                 "STOPPED: Pause complete. Searching for next direction.");

//             stop_counter_ = 0;
//             clearObstacleMarkers();
//             current_state_ = NavigationState::SEARCHING;
//         }
//         else
//         {
//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "STOPPED: Pausing %.1f / %.1f seconds",
//                 stop_counter_ * kControlPeriodSeconds,
//                 kStopDurationLoops * kControlPeriodSeconds);
//         }

//         break;
//     }

//     case NavigationState::HUMAN_DETECTED:
//     {
//         const double time_since_tracking =
//             (this->now() - last_human_tracking_time_).seconds();
//         const double time_since_visible_human = g_last_visible_human_valid
//             ? (this->now() - g_last_visible_human_time).seconds()
//             : std::numeric_limits<double>::infinity();

//         // If the robot has successfully reached interaction state, hold the
//         // interaction pose for a fixed session time, then move on and ignore
//         // camera-triggered human interrupts for a short cooldown period.
//         if (g_interaction_session_active)
//         {
//             const double interaction_elapsed =
//                 (this->now() - g_interaction_session_start_time).seconds();

//             if (interaction_elapsed >= kHumanInteractionDurationSeconds)
//             {
//                 g_human_interaction_completed_count++;
//                 g_interaction_session_active = false;
//                 g_interaction_hold_active = false;
//                 g_human_detection_cooldown_active = true;
//                 g_human_detection_cooldown_start_time = this->now();
//                 human_locked_ = false;
//                 safety_zone_violated_ = false;
//                 safety_zone_baseline_captured_ = false;
//                 clearObstacleMarkers();
//                 stopRobot(twist);
//                 current_state_ = NavigationState::SEARCHING;

//                 writeHumanInteractionStatsLog("completed_30s_interaction_started_cooldown", this->now());

//                 RCLCPP_INFO(
//                     this->get_logger(),
//                     "HUMAN_SESSION: Completed %.1f s interaction. Returning to DVD exploration with %.1f s human-detection cooldown.",
//                     kHumanInteractionDurationSeconds,
//                     kHumanDetectionCooldownSeconds);
//                 break;
//             }
//         }

//         // If the detector briefly drops the human while we are correcting near a wall,
//         // do not instantly return to DVD exploration. Hold/reacquire using the last
//         // confirmed camera offset so the interaction has a chance to recover.
//         if (!human_locked_)
//         {
//             const double hold_age = g_interaction_hold_active
//                 ? (this->now() - g_interaction_hold_time).seconds()
//                 : std::numeric_limits<double>::infinity();

//             // If we had already reached a valid interaction pose, do not rotate
//             // or return to DVD exploration just because the detector flickers.
//             // Hold still so the camera has a chance to reacquire the person.
//             if (g_interaction_hold_active && hold_age <= kHumanInteractionHoldTimeout)
//             {
//                 twist.linear.x = 0.0;
//                 twist.angular.z = 0.0;

//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     500,
//                     "HUMAN_HOLD: Human briefly lost %.2f s after reaching interaction pose. Holding still at last range %.2f m.",
//                     hold_age,
//                     g_interaction_hold_range_m);
//                 break;
//             }

//             if (time_since_visible_human <= kHumanLostTimeout)
//             {
//                 twist.linear.x = kHumanRecoveryCreepSpeed;

//                 if (std::fabs(g_last_visible_human_centre_offset) <= kHumanRecoveryRotateOffsetThreshold)
//                 {
//                     // The person was last seen near the centre, so rotating can make
//                     // reacquisition worse. Hold still instead.
//                     twist.angular.z = 0.0;
//                 }
//                 else
//                 {
//                     twist.angular.z = std::clamp(
//                         -kHumanTurnGain * g_last_visible_human_centre_offset,
//                         -kHumanRecoveryTurnSpeed,
//                         kHumanRecoveryTurnSpeed);
//                 }

//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     500,
//                     "HUMAN_RECOVERY: Human briefly lost %.2f s ago. Last offset %.2f, cmd angular=%.2f",
//                     time_since_visible_human,
//                     g_last_visible_human_centre_offset,
//                     twist.angular.z);
//                 break;
//             }

//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: Human lost for %.2f s. Returning to blind autonomous search.",
//                 time_since_visible_human);

//             if (g_interaction_session_active || g_interaction_hold_active)
//             {
//                 g_human_abandoned_after_interaction_count++;
//             }
//             else
//             {
//                 g_human_abandoned_before_interaction_count++;
//             }

//             writeHumanInteractionStatsLog("human_lost_returning_to_exploration", this->now());

//             stopRobot(twist);
//             human_locked_ = false;
//             safety_zone_violated_ = false;
//             safety_zone_baseline_captured_ = false;
//             g_interaction_hold_active = false;
//             g_interaction_session_active = false;
//             clearObstacleMarkers();
//             current_state_ = NavigationState::SEARCHING;
//             break;
//         }

//         if (time_since_tracking > kHumanLostTimeout)
//         {
//             const double hold_age = g_interaction_hold_active
//                 ? (this->now() - g_interaction_hold_time).seconds()
//                 : std::numeric_limits<double>::infinity();

//             if (g_interaction_hold_active && hold_age <= kHumanInteractionHoldTimeout)
//             {
//                 twist.linear.x = 0.0;
//                 twist.angular.z = 0.0;

//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     500,
//                     "HUMAN_HOLD: Tracking stale %.2f s, but interaction pose was recently reached. Holding still.",
//                     time_since_tracking);
//                 break;
//             }

//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: Tracking stale for %.2f s. Returning to blind autonomous search.",
//                 time_since_tracking);

//             if (g_interaction_session_active || g_interaction_hold_active)
//             {
//                 g_human_abandoned_after_interaction_count++;
//             }
//             else
//             {
//                 g_human_abandoned_before_interaction_count++;
//             }

//             writeHumanInteractionStatsLog("human_lost_returning_to_exploration", this->now());

//             stopRobot(twist);
//             human_locked_ = false;
//             safety_zone_violated_ = false;
//             safety_zone_baseline_captured_ = false;
//             g_interaction_hold_active = false;
//             g_interaction_session_active = false;
//             clearObstacleMarkers();
//             current_state_ = NavigationState::SEARCHING;
//             break;
//         }

//         if (!safety_zone_baseline_captured_)
//         {
//             captureSafetyZoneBaseline();
//         }

//         const double human_bearing_rad = -human_centre_offset_ * kCameraHorizontalFov;
//         const bool zone_now_violated = isSafetyZoneViolated(human_bearing_rad);

//         if (zone_now_violated != safety_zone_violated_)
//         {
//             safety_zone_violated_ = zone_now_violated;

//             if (safety_zone_violated_)
//             {
//                 RCLCPP_WARN(
//                     this->get_logger(),
//                     "SAFETY ZONE: Object detected within human-mode safety zone. Front/sides %.1f m, rear %.1f m. Interaction paused.",
//                     kSafetyZoneRadius,
//                     kHumanRearSafetyRadius);
//             }
//             else
//             {
//                 RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Clear. Resuming interaction.");
//             }
//         }

//         if (safety_zone_violated_)
//         {
//             stopRobot(twist);
//             break;
//         }

//         // ------------------------------------------------------
//         // Human tracking + interaction-space behaviour
//         // ------------------------------------------------------
//         // The normal human approach command keeps the person centred and moves
//         // to the target distance. The extra logic below checks whether there is
//         // enough 360-degree clearance for the arms. If not, the robot chooses a
//         // short safe arc that is predicted to improve the 1.5 m interaction
//         // bubble while still keeping the human in view.

//         const double human_keep_turn =
//             (std::fabs(human_centre_offset_) <= kHumanCentreDeadZone)
//                 ? 0.0
//                 : std::clamp(
//                     -kHumanTurnGain * human_centre_offset_,
//                     -kHumanMaxTurnSpeed,
//                     kHumanMaxTurnSpeed);

//         const double human_lidar_range = getHumanLidarRange(human_centre_offset_);
//         const bool lidar_distance_valid = std::isfinite(human_lidar_range);

//         const double estimated_human_range =
//             lidar_distance_valid
//                 ? human_lidar_range
//                 : ((human_distance_m_ > 0.0 && std::isfinite(human_distance_m_))
//                     ? human_distance_m_
//                     : kHumanTargetDistance);

//         const double human_x = estimated_human_range * std::cos(human_bearing_rad);
//         const double human_y = estimated_human_range * std::sin(human_bearing_rad);

//         struct InteractionBubbleCheck
//         {
//             int considered_beams = 0;
//             int blocked_beams = 0;
//             double min_clearance = std::numeric_limits<double>::infinity();
//             double blocked_fraction = 0.0;
//             bool clear = false;
//         };

//         auto evaluateInteractionBubbleAt = [&](double future_x, double future_y)
//         {
//             InteractionBubbleCheck check;

//             if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//             {
//                 check.clear = false;
//                 check.blocked_beams = 9999;
//                 check.blocked_fraction = 1.0;
//                 check.min_clearance = 0.0;
//                 return check;
//             }

//             for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//             {
//                 const double range = latest_scan_.ranges[i];

//                 if (!std::isfinite(range) || range <= kMinValidRange)
//                 {
//                     continue;
//                 }

//                 const double angle = latest_scan_.angle_min +
//                     static_cast<double>(i) * latest_scan_.angle_increment;
//                 const double angle_diff_to_human = normaliseAngle(angle - human_bearing_rad);

//                 // The tracked human is allowed to be in the front interaction
//                 // window. Everything else inside the arm bubble blocks interaction.
//                 if (std::fabs(angle_diff_to_human) <= kInteractionHumanExclusionAngle)
//                 {
//                     continue;
//                 }

//                 const double point_x = range * std::cos(angle);
//                 const double point_y = range * std::sin(angle);
//                 const double distance_to_future_robot =
//                     std::hypot(point_x - future_x, point_y - future_y);

//                 check.considered_beams++;
//                 check.min_clearance = std::min(check.min_clearance, distance_to_future_robot);

//                 const double required_radius = humanModeInteractionBubbleRadiusForAngle(angle);

//                 if (distance_to_future_robot < required_radius)
//                 {
//                     check.blocked_beams++;
//                 }
//             }

//             if (check.considered_beams > 0)
//             {
//                 check.blocked_fraction = static_cast<double>(check.blocked_beams) /
//                     static_cast<double>(check.considered_beams);
//             }
//             else
//             {
//                 // No finite obstacles outside the human window means the bubble
//                 // is clear as far as LiDAR can tell.
//                 check.blocked_fraction = 0.0;
//             }

//             check.clear = check.blocked_beams <= kInteractionAllowedBlockedBeams;
//             return check;
//         };

//         auto pathToFuturePoseIsClear = [&](double heading, double move_distance)
//         {
//             if (move_distance <= 1e-3)
//             {
//                 return true;
//             }

//             const double dir_x = std::cos(heading);
//             const double dir_y = std::sin(heading);
//             const double max_forward = move_distance + kInteractionPathForwardBuffer;

//             for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//             {
//                 const double range = latest_scan_.ranges[i];

//                 if (!std::isfinite(range) || range <= kMinValidRange)
//                 {
//                     continue;
//                 }

//                 const double angle = latest_scan_.angle_min +
//                     static_cast<double>(i) * latest_scan_.angle_increment;
//                 const double angle_diff_to_human = normaliseAngle(angle - human_bearing_rad);

//                 if (std::fabs(angle_diff_to_human) <= kInteractionHumanExclusionAngle)
//                 {
//                     continue;
//                 }

//                 const double point_x = range * std::cos(angle);
//                 const double point_y = range * std::sin(angle);

//                 const double forward = point_x * dir_x + point_y * dir_y;
//                 const double lateral = -point_x * dir_y + point_y * dir_x;

//                 if (forward > 0.0 && forward < max_forward &&
//                     std::fabs(lateral) < kInteractionPathHalfWidth)
//                 {
//                     return false;
//                 }
//             }

//             return true;
//         };

//         const InteractionBubbleCheck current_bubble = evaluateInteractionBubbleAt(0.0, 0.0);
//         const bool interaction_bubble_clear = current_bubble.clear;

//         if (lidar_distance_valid &&
//             human_distance_m_ > 0.0 &&
//             std::isfinite(human_distance_m_))
//         {
//             const double disagreement = std::fabs(human_distance_m_ - human_lidar_range);

//             if (disagreement > kLidarCameraMaxDisagreement)
//             {
//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     1000,
//                     "HUMAN_DETECTED: Camera/LiDAR distance disagreement. Camera=%.2f m, LiDAR=%.2f m",
//                     human_distance_m_,
//                     human_lidar_range);
//             }
//         }

//         const double distance_error = estimated_human_range - kHumanLidarStopDistance;
//         const bool at_human_target_distance =
//             std::fabs(distance_error) <= kHumanLidarStopTolerance;

//         const bool human_centred_for_interaction =
//             std::fabs(human_centre_offset_) <= kHumanInteractionHoldMaxOffset;

//         const bool interaction_pose_valid =
//             interaction_bubble_clear &&
//             at_human_target_distance &&
//             human_centred_for_interaction;

//         if (interaction_pose_valid)
//         {
//             if (!g_interaction_session_active)
//             {
//                 g_interaction_session_active = true;
//                 g_interaction_session_start_time = this->now();
//                 g_human_interaction_success_count++;
//                 writeHumanInteractionStatsLog("entered_interaction_state_successfully", this->now());

//                 RCLCPP_INFO(
//                     this->get_logger(),
//                     "HUMAN_SESSION: Entered interaction state successfully. Count=%d. Holding for %.1f s.",
//                     g_human_interaction_success_count,
//                     kHumanInteractionDurationSeconds);
//             }

//             g_interaction_hold_active = true;
//             g_interaction_hold_time = this->now();
//             g_interaction_hold_range_m = estimated_human_range;
//             g_interaction_hold_offset = human_centre_offset_;
//         }
//         else if (g_interaction_hold_active)
//         {
//             const bool still_close_to_hold_range =
//                 std::fabs(estimated_human_range - kHumanLidarStopDistance) <=
//                     (kHumanLidarStopTolerance + kHumanInteractionHoldRangeSlack);

//             if (!interaction_bubble_clear ||
//                 !still_close_to_hold_range ||
//                 std::fabs(human_centre_offset_) > (kHumanInteractionHoldMaxOffset + 0.12))
//             {
//                 g_interaction_hold_active = false;
//             }
//         }

//         // If the robot is at interaction distance but the arm bubble is blocked,
//         // do not enter arm interaction. Reposition instead.
//         const bool needs_interaction_reposition = !interaction_bubble_clear;

//         // If the person is already close to the edge of the camera image, do not
//         // start a wall-clearance arc yet. First slow down and re-centre them. This
//         // prevents the robot from choosing a 30-45 degree clearance manoeuvre that
//         // swings the human out of view.
//         if (needs_interaction_reposition &&
//             std::fabs(human_centre_offset_) >= kInteractionRecenterFirstOffset)
//         {
//             twist.linear.x = 0.0;
//             twist.angular.z = std::clamp(
//                 human_keep_turn,
//                 -kInteractionRepositionMaxAngularSpeed,
//                 kInteractionRepositionMaxAngularSpeed);

//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 500,
//                 "HUMAN_REPOSITION: Arm bubble blocked, but human is near image edge (offset %.2f). Recentring first. cmd angular=%.2f",
//                 human_centre_offset_,
//                 twist.angular.z);
//             break;
//         }

//         if (needs_interaction_reposition)
//         {
//             struct RepositionCandidate
//             {
//                 double heading = 0.0;
//                 double move_distance = 0.0;
//                 double linear = 0.0;
//                 double angular = 0.0;
//                 double score = -std::numeric_limits<double>::infinity();
//                 InteractionBubbleCheck bubble;
//                 bool path_clear = false;
//             };

//             const std::vector<double> candidate_headings = {
//                 -35.0 * M_PI / 180.0,
//                 -25.0 * M_PI / 180.0,
//                 -15.0 * M_PI / 180.0,
//                  -8.0 * M_PI / 180.0,
//                   0.0,
//                   8.0 * M_PI / 180.0,
//                  15.0 * M_PI / 180.0,
//                  25.0 * M_PI / 180.0,
//                  35.0 * M_PI / 180.0
//             };

//             RepositionCandidate best_candidate;
//             const double required_radius =
//                 kInteractionBubbleRadius + kInteractionBubbleSafetyMargin;

//             for (const double heading : candidate_headings)
//             {
//                 const double abs_heading = std::fabs(heading);

//                 // More sideways arcs move a little less in one decision step.
//                 const double move_scale = std::clamp(
//                     1.0 - 0.45 * (abs_heading / kInteractionMaxCandidateAngle),
//                     0.45,
//                     1.0);
//                 const double move_distance = kInteractionRepositionLookahead * move_scale;
//                 const double future_x = move_distance * std::cos(heading);
//                 const double future_y = move_distance * std::sin(heading);

//                 RepositionCandidate candidate;
//                 candidate.heading = heading;
//                 candidate.move_distance = move_distance;
//                 candidate.path_clear = pathToFuturePoseIsClear(heading, move_distance);
//                 candidate.bubble = evaluateInteractionBubbleAt(future_x, future_y);

//                 const double future_human_bearing = normaliseAngle(
//                     std::atan2(human_y - future_y, human_x - future_x));
//                 const double future_human_distance =
//                     std::hypot(human_x - future_x, human_y - future_y);

//                 // Estimate where the person would sit in the camera after the short
//                 // reposition. This is approximate, but it lets us heavily reject moves
//                 // that are likely to push the human out of frame.
//                 const double predicted_offset = std::clamp(
//                     -future_human_bearing / kCameraHorizontalFov,
//                     -0.75,
//                     0.75);
//                 const double human_center_score = std::clamp(
//                     1.0 - std::fabs(predicted_offset) / 0.45,
//                     0.0,
//                     1.0);
//                 const double human_distance_score = std::clamp(
//                     1.0 - std::fabs(future_human_distance - kHumanLidarStopDistance) / 1.0,
//                     0.0,
//                     1.0);
//                 const double clearance_score = std::clamp(
//                     candidate.bubble.min_clearance / required_radius,
//                     0.0,
//                     1.4);
//                 const double blocked_score = 1.0 - std::clamp(
//                     candidate.bubble.blocked_fraction,
//                     0.0,
//                     1.0);
//                 const double human_view_penalty = std::clamp(
//                     (std::fabs(predicted_offset) - kInteractionRecenterFirstOffset) /
//                         (kInteractionViewLossOffset - kInteractionRecenterFirstOffset),
//                     0.0,
//                     1.0);
//                 const double parallel_arc_score = std::clamp(
//                     1.0 - std::fabs(abs_heading - kInteractionParallelBiasAngle) /
//                         kInteractionParallelBiasAngle,
//                     0.0,
//                     1.0);

//                 // Human view is the hard constraint. Clearance still matters, but the
//                 // robot should improve the arm bubble through repeated shallow arcs,
//                 // not a single sharp turn that loses the person.
//                 candidate.score =
//                     5.0 * blocked_score +
//                     2.0 * clearance_score +
//                     6.0 * human_center_score +
//                     1.0 * human_distance_score +
//                     1.2 * parallel_arc_score -
//                     3.0 * (abs_heading / kInteractionMaxCandidateAngle) -
//                     10.0 * human_view_penalty;

//                 if (!candidate.path_clear)
//                 {
//                     candidate.score -= 8.0;
//                 }

//                 // Prefer candidates that improve the currently blocked bubble, but do
//                 // not let this dominate the camera-view constraint.
//                 if (candidate.bubble.blocked_beams < current_bubble.blocked_beams)
//                 {
//                     candidate.score += 1.5;
//                 }

//                 const double candidate_turn = std::clamp(
//                     candidate.heading * kInteractionRepositionTurnGain,
//                     -kInteractionRepositionMaxAngularSpeed,
//                     kInteractionRepositionMaxAngularSpeed);

//                 // Human centring dominates; reposition is a gentle bias. This keeps
//                 // the camera engaged with the person while the base clears the wall.
//                 candidate.angular = std::clamp(
//                     0.30 * candidate_turn + 0.70 * human_keep_turn,
//                     -kInteractionRepositionMaxAngularSpeed,
//                     kInteractionRepositionMaxAngularSpeed);

//                 candidate.linear = std::clamp(
//                     kInteractionRepositionSpeed * move_scale,
//                     0.03,
//                     kInteractionRepositionSpeed);

//                 if (candidate.score > best_candidate.score)
//                 {
//                     best_candidate = candidate;
//                 }
//             }

//             if (best_candidate.score > -1.0)
//             {
//                 twist.linear.x = best_candidate.linear;
//                 twist.angular.z = best_candidate.angular;

//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     750,
//                     "HUMAN_REPOSITION: Arm bubble blocked now (%d beams, min %.2f m). "
//                     "Best heading %.1f deg -> future blocked %d beams, min %.2f m. cmd linear=%.2f angular=%.2f",
//                     current_bubble.blocked_beams,
//                     current_bubble.min_clearance,
//                     best_candidate.heading * 180.0 / M_PI,
//                     best_candidate.bubble.blocked_beams,
//                     best_candidate.bubble.min_clearance,
//                     twist.linear.x,
//                     twist.angular.z);
//             }
//             else
//             {
//                 // If there is no safe reposition movement, at least keep the
//                 // human in view and do not enter arm interaction.
//                 twist.linear.x = 0.0;
//                 twist.angular.z = std::clamp(human_keep_turn, -kInteractionRepositionMaxAngularSpeed, kInteractionRepositionMaxAngularSpeed);

//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     1000,
//                     "HUMAN_REPOSITION: No safe movement found to clear arm bubble. Holding and keeping human centred.");
//             }

//             break;
//         }

//         // Arm bubble is clear, so normal human approach/hold behaviour can run.
//         // If we have latched a good interaction pose, stay still. This prevents
//         // the robot from creeping forward and losing the detector right when it
//         // reaches the usable arm-interaction distance.
//         if (g_interaction_hold_active && interaction_pose_valid)
//         {
//             twist.linear.x = 0.0;
//             twist.angular.z = 0.0;

//             const double interaction_elapsed = g_interaction_session_active
//                 ? (this->now() - g_interaction_session_start_time).seconds()
//                 : 0.0;

//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 500,
//                 "HUMAN_HOLD: Interaction pose valid. Holding at range %.2f m, offset %.2f, bubble min %.2f m. Session %.1f / %.1f s.",
//                 estimated_human_range,
//                 human_centre_offset_,
//                 current_bubble.min_clearance,
//                 interaction_elapsed,
//                 kHumanInteractionDurationSeconds);
//             break;
//         }

//         twist.angular.z = human_keep_turn;

//         if (!lidar_distance_valid && !(human_distance_m_ > 0.0 && std::isfinite(human_distance_m_)))
//         {
//             twist.linear.x = 0.0;

//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: No valid distance estimate. Turning only to keep human centred.");
//         }
//         else if (at_human_target_distance)
//         {
//             twist.linear.x = 0.0;

//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: Human at target distance %.2f m and 1.5 m arm bubble clear. Ready for interaction.",
//                 estimated_human_range);
//         }
//         else
//         {
//             twist.linear.x = std::clamp(
//                 kHumanForwardGain * distance_error,
//                 -kHumanMaxReverseSpeed,
//                 kHumanMaxForwardSpeed);
//         }

//         if (estimated_human_range < kHumanLidarStopDistance && twist.linear.x > 0.0)
//         {
//             twist.linear.x = 0.0;
//         }

//         RCLCPP_INFO_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             500,
//             "HUMAN_DETECTED: offset=%.2f distance=%.2f arm_clear=%s blocked=%d min_clearance=%.2f cmd linear=%.2f angular=%.2f",
//             human_centre_offset_,
//             estimated_human_range,
//             interaction_bubble_clear ? "yes" : "no",
//             current_bubble.blocked_beams,
//             current_bubble.min_clearance,
//             twist.linear.x,
//             twist.angular.z);

//         break;
//     }

//     default:
//     {
//         RCLCPP_WARN(this->get_logger(), "Unknown navigation state. Returning to SEARCHING.");
//         stopRobot(twist);
//         clearObstacleMarkers();
//         current_state_ = NavigationState::SEARCHING;
//         break;
//     }
//     }

//     current_twist_ = twist;
//     cmd_vel_publisher_->publish(twist);
// }

// void MechelangeloBehaviour::stopRobot(geometry_msgs::msg::Twist &twist)
// {
//     const double linear_step = kStopLinearDecel * kControlPeriodSeconds;
//     const double angular_step = kStopAngularDecel * kControlPeriodSeconds;

//     auto rampTowardZero = [](double value, double max_step)
//     {
//         if (std::fabs(value) <= max_step)
//         {
//             return 0.0;
//         }

//         return value - std::copysign(max_step, value);
//     };

//     twist.linear.x = rampTowardZero(current_twist_.linear.x, linear_step);
//     twist.linear.y = rampTowardZero(current_twist_.linear.y, linear_step);
//     twist.linear.z = rampTowardZero(current_twist_.linear.z, linear_step);

//     twist.angular.x = rampTowardZero(current_twist_.angular.x, angular_step);
//     twist.angular.y = rampTowardZero(current_twist_.angular.y, angular_step);
//     twist.angular.z = rampTowardZero(current_twist_.angular.z, angular_step);
// }

// void MechelangeloBehaviour::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//     const sensor_msgs::msg::LaserScan filtered_scan = filterLaserScan(*msg);

//     latest_scan_ = filtered_scan;
//     latest_segments_ = buildLaserSegments(filtered_scan);

//     filtered_scan_publisher_->publish(filtered_scan);
// }

// void MechelangeloBehaviour::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//     latest_imu_ = *msg;
//     imu_available_ = true;
// }

// void MechelangeloBehaviour::humanDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     if (!msg->data)
//     {
//         return;
//     }

//     if (humanDetectionCooldownActive(this->now()))
//     {
//         g_human_detection_ignored_cooldown_count++;
//         writeHumanInteractionStatsLog("manual_human_detection_ignored_during_cooldown", this->now());

//         RCLCPP_WARN_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             1000,
//             "Human detection ignored during cooldown. Remaining %.1f s.",
//             humanDetectionCooldownRemaining(this->now()));
//         return;
//     }

//     if (g_human_detection_cooldown_active)
//     {
//         g_human_detection_cooldown_active = false;
//     }

//     RCLCPP_WARN(
//         this->get_logger(),
//         "Manual human detection trigger received. Interrupting autonomous behaviour.");

//     current_state_ = NavigationState::HUMAN_DETECTED;
// }

// void MechelangeloBehaviour::humanTrackingCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
// {
//     if (msg->data.size() < 3)
//     {
//         RCLCPP_WARN_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             2000,
//             "Received invalid /human_tracking message. Expected [detected, centre_offset, distance_m].");
//         return;
//     }

//     const bool detected = msg->data[0] > 0.5F;

//     if (detected && humanDetectionCooldownActive(this->now()))
//     {
//         g_human_detection_ignored_cooldown_count++;
//         human_locked_ = false;
//         writeHumanInteractionStatsLog("camera_human_detection_ignored_during_cooldown", this->now());

//         RCLCPP_WARN_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             1000,
//             "Camera human detection ignored during cooldown. Remaining %.1f s.",
//             humanDetectionCooldownRemaining(this->now()));
//         return;
//     }

//     if (g_human_detection_cooldown_active && !humanDetectionCooldownActive(this->now()))
//     {
//         g_human_detection_cooldown_active = false;
//         writeHumanInteractionStatsLog("human_detection_cooldown_finished", this->now());
//     }

//     human_locked_ = detected;
//     human_centre_offset_ = static_cast<double>(msg->data[1]);
//     human_distance_m_ = static_cast<double>(msg->data[2]);
//     last_human_tracking_time_ = this->now();

//     if (human_locked_)
//     {
//         g_human_detection_accepted_count++;
//         g_last_visible_human_valid = true;
//         g_last_visible_human_centre_offset = human_centre_offset_;
//         g_last_visible_human_distance_m = human_distance_m_;
//         g_last_visible_human_time = last_human_tracking_time_;
//         current_state_ = NavigationState::HUMAN_DETECTED;
//     }
// }

// sensor_msgs::msg::LaserScan MechelangeloBehaviour::filterLaserScan(
//     const sensor_msgs::msg::LaserScan &raw_scan)
// {
//     sensor_msgs::msg::LaserScan neighbour_filtered = raw_scan;
//     sensor_msgs::msg::LaserScan final_filtered = raw_scan;

//     if (raw_scan.ranges.empty() || raw_scan.angle_increment == 0.0)
//     {
//         return final_filtered;
//     }

//     const int scan_count = static_cast<int>(raw_scan.ranges.size());
//     std::vector<double> x_points(scan_count, std::numeric_limits<double>::quiet_NaN());
//     std::vector<double> y_points(scan_count, std::numeric_limits<double>::quiet_NaN());
//     std::vector<bool> usable(scan_count, false);

//     // Convert valid polar points to local Cartesian points.
//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = raw_scan.ranges[i];

//         if (!isRangeUsableForFiltering(raw_scan, range))
//         {
//             neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             final_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             continue;
//         }

//         const double angle = raw_scan.angle_min + static_cast<double>(i) * raw_scan.angle_increment;
//         x_points[i] = range * std::cos(angle);
//         y_points[i] = range * std::sin(angle);
//         usable[i] = true;
//     }

//     // Stage 1: suppress isolated points that do not have nearby neighbours.
//     for (int i = 0; i < scan_count; ++i)
//     {
//         if (!usable[i])
//         {
//             neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             continue;
//         }

//         int close_neighbour_count = 0;
//         const int start_index = std::max(0, i - kNoiseNeighbourWindow);
//         const int end_index = std::min(scan_count - 1, i + kNoiseNeighbourWindow);

//         for (int j = start_index; j <= end_index; ++j)
//         {
//             if (j == i || !usable[j])
//             {
//                 continue;
//             }

//             const double distance = std::hypot(x_points[i] - x_points[j], y_points[i] - y_points[j]);

//             if (distance <= kNoiseNeighbourDistance)
//             {
//                 close_neighbour_count++;
//             }
//         }

//         if (close_neighbour_count < kNoiseMinNeighbourCount)
//         {
//             neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
//         }
//     }

//     // Stage 2: build LaserProcessing-style segments and only keep points
//     // that belong to a real segment.
//     const std::vector<LaserSegment> accepted_segments = buildLaserSegments(neighbour_filtered);

//     std::fill(final_filtered.ranges.begin(), final_filtered.ranges.end(), std::numeric_limits<float>::infinity());

//     for (const LaserSegment &segment : accepted_segments)
//     {
//         for (int i = segment.start_index; i <= segment.end_index; ++i)
//         {
//             if (i >= 0 && i < scan_count && std::isfinite(neighbour_filtered.ranges[i]))
//             {
//                 final_filtered.ranges[i] = neighbour_filtered.ranges[i];
//             }
//         }
//     }

//     return final_filtered;
// }

// std::vector<LaserSegment> MechelangeloBehaviour::buildLaserSegments(
//     const sensor_msgs::msg::LaserScan &scan) const
// {
//     std::vector<LaserSegment> segments;

//     if (scan.ranges.empty() || scan.angle_increment == 0.0)
//     {
//         return segments;
//     }

//     const int scan_count = static_cast<int>(scan.ranges.size());

//     bool segment_active = false;
//     LaserSegment current_segment;
//     geometry_msgs::msg::Point previous_point;

//     auto finish_segment = [&]()
//     {
//         if (!segment_active)
//         {
//             return;
//         }

//         current_segment.midpoint.x = 0.5 * (current_segment.start_point.x + current_segment.end_point.x);
//         current_segment.midpoint.y = 0.5 * (current_segment.start_point.y + current_segment.end_point.y);
//         current_segment.midpoint.z = 0.0;
//         current_segment.length = std::hypot(
//             current_segment.end_point.x - current_segment.start_point.x,
//             current_segment.end_point.y - current_segment.start_point.y);
//         current_segment.midpoint_angle = std::atan2(
//             current_segment.midpoint.y,
//             current_segment.midpoint.x);

//         if (current_segment.point_count >= kSegmentMinPoints &&
//             current_segment.length >= kSegmentMinLength)
//         {
//             segments.push_back(current_segment);
//         }

//         segment_active = false;
//         current_segment = LaserSegment();
//     };

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = scan.ranges[i];

//         if (!isRangeUsableForFiltering(scan, range))
//         {
//             finish_segment();
//             continue;
//         }

//         const geometry_msgs::msg::Point point = polarToPoint(scan, i);

//         if (!segment_active)
//         {
//             segment_active = true;
//             current_segment = LaserSegment();
//             current_segment.start_index = i;
//             current_segment.end_index = i;
//             current_segment.point_count = 1;
//             current_segment.start_point = point;
//             current_segment.end_point = point;
//             current_segment.min_range = range;
//             previous_point = point;
//             continue;
//         }

//         const double gap = std::hypot(point.x - previous_point.x, point.y - previous_point.y);

//         if (gap <= kSegmentJoinDistance)
//         {
//             current_segment.end_index = i;
//             current_segment.end_point = point;
//             current_segment.point_count++;
//             current_segment.min_range = std::min(current_segment.min_range, range);
//             previous_point = point;
//         }
//         else
//         {
//             finish_segment();

//             segment_active = true;
//             current_segment = LaserSegment();
//             current_segment.start_index = i;
//             current_segment.end_index = i;
//             current_segment.point_count = 1;
//             current_segment.start_point = point;
//             current_segment.end_point = point;
//             current_segment.min_range = range;
//             previous_point = point;
//         }
//     }

//     finish_segment();
//     return segments;
// }

// geometry_msgs::msg::Point MechelangeloBehaviour::polarToPoint(
//     const sensor_msgs::msg::LaserScan &scan,
//     int index) const
// {
//     geometry_msgs::msg::Point point;

//     if (index < 0 || index >= static_cast<int>(scan.ranges.size()))
//     {
//         return point;
//     }

//     const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
//     const double range = scan.ranges[index];

//     point.x = range * std::cos(angle);
//     point.y = range * std::sin(angle);
//     point.z = 0.0;

//     return point;
// }

// bool MechelangeloBehaviour::isRangeUsableForFiltering(
//     const sensor_msgs::msg::LaserScan &scan,
//     double range) const
// {
//     if (!std::isfinite(range))
//     {
//         return false;
//     }

//     if (range <= kMinValidRange)
//     {
//         return false;
//     }

//     if (scan.range_min > 0.0 && range < scan.range_min)
//     {
//         return false;
//     }

//     if (scan.range_max > 0.0 && range > scan.range_max)
//     {
//         return false;
//     }

//     return true;
// }

// bool MechelangeloBehaviour::isRangeValid(double range) const
// {
//     return std::isfinite(range) && range > kMinValidRange;
// }

// double MechelangeloBehaviour::getMinimumRange(double start_angle, double end_angle) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return std::numeric_limits<double>::infinity();
//     }

//     double min_range = std::numeric_limits<double>::infinity();

//     for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//     {
//         const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

//         if (!angleInsideWindow(angle, start_angle, end_angle))
//         {
//             continue;
//         }

//         const double range = latest_scan_.ranges[i];

//         if (isRangeValid(range) && range < min_range)
//         {
//             min_range = range;
//         }
//     }

//     return min_range;
// }

// double MechelangeloBehaviour::getFrontRange() const
// {
//     return getMinimumRange(-kFrontCheckAngle, kFrontCheckAngle);
// }

// bool MechelangeloBehaviour::getLongestRange(double &out_angle, double &out_range) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     const int scan_count = static_cast<int>(latest_scan_.ranges.size());
//     const double angle_step = std::fabs(latest_scan_.angle_increment);

//     if (scan_count <= 0 || angle_step <= 0.0)
//     {
//         return false;
//     }

//     // ------------------------------------------------------
//     // Step 1: classify beams as open or blocked.
//     //
//     // Open means either:
//     //   - infinity: the LiDAR did not hit anything within range, or
//     //   - a finite return farther than the clearance distance.
//     // ------------------------------------------------------
//     std::vector<bool> open_beam(scan_count, false);

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = latest_scan_.ranges[i];

//         if (std::isinf(range))
//         {
//             open_beam[i] = true;
//         }
//         else if (std::isfinite(range) && range >= kDvdOpenClearanceDistance)
//         {
//             open_beam[i] = true;
//         }
//     }

//     auto angleForIndex = [&](int index)
//     {
//         return normaliseAngle(
//             latest_scan_.angle_min + static_cast<double>(index) * latest_scan_.angle_increment);
//     };

//     auto absoluteAngleForIndex = [&](int index)
//     {
//         return std::fabs(angleForIndex(index));
//     };

//     auto rangeForLog = [&](int index)
//     {
//         const double range = latest_scan_.ranges[index];

//         if (std::isinf(range))
//         {
//             return std::numeric_limits<double>::infinity();
//         }

//         if (std::isfinite(range))
//         {
//             return range;
//         }

//         return 0.0;
//     };

//     auto isPreferredBounceAngle = [&](int index)
//     {
//         const double abs_angle = absoluteAngleForIndex(index);
//         return abs_angle >= kDvdPreferredMinTurnAngle &&
//                abs_angle <= kDvdPreferredMaxTurnAngle;
//     };

//     auto isFallbackBounceAngle = [&](int index)
//     {
//         const double abs_angle = absoluteAngleForIndex(index);
//         return abs_angle >= kDvdAvoidFrontAngle &&
//                abs_angle <= kDvdAvoidReverseAngle;
//     };

//     // ------------------------------------------------------
//     // Step 2: extract open sectors and collect safe candidate beams.
//     //
//     // Preferred candidates are side-bounce angles, roughly +/-55 to +/-150 deg.
//     // General fallback candidates simply avoid the front wall and exact reverse.
//     // Left and right candidates are stored separately so one long side of the
//     // room does not dominate every decision.
//     // ------------------------------------------------------
//     std::vector<int> preferred_left_candidates;
//     std::vector<int> preferred_right_candidates;
//     std::vector<int> fallback_left_candidates;
//     std::vector<int> fallback_right_candidates;
//     std::vector<int> fallback_all_candidates;

//     int accepted_sector_count = 0;
//     int widest_sector_count = 0;

//     auto collectCandidatesFromSector = [&](int start_index, int count)
//     {
//         if (count <= 0)
//         {
//             return;
//         }

//         const double sector_width = static_cast<double>(count) * angle_step;
//         widest_sector_count = std::max(widest_sector_count, count);

//         if (sector_width < kDvdMinSectorWidth)
//         {
//             return;
//         }

//         accepted_sector_count++;

//         int edge_margin_count = static_cast<int>(std::ceil(kDvdSectorEdgeMargin / angle_step));

//         // Never let the edge margin remove the entire sector.
//         edge_margin_count = std::min(edge_margin_count, std::max(0, (count - 1) / 2));

//         for (int offset = edge_margin_count; offset < count - edge_margin_count; ++offset)
//         {
//             const int index = (start_index + offset) % scan_count;
//             const double angle = angleForIndex(index);

//             if (isPreferredBounceAngle(index))
//             {
//                 if (angle >= 0.0)
//                 {
//                     preferred_left_candidates.push_back(index);
//                 }
//                 else
//                 {
//                     preferred_right_candidates.push_back(index);
//                 }
//             }

//             if (isFallbackBounceAngle(index))
//             {
//                 fallback_all_candidates.push_back(index);

//                 if (angle >= 0.0)
//                 {
//                     fallback_left_candidates.push_back(index);
//                 }
//                 else
//                 {
//                     fallback_right_candidates.push_back(index);
//                 }
//             }
//         }
//     };

//     const bool all_open = std::all_of(open_beam.begin(), open_beam.end(),
//         [](bool value) { return value; });

//     if (all_open)
//     {
//         collectCandidatesFromSector(0, scan_count);
//     }
//     else
//     {
//         for (int start_index = 0; start_index < scan_count; ++start_index)
//         {
//             if (!open_beam[start_index])
//             {
//                 continue;
//             }

//             const int previous_index = (start_index - 1 + scan_count) % scan_count;
//             if (open_beam[previous_index])
//             {
//                 continue;
//             }

//             int count = 0;
//             while (count < scan_count && open_beam[(start_index + count) % scan_count])
//             {
//                 count++;
//             }

//             collectCandidatesFromSector(start_index, count);
//         }
//     }

//     // ------------------------------------------------------
//     // Step 3: choose a random safe bounce angle.
//     //
//     // Preferred behaviour:
//     //   - use the side-bounce band if possible;
//     //   - choose left/right with a 50/50 coin flip when both are available;
//     //   - choose a random beam inside that side's safe candidate list.
//     // This produces a DVD-screensaver style movement instead of always going
//     // down the longest wall/side of the room.
//     // ------------------------------------------------------
//     static thread_local std::mt19937 rng(std::random_device{}());

//     auto chooseFromCandidates = [&](const std::vector<int> &candidates)
//     {
//         std::uniform_int_distribution<int> index_dist(
//             0, static_cast<int>(candidates.size()) - 1);
//         return candidates[index_dist(rng)];
//     };

//     auto chooseBalancedSide = [&](const std::vector<int> &left_candidates,
//                                   const std::vector<int> &right_candidates,
//                                   bool &used_left_side,
//                                   bool &used_right_side)
//     {
//         used_left_side = false;
//         used_right_side = false;

//         if (!left_candidates.empty() && !right_candidates.empty())
//         {
//             std::uniform_int_distribution<int> side_dist(0, 1);

//             if (side_dist(rng) == 0)
//             {
//                 used_left_side = true;
//                 return chooseFromCandidates(left_candidates);
//             }

//             used_right_side = true;
//             return chooseFromCandidates(right_candidates);
//         }

//         if (!left_candidates.empty())
//         {
//             used_left_side = true;
//             return chooseFromCandidates(left_candidates);
//         }

//         used_right_side = true;
//         return chooseFromCandidates(right_candidates);
//     };

//     int selected_index = -1;
//     bool used_preferred_bounce_band = false;
//     bool used_left_side = false;
//     bool used_right_side = false;

//     if (!preferred_left_candidates.empty() || !preferred_right_candidates.empty())
//     {
//         selected_index = chooseBalancedSide(
//             preferred_left_candidates,
//             preferred_right_candidates,
//             used_left_side,
//             used_right_side);
//         used_preferred_bounce_band = true;
//     }
//     else if (!fallback_left_candidates.empty() || !fallback_right_candidates.empty())
//     {
//         selected_index = chooseBalancedSide(
//             fallback_left_candidates,
//             fallback_right_candidates,
//             used_left_side,
//             used_right_side);
//         used_preferred_bounce_band = false;
//     }
//     else if (!fallback_all_candidates.empty())
//     {
//         selected_index = chooseFromCandidates(fallback_all_candidates);
//         used_left_side = angleForIndex(selected_index) >= 0.0;
//         used_right_side = !used_left_side;
//         used_preferred_bounce_band = false;
//     }

//     if (selected_index >= 0)
//     {
//         g_dvd_heading_selection_count++;
//         out_angle = angleForIndex(selected_index);
//         out_range = rangeForLog(selected_index);

//         {
//             std::ofstream log("/tmp/mechelangelo_dvd_bounce_debug.txt");
//             log << std::fixed << std::setprecision(4);
//             log << "=== DVD Bounce Heading Selection ===\n";
//             log << "DVD heading selections so far: " << g_dvd_heading_selection_count << "\n";
//             log << "Successful interaction entries: " << g_human_interaction_success_count << "\n";
//             log << "Completed 30 s interactions: " << g_human_interaction_completed_count << "\n";
//             log << "Abandoned before interaction: " << g_human_abandoned_before_interaction_count << "\n";
//             log << "Abandoned after interaction: " << g_human_abandoned_after_interaction_count << "\n";
//             log << "Cooldown ignored detections: " << g_human_detection_ignored_cooldown_count << "\n\n";
//             log << "Open beam rule: inf OR range >= " << kDvdOpenClearanceDistance << " m\n";
//             log << "Minimum accepted sector width: " << kDvdMinSectorWidth * 180.0 / M_PI << " deg\n";
//             log << "Sector edge margin: " << kDvdSectorEdgeMargin * 180.0 / M_PI << " deg\n";
//             log << "Preferred bounce band: +/-"
//                 << kDvdPreferredMinTurnAngle * 180.0 / M_PI << " to +/-"
//                 << kDvdPreferredMaxTurnAngle * 180.0 / M_PI << " deg\n";
//             log << "Fallback bounce band: +/-"
//                 << kDvdAvoidFrontAngle * 180.0 / M_PI << " to +/-"
//                 << kDvdAvoidReverseAngle * 180.0 / M_PI << " deg\n\n";

//             log << "Accepted open sectors: " << accepted_sector_count << "\n";
//             log << "Widest open sector beams: " << widest_sector_count << "\n";
//             log << "Preferred left candidates: " << preferred_left_candidates.size() << "\n";
//             log << "Preferred right candidates: " << preferred_right_candidates.size() << "\n";
//             log << "Fallback candidates: " << fallback_all_candidates.size() << "\n\n";

//             log << "=== Result ===\n";
//             log << "Selected behaviour: "
//                 << (used_preferred_bounce_band ? "preferred random side-bounce" : "fallback random safe bounce")
//                 << "\n";
//             log << "Selected side: "
//                 << (used_left_side ? "left/positive" : (used_right_side ? "right/negative" : "unknown"))
//                 << "\n";
//             log << "Selected index: " << selected_index << "\n";
//             log << "Selected angle: " << out_angle * 180.0 / M_PI << " deg\n";
//             log << "Representative range: " << out_range << " m\n\n";

//             log << "=== Laser Scan Values ===\n";
//             for (int i = 0; i < scan_count; ++i)
//             {
//                 const double angle_deg = angleForIndex(i) * 180.0 / M_PI;
//                 log << "  [" << std::setw(4) << i << "]  angle="
//                     << std::setw(9) << angle_deg << " deg  range="
//                     << latest_scan_.ranges[i] << " m  open="
//                     << (open_beam[i] ? "yes" : "no") << "  preferred="
//                     << (isPreferredBounceAngle(i) ? "yes" : "no") << "\n";
//             }
//         }

//         return true;
//     }

//     // ------------------------------------------------------
//     // Step 4: fallback to the old longest finite scan if the room is too
//     // cluttered for a safe random bounce heading.
//     // ------------------------------------------------------
//     double max_finite_range = 0.0;
//     int max_finite_index = -1;

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = latest_scan_.ranges[i];

//         if (isRangeValid(range) && range > max_finite_range)
//         {
//             max_finite_range = range;
//             max_finite_index = i;
//         }
//     }

//     if (max_finite_index < 0)
//     {
//         return false;
//     }

//     const double tied_range_threshold =
//         std::max(kMinValidRange, max_finite_range - kLongestRangeTieTolerance);

//     std::vector<bool> near_longest(scan_count, false);

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = latest_scan_.ranges[i];
//         near_longest[i] = isRangeValid(range) && range >= tied_range_threshold;
//     }

//     int best_start_index = max_finite_index;
//     int best_count = 0;

//     for (int start_index = 0; start_index < scan_count; ++start_index)
//     {
//         if (!near_longest[start_index])
//         {
//             continue;
//         }

//         const int previous_index = (start_index - 1 + scan_count) % scan_count;
//         if (near_longest[previous_index])
//         {
//             continue;
//         }

//         int count = 0;
//         while (count < scan_count && near_longest[(start_index + count) % scan_count])
//         {
//             count++;
//         }

//         if (count > best_count)
//         {
//             best_count = count;
//             best_start_index = start_index;
//         }
//     }

//     if (best_count == 0)
//     {
//         best_count = 1;
//         best_start_index = max_finite_index;
//     }

//     const double best_mid_index = std::fmod(
//         static_cast<double>(best_start_index) +
//             0.5 * static_cast<double>(std::max(0, best_count - 1)),
//         static_cast<double>(scan_count));

//     g_dvd_heading_selection_count++;
//     out_angle = normaliseAngle(latest_scan_.angle_min + best_mid_index * latest_scan_.angle_increment);
//     out_range = max_finite_range;

//     {
//         std::ofstream log("/tmp/mechelangelo_dvd_bounce_debug.txt");
//         log << std::fixed << std::setprecision(4);
//         log << "=== DVD Bounce Heading Selection ===\n";
//         log << "DVD heading selections so far: " << g_dvd_heading_selection_count << "\n";
//         log << "Successful interaction entries: " << g_human_interaction_success_count << "\n";
//         log << "Completed 30 s interactions: " << g_human_interaction_completed_count << "\n";
//         log << "Abandoned before interaction: " << g_human_abandoned_before_interaction_count << "\n";
//         log << "Abandoned after interaction: " << g_human_abandoned_after_interaction_count << "\n";
//         log << "Cooldown ignored detections: " << g_human_detection_ignored_cooldown_count << "\n\n";
//         log << "Selected behaviour: fallback longest finite scan\n";
//         log << "Longest finite range: " << out_range << " m\n";
//         log << "Selected angle: " << out_angle * 180.0 / M_PI << " deg\n";
//     }

//     return true;
// }

// int MechelangeloBehaviour::angleToIndex(double angle_rad) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return -1;
//     }

//     double capped_angle = angle_rad;

//     if (capped_angle < latest_scan_.angle_min)
//     {
//         capped_angle = latest_scan_.angle_min;
//     }

//     if (capped_angle > latest_scan_.angle_max)
//     {
//         capped_angle = latest_scan_.angle_max;
//     }

//     int index = static_cast<int>(
//         std::round((capped_angle - latest_scan_.angle_min) / latest_scan_.angle_increment));

//     index = std::clamp(index, 0, static_cast<int>(latest_scan_.ranges.size()) - 1);
//     return index;
// }

// double MechelangeloBehaviour::normaliseAngle(double angle_rad) const
// {
//     while (angle_rad > M_PI)
//     {
//         angle_rad -= 2.0 * M_PI;
//     }

//     while (angle_rad < -M_PI)
//     {
//         angle_rad += 2.0 * M_PI;
//     }

//     return angle_rad;
// }

// bool MechelangeloBehaviour::angleInsideWindow(
//     double angle_rad,
//     double start_angle,
//     double end_angle) const
// {
//     const double angle = normaliseAngle(angle_rad);
//     const double start = normaliseAngle(start_angle);
//     const double end = normaliseAngle(end_angle);

//     if (start <= end)
//     {
//         return angle >= start && angle <= end;
//     }

//     return angle >= start || angle <= end;
// }

// bool MechelangeloBehaviour::segmentOverlapsAngleWindow(
//     const LaserSegment &segment,
//     double start_angle,
//     double end_angle) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     for (int i = segment.start_index; i <= segment.end_index; ++i)
//     {
//         if (i < 0 || i >= static_cast<int>(latest_scan_.ranges.size()))
//         {
//             continue;
//         }

//         const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

//         if (angleInsideWindow(angle, start_angle, end_angle))
//         {
//             return true;
//         }
//     }

//     return false;
// }

// bool MechelangeloBehaviour::findBlockingObstaclesInFront(
//     std::vector<LaserSegment> &blocking_segments) const
// {
//     blocking_segments.clear();

//     for (const LaserSegment &segment : latest_segments_)
//     {
//         if (segment.min_range <= stop_distance_m_ &&
//             segmentOverlapsAngleWindow(segment, -kFrontCheckAngle, kFrontCheckAngle))
//         {
//             blocking_segments.push_back(segment);
//         }
//     }

//     return !blocking_segments.empty();
// }

// void MechelangeloBehaviour::publishObstacleMarkers(
//     const std::vector<LaserSegment> &blocking_segments)
// {
//     visualization_msgs::msg::MarkerArray marker_array;

//     visualization_msgs::msg::Marker clear_marker;
//     clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
//     clear_marker.header.stamp = this->get_clock()->now();
//     clear_marker.ns = "behaviour_blocking_obstacles";
//     clear_marker.id = 0;
//     clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//     marker_array.markers.push_back(clear_marker);

//     int marker_id = 1;

//     for (const LaserSegment &segment : blocking_segments)
//     {
//         visualization_msgs::msg::Marker marker;
//         marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
//         marker.header.stamp = this->get_clock()->now();
//         marker.ns = "behaviour_blocking_obstacles";
//         marker.id = marker_id++;
//         marker.type = visualization_msgs::msg::Marker::CYLINDER;
//         marker.action = visualization_msgs::msg::Marker::ADD;

//         marker.pose.position.x = segment.midpoint.x;
//         marker.pose.position.y = segment.midpoint.y;
//         marker.pose.position.z = 0.15;
//         marker.pose.orientation.x = 0.0;
//         marker.pose.orientation.y = 0.0;
//         marker.pose.orientation.z = 0.0;
//         marker.pose.orientation.w = 1.0;

//         // Make marker size scale slightly with the observed segment length.
//         const double marker_width = std::clamp(segment.length + 0.15, 0.20, 0.80);
//         marker.scale.x = marker_width;
//         marker.scale.y = marker_width;
//         marker.scale.z = 0.30;

//         // Red/orange transparent marker for blocking obstacle.
//         marker.color.a = 0.75F;
//         marker.color.r = 1.0F;
//         marker.color.g = 0.15F;
//         marker.color.b = 0.0F;

//         marker.lifetime.sec = 0;
//         marker.lifetime.nanosec = 400000000; // 0.4 s

//         marker_array.markers.push_back(marker);
//     }

//     obstacle_marker_publisher_->publish(marker_array);
// }

// void MechelangeloBehaviour::clearObstacleMarkers()
// {
//     if (!obstacle_marker_publisher_)
//     {
//         return;
//     }

//     visualization_msgs::msg::MarkerArray marker_array;
//     visualization_msgs::msg::Marker clear_marker;
//     clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
//     clear_marker.header.stamp = this->get_clock()->now();
//     clear_marker.ns = "behaviour_blocking_obstacles";
//     clear_marker.id = 0;
//     clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//     marker_array.markers.push_back(clear_marker);
//     obstacle_marker_publisher_->publish(marker_array);
// }

// void MechelangeloBehaviour::longestLaserScan()
// {
//     double longest_angle = 0.0;
//     double longest_range = 0.0;

//     if (!getLongestRange(longest_angle, longest_range))
//     {
//         RCLCPP_WARN(
//             this->get_logger(),
//             "No valid filtered laser scan data available for longest scan calculation.");
//         return;
//     }

//     RCLCPP_INFO(
//         this->get_logger(),
//         "Selected exploration heading: Representative range = %.2f m at Angle = %.2f degrees",
//         longest_range,
//         longest_angle * 180.0 / M_PI);
// }

// double MechelangeloBehaviour::getHumanLidarRange(double centre_offset) const
// {
//     const double estimated_human_angle = -centre_offset * kCameraHorizontalFov;
//     const double start_angle = estimated_human_angle - kHumanLidarWindow;
//     const double end_angle = estimated_human_angle + kHumanLidarWindow;

//     return getMinimumRange(start_angle, end_angle);
// }

// void MechelangeloBehaviour::captureSafetyZoneBaseline()
// {
//     safety_zone_baseline_scan_ = latest_scan_;
//     safety_zone_baseline_captured_ = true;
//     RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Filtered background baseline captured.");
// }

// bool MechelangeloBehaviour::isSafetyZoneViolated(double human_bearing_rad) const
// {
//     if (!safety_zone_baseline_captured_)
//     {
//         return false;
//     }

//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     if (safety_zone_baseline_scan_.ranges.size() != latest_scan_.ranges.size())
//     {
//         return false;
//     }

//     for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
//     {
//         const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
//         const double angle_diff = normaliseAngle(angle - human_bearing_rad);

//         // Exclude the window around the tracked human so they do not self-trigger.
//         if (std::fabs(angle_diff) <= kSafetyZoneHumanExclusionAngle)
//         {
//             continue;
//         }

//         const double current_range = latest_scan_.ranges[i];

//         if (!std::isfinite(current_range) || current_range <= kMinValidRange)
//         {
//             continue;
//         }

//         const double required_safety_radius = humanModeSafetyRadiusForAngle(angle);

//         if (current_range >= required_safety_radius)
//         {
//             continue;
//         }

//         const double baseline_range = std::isfinite(safety_zone_baseline_scan_.ranges[i])
//             ? safety_zone_baseline_scan_.ranges[i]
//             : required_safety_radius;

//         if (current_range < (baseline_range - kSafetyZoneIntruderThreshold))
//         {
//             return true;
//         }
//     }

//     return false;
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<MechelangeloBehaviour>();

//     // true = simulation mode
//     // false = real robot mode
//     node->run(true);

//     rclcpp::shutdown();
//     return 0;
// }


/////////////////////////////////////////////////////////////////////////
/// DVD bounce + human interaction timer/statistics test

#include "behaviour.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ------------------------------------------------------
// Behaviour constants
// ------------------------------------------------------

// Control loop runs every 100 ms.
static constexpr double kControlPeriodSeconds = 0.1;

// Movement tuning.
static constexpr double kForwardSpeed = 0.26;       // m/s
static constexpr double kTurnSpeed = 0.6;           // rad/s
static constexpr double kAngleGain = 0.8;           // proportional turning gain
static constexpr double kAlignmentTolerance = 0.10; // radians, about 5.7 degrees

// Smooth commanded stops so the physical base does not snap from motion to zero
// in one control tick. At the 100 ms control period, these remove roughly
// 0.04 m/s and 0.12 rad/s from the command each loop.
static constexpr double kStopLinearDecel = 0.4;  // m/s^2
static constexpr double kStopAngularDecel = 1.2; // rad/s^2

// Stop this far before a real obstacle/wall.
// Loaded from the ROS parameter 'stop_distance_m'.
// Default: 1.5 m (simulation). Physical robot: set to 0.75 in the launch file.

// 30 loops x 0.1 s = 3 seconds.
static constexpr int kStopDurationLoops = 30;

// Ignore returns too close to the robot body / lidar blind spot.
static constexpr double kMinValidRange = 0.5; // m

// Front scan window used while moving forward.
static constexpr double kFrontCheckAngle = 30 * M_PI / 180.0; // +/- 30 degrees

// When several neighbouring beams are effectively tied for the longest
// distance, steer toward the middle of that opening instead of whichever beam
// happens to appear first in the scan array.
static constexpr double kLongestRangeTieTolerance = 0.05; // m

// ------------------------------------------------------
// DVD-style bounce exploration tuning
// ------------------------------------------------------
// The autonomous gallery behaviour is intended to move like a DVD screensaver:
// drive straight until the front is blocked, stop at a safe distance, then pick
// a new safe side-bounce angle instead of always chasing the longest wall.
//
// A beam is considered open if it is infinity OR farther than this clearance.
// Infinity is useful in large rooms because it means the LiDAR did not hit
// anything within range.
static constexpr double kDvdOpenClearanceDistance = 4.5; // m

// Ignore very narrow open gaps. This prevents the robot from aiming through
// thin laser cracks between obstacles.
static constexpr double kDvdMinSectorWidth = 18.0 * M_PI / 180.0; // radians

// Trim candidate angles away from the edge of an open sector.
static constexpr double kDvdSectorEdgeMargin = 8.0 * M_PI / 180.0; // radians

// Preferred bounce band after the robot stops at a wall.
// These angles are relative to the robot's current forward direction:
//   0 deg   = back into the wall/obstacle it just stopped for
//   +/-90   = side-bounce
//   +/-180  = drive exactly back along the previous path
// The robot randomly chooses inside this safe side band when possible.
static constexpr double kDvdPreferredMinTurnAngle = 55.0 * M_PI / 180.0;  // radians
static constexpr double kDvdPreferredMaxTurnAngle = 150.0 * M_PI / 180.0; // radians

// Fallback band used if the preferred side-bounce band has no safe candidates.
// This still avoids the front wall and avoids exact reverse.
static constexpr double kDvdAvoidFrontAngle = 35.0 * M_PI / 180.0;  // radians
static constexpr double kDvdAvoidReverseAngle = 165.0 * M_PI / 180.0; // radians

// ------------------------------------------------------
// LaserScan noise suppression constants
// ------------------------------------------------------
// The filter is based on the previous LaserProcessing::countSegments()
// approach: valid objects form segments of neighbouring points. Random
// dots are usually isolated or only one/two points, so they get replaced
// with infinity and will not stop the robot.

// Stage 1: local neighbour test.
static constexpr int kNoiseNeighbourWindow = 4;          // check +/- 4 beams
static constexpr int kNoiseMinNeighbourCount = 2;        // require at least 2 close neighbours
static constexpr double kNoiseNeighbourDistance = 0.22;  // m in local XY space

// Stage 2: segment extraction test.
static constexpr double kSegmentJoinDistance = 0.18;     // m max gap between consecutive points
static constexpr int kSegmentMinPoints = 4;              // reject tiny speckle clusters
static constexpr double kSegmentMinLength = 0.05;        // m reject near-zero length segments

// ------------------------------------------------------
// Human tracking tuning
// ------------------------------------------------------
// /human_tracking message format:
// data[0] = detected, data[1] = centre_offset, data[2] = distance_m
// centre_offset is normalised image offset from centre: -0.5 left, 0 centre, +0.5 right.
static constexpr double kHumanTargetDistance = 1.5;     // m
static constexpr double kHumanDistanceTolerance = 0.15; // m
static constexpr double kHumanMaxForwardSpeed = 0.16;   // m/s, slower approach helps keep person in camera
static constexpr double kHumanMaxReverseSpeed = 0.12;   // m/s
static constexpr double kHumanMaxTurnSpeed = 0.50;      // rad/s, gentler human tracking so the camera does not swing off target
static constexpr double kHumanTurnGain = 1.8;           // image offset to angular speed
static constexpr double kHumanForwardGain = 0.35;       // distance error to linear speed
static constexpr double kHumanCentreDeadZone = 0.06;    // normalised image width
static constexpr double kHumanLostTimeout = 4.0;        // seconds, allow longer brief camera loss before returning to exploration
static constexpr double kHumanRecoveryTurnSpeed = 0.25;   // rad/s, gentle reacquire turn using last known offset
static constexpr double kHumanRecoveryCreepSpeed = 0.00;  // m/s, keep zero while reacquiring to avoid blind motion

// LiDAR validation for human distance.
static constexpr double kCameraHorizontalFov = 60.0 * M_PI / 180.0;
static constexpr double kHumanLidarWindow = 10.0 * M_PI / 180.0;
static constexpr double kLidarCameraMaxDisagreement = 0.4;
static constexpr double kHumanLidarStopDistance = 1.65;
static constexpr double kHumanLidarStopTolerance = 0.20;

// Once the robot reaches a usable interaction pose, hold it instead of
// continuing to creep forward or dropping straight back into DVD exploration
// when the detector flickers for a moment.
static constexpr double kHumanInteractionHoldTimeout = 5.0;       // seconds to hold pose through brief detector loss
static constexpr double kHumanInteractionHoldMaxOffset = 0.18;    // human must be reasonably centred to latch interaction hold
static constexpr double kHumanRecoveryRotateOffsetThreshold = 0.15; // do not rotate during recovery if human was basically centred
static constexpr double kHumanInteractionHoldRangeSlack = 0.25;   // extra distance slack while staying latched

// Human interaction session timing. Once a valid interaction pose is reached,
// hold interaction for this long, then return to DVD exploration and ignore
// camera-triggered human interrupts for a short cooldown period.
static constexpr double kHumanInteractionDurationSeconds = 30.0;
static constexpr double kHumanDetectionCooldownSeconds = 10.0;

// Safety zone.
static constexpr double kSafetyZoneRadius = 1.5;
static constexpr double kSafetyZoneIntruderThreshold = 0.3;
static constexpr double kSafetyZoneHumanExclusionAngle = 25.0 * M_PI / 180.0;

// During HUMAN_DETECTED, the robot is allowed to have a closer wall behind it
// because the arms and human interaction are mainly front/side constrained.
// This avoids false blocking when the base turns imperfectly and the rear of
// the LiDAR/base drifts slightly toward the wall it just stopped near.
static constexpr double kHumanRearSafetyRadius = 1.0; // m, rear-only during human approach/interaction
static constexpr double kHumanRearSectorHalfAngle = 45.0 * M_PI / 180.0; // rear cone around +/-180 deg

// In human reposition mode, the 1.5 m bubble is a requirement for arm interaction,
// not a hard stop for base motion. These smaller radii are the true emergency
// stop distances while the robot is trying to move out of a bad interaction pose.
static constexpr double kHumanModeFrontSideHardStopRadius = 0.85; // m
static constexpr double kHumanModeRearHardStopRadius = 1.0;       // m, keep rear at 1 m as requested

// ------------------------------------------------------
// Human interaction repositioning tuning
// ------------------------------------------------------
// The robot needs a clear space around itself before arm interaction.
// If the human is visible but the 1.5 m arm bubble is blocked by a wall or
// obstacle, the robot does not drive straight at the human. Instead it samples
// short forward/arc moves and chooses the one predicted to clear the bubble
// while still keeping the human in view.
static constexpr double kInteractionBubbleRadius = 1.5;                 // m, arm movement clearance around robot
static constexpr double kInteractionBubbleSafetyMargin = 0.10;          // m, extra buffer added to the bubble check
static constexpr double kInteractionHumanExclusionAngle = 25.0 * M_PI / 180.0; // ignore tracked human cone
static constexpr int kInteractionAllowedBlockedBeams = 3;               // tolerate a few filtered/noisy beams
static constexpr double kInteractionRepositionLookahead = 0.65;         // m, predicted short move distance
static constexpr double kInteractionRepositionSpeed = 0.11;             // m/s, enough movement to escape a bad pose
static constexpr double kInteractionRepositionTurnGain = 0.85;          // rad/s per rad target heading
static constexpr double kInteractionRepositionMaxAngularSpeed = 0.45;   // rad/s, still below normal exploration turning
static constexpr double kInteractionPathHalfWidth = 0.45;               // m, collision corridor half-width
static constexpr double kInteractionPathForwardBuffer = 0.25;           // m, extra forward collision buffer
static constexpr double kInteractionMaxCandidateAngle = 60.0 * M_PI / 180.0;
static constexpr double kInteractionRecenterFirstOffset = 0.38;         // if human is near camera edge, re-centre before repositioning
static constexpr double kInteractionViewLossOffset = 0.48;              // heavy penalty if predicted offset approaches camera edge
static constexpr double kInteractionParallelBiasAngle = 35.0 * M_PI / 180.0; // prefer shallow wall-parallel arcs

// Last confirmed visual human observation. Kept file-scope so this patch does not
// require behaviour.hpp changes. Used to pause/reacquire instead of immediately
// dropping back to DVD exploration when the camera briefly loses the person.
static bool g_last_visible_human_valid = false;
static double g_last_visible_human_centre_offset = 0.0;
static double g_last_visible_human_distance_m = -1.0;
static rclcpp::Time g_last_visible_human_time;

// File-scope latch so this test patch does not require behaviour.hpp changes.
// This becomes active once the robot is centred, at a usable interaction range,
// and has a clear 1.5 m arm bubble.
static bool g_interaction_hold_active = false;
static rclcpp::Time g_interaction_hold_time;
static double g_interaction_hold_range_m = -1.0;
static double g_interaction_hold_offset = 0.0;

// File-scope interaction session/cooldown and long-run stats. Keeping these
// outside the class avoids requiring behaviour.hpp changes for this test.
static bool g_interaction_session_active = false;
static rclcpp::Time g_interaction_session_start_time;
static bool g_human_detection_cooldown_active = false;
static rclcpp::Time g_human_detection_cooldown_start_time;

static int g_dvd_heading_selection_count = 0;
static int g_human_detection_accepted_count = 0;
static int g_human_detection_ignored_cooldown_count = 0;
static int g_human_interaction_success_count = 0;
static int g_human_interaction_completed_count = 0;
static int g_human_abandoned_before_interaction_count = 0;
static int g_human_abandoned_after_interaction_count = 0;

static constexpr const char *kHumanStatsLogFilename =
    "human_interaction_stats.txt";
static constexpr const char *kDvdDebugLogFilename =
    "dvd_bounce_debug.txt";

static std::string behaviourDebugDirectory()
{
    const char *home = std::getenv("HOME");
    std::vector<std::string> candidates;

    if (home != nullptr)
    {
        candidates.push_back(
            std::string(home) +
            "/ros2_ws/src/MECHelangelo/src/mechelangelo_behaviour/debug");
    }

    candidates.push_back("/home/andy/ros2_ws/src/MECHelangelo/src/mechelangelo_behaviour/debug");
    candidates.push_back("/home/pi/ros2_ws/src/MECHelangelo/src/mechelangelo_behaviour/debug");

    for (const std::string &candidate : candidates)
    {
        const std::string test_path = candidate + "/.write_test";
        std::ofstream test_file(test_path, std::ios::app);
        if (test_file.good())
        {
            return candidate;
        }
    }

    // Deliberately do not fall back to /tmp: if this path is wrong, the failed
    // file write makes it obvious that the workspace path needs to be adjusted.
    return "/home/andy/ros2_ws/src/MECHelangelo/src/mechelangelo_behaviour/debug";
}

static std::string debugLogPath(const std::string &filename)
{
    return behaviourDebugDirectory() + "/" + filename;
}

static double normaliseAngleForFileScope(double angle_rad)
{
    while (angle_rad > M_PI)
    {
        angle_rad -= 2.0 * M_PI;
    }

    while (angle_rad < -M_PI)
    {
        angle_rad += 2.0 * M_PI;
    }

    return angle_rad;
}

static bool isRearBeamAngle(double angle_rad)
{
    const double angle = normaliseAngleForFileScope(angle_rad);
    return std::fabs(angle) >= (M_PI - kHumanRearSectorHalfAngle);
}

static double humanModeSafetyRadiusForAngle(double angle_rad)
{
    // During HUMAN_DETECTED this is only the hard-stop safety radius for base
    // repositioning. The full arm-clearance radius is checked separately by
    // humanModeInteractionBubbleRadiusForAngle().
    return isRearBeamAngle(angle_rad)
        ? kHumanModeRearHardStopRadius
        : kHumanModeFrontSideHardStopRadius;
}

static double humanModeInteractionBubbleRadiusForAngle(double angle_rad)
{
    return isRearBeamAngle(angle_rad)
        ? kHumanRearSafetyRadius
        : (kInteractionBubbleRadius + kInteractionBubbleSafetyMargin);
}

static bool humanDetectionCooldownActive(const rclcpp::Time &now)
{
    if (!g_human_detection_cooldown_active)
    {
        return false;
    }

    return (now - g_human_detection_cooldown_start_time).seconds() <
        kHumanDetectionCooldownSeconds;
}

static double humanDetectionCooldownRemaining(const rclcpp::Time &now)
{
    if (!g_human_detection_cooldown_active)
    {
        return 0.0;
    }

    const double elapsed = (now - g_human_detection_cooldown_start_time).seconds();
    return std::max(0.0, kHumanDetectionCooldownSeconds - elapsed);
}

static void writeHumanInteractionStatsLog(
    const std::string &event,
    const rclcpp::Time &now)
{
    std::ofstream log(debugLogPath(kHumanStatsLogFilename));
    log << std::fixed << std::setprecision(3);
    log << "=== Mechelangelo Human Interaction Stats ===\n";
    log << "Last event: " << event << "\n";
    log << "ROS time: " << now.seconds() << " s\n\n";

    log << "DVD exploration headings selected: " << g_dvd_heading_selection_count << "\n";
    log << "Human detections accepted: " << g_human_detection_accepted_count << "\n";
    log << "Human detections ignored during cooldown: "
        << g_human_detection_ignored_cooldown_count << "\n\n";

    log << "Successful entries into interaction state: "
        << g_human_interaction_success_count << "\n";
    log << "Completed 30 s interactions: "
        << g_human_interaction_completed_count << "\n";
    log << "Abandoned before reaching interaction: "
        << g_human_abandoned_before_interaction_count << "\n";
    log << "Abandoned after reaching interaction: "
        << g_human_abandoned_after_interaction_count << "\n\n";

    log << "Interaction session active: "
        << (g_interaction_session_active ? "yes" : "no") << "\n";
    if (g_interaction_session_active)
    {
        log << "Interaction elapsed: "
            << (now - g_interaction_session_start_time).seconds() << " / "
            << kHumanInteractionDurationSeconds << " s\n";
    }

    log << "Human detection cooldown active: "
        << (humanDetectionCooldownActive(now) ? "yes" : "no") << "\n";
    if (humanDetectionCooldownActive(now))
    {
        log << "Cooldown remaining: "
            << humanDetectionCooldownRemaining(now) << " s\n";
    }

    log << "\nRear safety radius during HUMAN_DETECTED: "
        << kHumanRearSafetyRadius << " m for rear +/-"
        << kHumanRearSectorHalfAngle * 180.0 / M_PI << " deg around 180 deg\n";
    log << "Front/side safety radius during HUMAN_DETECTED: "
        << kSafetyZoneRadius << " m\n";
}

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

    RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_m_);

    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&MechelangeloBehaviour::laserScanCallback, this, std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        rclcpp::SensorDataQoS(),
        std::bind(&MechelangeloBehaviour::imuCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);

    filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan_filtered",
        rclcpp::SensorDataQoS());

    obstacle_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/behaviour_obstacle_markers",
        10);

    human_detected_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/human_detected",
        10,
        std::bind(&MechelangeloBehaviour::humanDetectedCallback, this, std::placeholders::_1));

    human_tracking_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/human_tracking",
        10,
        std::bind(&MechelangeloBehaviour::humanTrackingCallback, this, std::placeholders::_1));

    last_human_tracking_time_ = this->now();
    g_last_visible_human_time = this->now();
    g_last_visible_human_valid = false;
    g_last_visible_human_centre_offset = 0.0;
    g_last_visible_human_distance_m = -1.0;

    control_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&MechelangeloBehaviour::controlLoop, this));

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
    g_interaction_hold_active = false;
    clearObstacleMarkers();
}

void MechelangeloBehaviour::mappedAutonomous()
{
    RCLCPP_INFO(this->get_logger(), "Executing mapped autonomous behaviour.");
}

void MechelangeloBehaviour::controlLoop()
{
    if (!blind_autonomous_active_)
    {
        return;
    }

    geometry_msgs::msg::Twist twist;

    // Safety: wait until valid LaserScan data exists.
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Waiting for valid filtered LaserScan data...");

        stopRobot(twist);
        current_twist_ = twist;
        cmd_vel_publisher_->publish(twist);
        return;
    }

    switch (current_state_)
    {
    case NavigationState::SEARCHING:
    {
        stopRobot(twist);
        clearObstacleMarkers();

        double longest_angle = 0.0;
        double longest_range = 0.0;

        if (!getLongestRange(longest_angle, longest_range))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "SEARCHING: No valid filtered LaserScan range found.");
            break;
        }

        target_angle_ = longest_angle;
        target_range_ = longest_range;
        align_yaw_initialised_ = false;

        RCLCPP_INFO(
            this->get_logger(),
            "SEARCHING: Selected exploration heading at %.2f deg, representative range %.2f m",
            target_angle_ * 180.0 / M_PI,
            target_range_);

        current_state_ = NavigationState::ALIGNING;
        break;
    }

    case NavigationState::ALIGNING:
    {
        clearObstacleMarkers();
        twist.linear.x = 0.0;

        if (!imu_available_)
        {
            // IMU not yet publishing — fall back to open-loop time integration.
            if (std::fabs(target_angle_) <= kAlignmentTolerance)
            {
                stopRobot(twist);
                if (std::fabs(twist.angular.z) <= 1e-6)
                {
                    RCLCPP_INFO(this->get_logger(),
                        "ALIGNING (open-loop): Aligned. Starting forward movement.");
                    current_state_ = NavigationState::MOVING;
                }
                break;
            }

            const double turn_cmd = std::clamp(
                target_angle_ * kAngleGain, -kTurnSpeed, kTurnSpeed);
            twist.angular.z = turn_cmd;
            target_angle_ -= turn_cmd * kControlPeriodSeconds;
            target_angle_ = normaliseAngle(target_angle_);

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "ALIGNING: No IMU data yet — using open-loop time estimate. "
                "Remaining %.2f deg", target_angle_ * 180.0 / M_PI);
            break;
        }

        // IMU-confirmed alignment: compare actual yaw turned vs. required angle.
        tf2::Quaternion q;
        tf2::fromMsg(latest_imu_.orientation, q);
        const double current_yaw = tf2::getYaw(q);

        if (!align_yaw_initialised_)
        {
            align_start_yaw_ = current_yaw;
            align_yaw_initialised_ = true;
        }

        // How much has the robot actually rotated since ALIGNING began.
        const double yaw_turned = normaliseAngle(current_yaw - align_start_yaw_);

        // How many degrees still remain.
        const double remaining_angle = normaliseAngle(target_angle_ - yaw_turned);

        if (std::fabs(remaining_angle) <= kAlignmentTolerance)
        {
            stopRobot(twist);
            if (std::fabs(twist.angular.z) <= 1e-6)
            {
                RCLCPP_INFO(this->get_logger(),
                    "ALIGNING: IMU confirmed rotation. Turned %.2f deg (target %.2f deg). Starting forward movement.",
                    yaw_turned * 180.0 / M_PI,
                    target_angle_ * 180.0 / M_PI);
                current_state_ = NavigationState::MOVING;
            }
            break;
        }

        const double turn_cmd = std::clamp(
            remaining_angle * kAngleGain, -kTurnSpeed, kTurnSpeed);
        twist.angular.z = turn_cmd;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "ALIGNING: IMU-confirmed. Target %.2f deg, turned %.2f deg, remaining %.2f deg, cmd %.2f rad/s",
            target_angle_ * 180.0 / M_PI,
            yaw_turned * 180.0 / M_PI,
            remaining_angle * 180.0 / M_PI,
            twist.angular.z);

        break;
    }

    case NavigationState::MOVING:
    {
        std::vector<LaserSegment> blocking_segments;
        const bool blocked_by_segment = findBlockingObstaclesInFront(blocking_segments);
        const double front_range = getFrontRange();

        // Segment-based blocking is the main decision. This prevents a single
        // random dot from stopping the robot because the dot will not survive
        // the neighbour + segment filter.
        if (blocked_by_segment || front_range <= stop_distance_m_)
        {
            if (blocking_segments.empty())
            {
                // Fallback marker if the range check caught something but no
                // segment was available. This should be rare after filtering.
                LaserSegment fallback;
                fallback.point_count = 1;
                fallback.min_range = front_range;
                fallback.midpoint.x = std::isfinite(front_range) ? front_range : stop_distance_m_;
                fallback.midpoint.y = 0.0;
                fallback.midpoint.z = 0.0;
                blocking_segments.push_back(fallback);
            }

            publishObstacleMarkers(blocking_segments);

            RCLCPP_WARN(
                this->get_logger(),
                "MOVING: Blocking obstacle detected in front. Front range = %.2f m. Stopping.",
                front_range);

            stopRobot(twist);
            stop_counter_ = 0;
            current_state_ = NavigationState::STOPPED;
            break;
        }

        clearObstacleMarkers();

        if (std::isinf(front_range))
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "MOVING: Front is clear after filtering. Driving forward.");
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "MOVING: Driving forward. Filtered front range = %.2f m",
                front_range);
        }

        twist.linear.x = kForwardSpeed;
        twist.angular.z = 0.0;
        break;
    }

    case NavigationState::STOPPED:
    {
        stopRobot(twist);
        stop_counter_++;

        if (stop_counter_ >= kStopDurationLoops)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "STOPPED: Pause complete. Searching for next direction.");

            stop_counter_ = 0;
            clearObstacleMarkers();
            current_state_ = NavigationState::SEARCHING;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "STOPPED: Pausing %.1f / %.1f seconds",
                stop_counter_ * kControlPeriodSeconds,
                kStopDurationLoops * kControlPeriodSeconds);
        }

        break;
    }

    case NavigationState::HUMAN_DETECTED:
    {
        const double time_since_tracking =
            (this->now() - last_human_tracking_time_).seconds();
        const double time_since_visible_human = g_last_visible_human_valid
            ? (this->now() - g_last_visible_human_time).seconds()
            : std::numeric_limits<double>::infinity();

        // If the robot has successfully reached interaction state, hold the
        // interaction pose for a fixed session time, then move on and ignore
        // camera-triggered human interrupts for a short cooldown period.
        if (g_interaction_session_active)
        {
            const double interaction_elapsed =
                (this->now() - g_interaction_session_start_time).seconds();

            if (interaction_elapsed >= kHumanInteractionDurationSeconds)
            {
                g_human_interaction_completed_count++;
                g_interaction_session_active = false;
                g_interaction_hold_active = false;
                g_human_detection_cooldown_active = true;
                g_human_detection_cooldown_start_time = this->now();
                human_locked_ = false;
                safety_zone_violated_ = false;
                safety_zone_baseline_captured_ = false;
                clearObstacleMarkers();
                stopRobot(twist);
                current_state_ = NavigationState::SEARCHING;

                writeHumanInteractionStatsLog("completed_30s_interaction_started_cooldown", this->now());

                RCLCPP_INFO(
                    this->get_logger(),
                    "HUMAN_SESSION: Completed %.1f s interaction. Returning to DVD exploration with %.1f s human-detection cooldown.",
                    kHumanInteractionDurationSeconds,
                    kHumanDetectionCooldownSeconds);
                break;
            }
        }

        // If the detector briefly drops the human while we are correcting near a wall,
        // do not instantly return to DVD exploration. Hold/reacquire using the last
        // confirmed camera offset so the interaction has a chance to recover.
        if (!human_locked_)
        {
            const double hold_age = g_interaction_hold_active
                ? (this->now() - g_interaction_hold_time).seconds()
                : std::numeric_limits<double>::infinity();

            // If we had already reached a valid interaction pose, do not rotate
            // or return to DVD exploration just because the detector flickers.
            // Hold still so the camera has a chance to reacquire the person.
            if (g_interaction_hold_active && hold_age <= kHumanInteractionHoldTimeout)
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "HUMAN_HOLD: Human briefly lost %.2f s after reaching interaction pose. Holding still at last range %.2f m.",
                    hold_age,
                    g_interaction_hold_range_m);
                break;
            }

            if (time_since_visible_human <= kHumanLostTimeout)
            {
                twist.linear.x = kHumanRecoveryCreepSpeed;

                if (std::fabs(g_last_visible_human_centre_offset) <= kHumanRecoveryRotateOffsetThreshold)
                {
                    // The person was last seen near the centre, so rotating can make
                    // reacquisition worse. Hold still instead.
                    twist.angular.z = 0.0;
                }
                else
                {
                    twist.angular.z = std::clamp(
                        -kHumanTurnGain * g_last_visible_human_centre_offset,
                        -kHumanRecoveryTurnSpeed,
                        kHumanRecoveryTurnSpeed);
                }

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "HUMAN_RECOVERY: Human briefly lost %.2f s ago. Last offset %.2f, cmd angular=%.2f",
                    time_since_visible_human,
                    g_last_visible_human_centre_offset,
                    twist.angular.z);
                break;
            }

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "HUMAN_DETECTED: Human lost for %.2f s. Returning to blind autonomous search.",
                time_since_visible_human);

            if (g_interaction_session_active || g_interaction_hold_active)
            {
                g_human_abandoned_after_interaction_count++;
            }
            else
            {
                g_human_abandoned_before_interaction_count++;
            }

            writeHumanInteractionStatsLog("human_lost_returning_to_exploration", this->now());

            stopRobot(twist);
            human_locked_ = false;
            safety_zone_violated_ = false;
            safety_zone_baseline_captured_ = false;
            g_interaction_hold_active = false;
            g_interaction_session_active = false;
            clearObstacleMarkers();
            current_state_ = NavigationState::SEARCHING;
            break;
        }

        if (time_since_tracking > kHumanLostTimeout)
        {
            const double hold_age = g_interaction_hold_active
                ? (this->now() - g_interaction_hold_time).seconds()
                : std::numeric_limits<double>::infinity();

            if (g_interaction_hold_active && hold_age <= kHumanInteractionHoldTimeout)
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "HUMAN_HOLD: Tracking stale %.2f s, but interaction pose was recently reached. Holding still.",
                    time_since_tracking);
                break;
            }

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "HUMAN_DETECTED: Tracking stale for %.2f s. Returning to blind autonomous search.",
                time_since_tracking);

            if (g_interaction_session_active || g_interaction_hold_active)
            {
                g_human_abandoned_after_interaction_count++;
            }
            else
            {
                g_human_abandoned_before_interaction_count++;
            }

            writeHumanInteractionStatsLog("human_lost_returning_to_exploration", this->now());

            stopRobot(twist);
            human_locked_ = false;
            safety_zone_violated_ = false;
            safety_zone_baseline_captured_ = false;
            g_interaction_hold_active = false;
            g_interaction_session_active = false;
            clearObstacleMarkers();
            current_state_ = NavigationState::SEARCHING;
            break;
        }

        if (!safety_zone_baseline_captured_)
        {
            captureSafetyZoneBaseline();
        }

        const double human_bearing_rad = -human_centre_offset_ * kCameraHorizontalFov;
        const bool zone_now_violated = isSafetyZoneViolated(human_bearing_rad);

        if (zone_now_violated != safety_zone_violated_)
        {
            safety_zone_violated_ = zone_now_violated;

            if (safety_zone_violated_)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "SAFETY ZONE: Hard-stop object inside human reposition safety zone. Front/sides %.2f m, rear %.2f m. Motion paused.",
                    kHumanModeFrontSideHardStopRadius,
                    kHumanModeRearHardStopRadius);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Clear. Resuming interaction.");
            }
        }

        if (safety_zone_violated_)
        {
            stopRobot(twist);
            break;
        }

        // ------------------------------------------------------
        // Human tracking + interaction-space behaviour
        // ------------------------------------------------------
        // The normal human approach command keeps the person centred and moves
        // to the target distance. The extra logic below checks whether there is
        // enough 360-degree clearance for the arms. If not, the robot chooses a
        // short safe arc that is predicted to improve the 1.5 m interaction
        // bubble while still keeping the human in view.

        const double human_keep_turn =
            (std::fabs(human_centre_offset_) <= kHumanCentreDeadZone)
                ? 0.0
                : std::clamp(
                    -kHumanTurnGain * human_centre_offset_,
                    -kHumanMaxTurnSpeed,
                    kHumanMaxTurnSpeed);

        const double human_lidar_range = getHumanLidarRange(human_centre_offset_);
                        
        const bool lidar_distance_valid =
            std::isfinite(human_lidar_range) &&
            human_lidar_range > kMinValidRange;
                        
        // The Pi camera has no depth.
        // The bridge intentionally sends human_distance_m_ = -1.0.
        // Therefore the robot must NEVER use camera distance or kHumanTargetDistance
        // as a movement distance. LiDAR is the only distance source.
        if (!lidar_distance_valid)
        {
            twist.linear.x = 0.0;
            twist.angular.z = human_keep_turn;
        
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "HUMAN_DETECTED: Camera sees human, but no valid LiDAR range near bearing. Turning only. offset=%.2f",
                human_centre_offset_);
            
            break;
        }
        
        const double estimated_human_range = human_lidar_range;

        const double human_x = estimated_human_range * std::cos(human_bearing_rad);
        const double human_y = estimated_human_range * std::sin(human_bearing_rad);

        struct InteractionBubbleCheck
        {
            int considered_beams = 0;
            int blocked_beams = 0;
            double min_clearance = std::numeric_limits<double>::infinity();
            double blocked_fraction = 0.0;
            bool clear = false;
        };

        auto evaluateInteractionBubbleAt = [&](double future_x, double future_y)
        {
            InteractionBubbleCheck check;

            if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
            {
                check.clear = false;
                check.blocked_beams = 9999;
                check.blocked_fraction = 1.0;
                check.min_clearance = 0.0;
                return check;
            }

            for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
            {
                const double range = latest_scan_.ranges[i];

                if (!std::isfinite(range) || range <= kMinValidRange)
                {
                    continue;
                }

                const double angle = latest_scan_.angle_min +
                    static_cast<double>(i) * latest_scan_.angle_increment;
                const double angle_diff_to_human = normaliseAngle(angle - human_bearing_rad);

                // The tracked human is allowed to be in the front interaction
                // window. Everything else inside the arm bubble blocks interaction.
                if (std::fabs(angle_diff_to_human) <= kInteractionHumanExclusionAngle)
                {
                    continue;
                }

                const double point_x = range * std::cos(angle);
                const double point_y = range * std::sin(angle);
                const double distance_to_future_robot =
                    std::hypot(point_x - future_x, point_y - future_y);

                check.considered_beams++;
                check.min_clearance = std::min(check.min_clearance, distance_to_future_robot);

                const double required_radius = humanModeInteractionBubbleRadiusForAngle(angle);

                if (distance_to_future_robot < required_radius)
                {
                    check.blocked_beams++;
                }
            }

            if (check.considered_beams > 0)
            {
                check.blocked_fraction = static_cast<double>(check.blocked_beams) /
                    static_cast<double>(check.considered_beams);
            }
            else
            {
                // No finite obstacles outside the human window means the bubble
                // is clear as far as LiDAR can tell.
                check.blocked_fraction = 0.0;
            }

            check.clear = check.blocked_beams <= kInteractionAllowedBlockedBeams;
            return check;
        };

        auto pathToFuturePoseIsClear = [&](double heading, double move_distance)
        {
            if (move_distance <= 1e-3)
            {
                return true;
            }

            const double dir_x = std::cos(heading);
            const double dir_y = std::sin(heading);
            const double max_forward = move_distance + kInteractionPathForwardBuffer;

            for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
            {
                const double range = latest_scan_.ranges[i];

                if (!std::isfinite(range) || range <= kMinValidRange)
                {
                    continue;
                }

                const double angle = latest_scan_.angle_min +
                    static_cast<double>(i) * latest_scan_.angle_increment;
                const double angle_diff_to_human = normaliseAngle(angle - human_bearing_rad);

                if (std::fabs(angle_diff_to_human) <= kInteractionHumanExclusionAngle)
                {
                    continue;
                }

                const double point_x = range * std::cos(angle);
                const double point_y = range * std::sin(angle);

                const double forward = point_x * dir_x + point_y * dir_y;
                const double lateral = -point_x * dir_y + point_y * dir_x;

                if (forward > 0.0 && forward < max_forward &&
                    std::fabs(lateral) < kInteractionPathHalfWidth)
                {
                    return false;
                }
            }

            return true;
        };

        const InteractionBubbleCheck current_bubble = evaluateInteractionBubbleAt(0.0, 0.0);
        const bool interaction_bubble_clear = current_bubble.clear;

        if (lidar_distance_valid &&
            human_distance_m_ > 0.0 &&
            std::isfinite(human_distance_m_))
        {
            const double disagreement = std::fabs(human_distance_m_ - human_lidar_range);

            if (disagreement > kLidarCameraMaxDisagreement)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "HUMAN_DETECTED: Camera/LiDAR distance disagreement. Camera=%.2f m, LiDAR=%.2f m",
                    human_distance_m_,
                    human_lidar_range);
            }
        }

        const double distance_error = estimated_human_range - kHumanLidarStopDistance;
        const bool at_human_target_distance =
            std::fabs(distance_error) <= kHumanLidarStopTolerance;

        const bool human_centred_for_interaction =
            std::fabs(human_centre_offset_) <= kHumanInteractionHoldMaxOffset;

        const bool interaction_pose_valid =
            interaction_bubble_clear &&
            at_human_target_distance &&
            human_centred_for_interaction;

        if (interaction_pose_valid)
        {
            if (!g_interaction_session_active)
            {
                g_interaction_session_active = true;
                g_interaction_session_start_time = this->now();
                g_human_interaction_success_count++;
                writeHumanInteractionStatsLog("entered_interaction_state_successfully", this->now());

                RCLCPP_INFO(
                    this->get_logger(),
                    "HUMAN_SESSION: Entered interaction state successfully. Count=%d. Holding for %.1f s.",
                    g_human_interaction_success_count,
                    kHumanInteractionDurationSeconds);
            }

            g_interaction_hold_active = true;
            g_interaction_hold_time = this->now();
            g_interaction_hold_range_m = estimated_human_range;
            g_interaction_hold_offset = human_centre_offset_;
        }
        else if (g_interaction_hold_active)
        {
            const bool still_close_to_hold_range =
                std::fabs(estimated_human_range - kHumanLidarStopDistance) <=
                    (kHumanLidarStopTolerance + kHumanInteractionHoldRangeSlack);

            if (!interaction_bubble_clear ||
                !still_close_to_hold_range ||
                std::fabs(human_centre_offset_) > (kHumanInteractionHoldMaxOffset + 0.12))
            {
                g_interaction_hold_active = false;
            }
        }

        // If the robot is at interaction distance but the arm bubble is blocked,
        // do not enter arm interaction. Reposition instead.
        const bool needs_interaction_reposition = !interaction_bubble_clear;

        // If the person is already close to the edge of the camera image, do not
        // start a wall-clearance arc yet. First slow down and re-centre them. This
        // prevents the robot from choosing a 30-45 degree clearance manoeuvre that
        // swings the human out of view.
        if (needs_interaction_reposition &&
            std::fabs(human_centre_offset_) >= kInteractionRecenterFirstOffset)
        {
            twist.linear.x = 0.0;
            twist.angular.z = std::clamp(
                human_keep_turn,
                -kInteractionRepositionMaxAngularSpeed,
                kInteractionRepositionMaxAngularSpeed);

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "HUMAN_REPOSITION: Arm bubble blocked, but human is near image edge (offset %.2f). Recentring first. cmd angular=%.2f",
                human_centre_offset_,
                twist.angular.z);
            break;
        }

        if (needs_interaction_reposition)
        {
            struct RepositionCandidate
            {
                double heading = 0.0;
                double move_distance = 0.0;
                double linear = 0.0;
                double angular = 0.0;
                double score = -std::numeric_limits<double>::infinity();
                InteractionBubbleCheck bubble;
                bool path_clear = false;
            };

            const std::vector<double> candidate_headings = {
                -60.0 * M_PI / 180.0,
                -50.0 * M_PI / 180.0,
                -40.0 * M_PI / 180.0,
                -30.0 * M_PI / 180.0,
                -20.0 * M_PI / 180.0,
                -10.0 * M_PI / 180.0,
                  0.0,
                 10.0 * M_PI / 180.0,
                 20.0 * M_PI / 180.0,
                 30.0 * M_PI / 180.0,
                 40.0 * M_PI / 180.0,
                 50.0 * M_PI / 180.0,
                 60.0 * M_PI / 180.0
            };

            RepositionCandidate best_candidate;
            const double required_radius =
                kInteractionBubbleRadius + kInteractionBubbleSafetyMargin;

            // Build a simple "escape vector" away from whatever is blocking the
            // 1.5 m interaction bubble. This stops the robot from repeatedly
            // choosing tiny 0-8 degree movements when it actually needs to arc
            // away from a wall or object beside the human.
            double escape_x = 0.0;
            double escape_y = 0.0;
            double escape_weight_sum = 0.0;

            for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
            {
                const double range = latest_scan_.ranges[i];
                if (!std::isfinite(range) || range <= kMinValidRange)
                {
                    continue;
                }

                const double angle = latest_scan_.angle_min +
                    static_cast<double>(i) * latest_scan_.angle_increment;
                const double angle_diff_to_human = normaliseAngle(angle - human_bearing_rad);
                if (std::fabs(angle_diff_to_human) <= kInteractionHumanExclusionAngle)
                {
                    continue;
                }

                const double required_at_angle = humanModeInteractionBubbleRadiusForAngle(angle);
                if (range >= required_at_angle)
                {
                    continue;
                }

                const double closeness = std::clamp(
                    (required_at_angle - range) / std::max(0.01, required_at_angle),
                    0.0,
                    1.0);
                const double weight = 0.25 + closeness;

                // Obstacle point direction from robot. Move opposite this vector.
                escape_x -= weight * std::cos(angle);
                escape_y -= weight * std::sin(angle);
                escape_weight_sum += weight;
            }

            const bool escape_vector_valid = escape_weight_sum > 1e-3;
            const double escape_angle = escape_vector_valid
                ? std::atan2(escape_y, escape_x)
                : 0.0;
            const double escape_strength = escape_vector_valid
                ? std::min(1.0, std::hypot(escape_x, escape_y) / escape_weight_sum)
                : 0.0;

            for (const double heading : candidate_headings)
            {
                const double abs_heading = std::fabs(heading);

                // More sideways arcs move a little less in one decision step.
                const double move_scale = std::clamp(
                    1.0 - 0.45 * (abs_heading / kInteractionMaxCandidateAngle),
                    0.45,
                    1.0);
                const double move_distance = kInteractionRepositionLookahead * move_scale;
                const double future_x = move_distance * std::cos(heading);
                const double future_y = move_distance * std::sin(heading);

                RepositionCandidate candidate;
                candidate.heading = heading;
                candidate.move_distance = move_distance;
                candidate.path_clear = pathToFuturePoseIsClear(heading, move_distance);
                candidate.bubble = evaluateInteractionBubbleAt(future_x, future_y);

                const double future_human_bearing = normaliseAngle(
                    std::atan2(human_y - future_y, human_x - future_x));
                const double future_human_distance =
                    std::hypot(human_x - future_x, human_y - future_y);

                // Estimate where the person would sit in the camera after the short
                // reposition. This is approximate, but it lets us heavily reject moves
                // that are likely to push the human out of frame.
                const double predicted_offset = std::clamp(
                    -future_human_bearing / kCameraHorizontalFov,
                    -0.75,
                    0.75);
                const double human_center_score = std::clamp(
                    1.0 - std::fabs(predicted_offset) / 0.45,
                    0.0,
                    1.0);
                const double human_distance_score = std::clamp(
                    1.0 - std::fabs(future_human_distance - kHumanLidarStopDistance) / 1.0,
                    0.0,
                    1.0);
                const double clearance_score = std::clamp(
                    candidate.bubble.min_clearance / required_radius,
                    0.0,
                    1.4);
                const double blocked_score = 1.0 - std::clamp(
                    candidate.bubble.blocked_fraction,
                    0.0,
                    1.0);
                const double human_view_penalty = std::clamp(
                    (std::fabs(predicted_offset) - kInteractionRecenterFirstOffset) /
                        (kInteractionViewLossOffset - kInteractionRecenterFirstOffset),
                    0.0,
                    1.0);
                const double parallel_arc_score = std::clamp(
                    1.0 - std::fabs(abs_heading - kInteractionParallelBiasAngle) /
                        kInteractionParallelBiasAngle,
                    0.0,
                    1.0);

                const double blocked_improvement_score =
                    static_cast<double>(current_bubble.blocked_beams - candidate.bubble.blocked_beams) /
                    static_cast<double>(std::max(1, current_bubble.blocked_beams));
                const double clearance_improvement_score = std::clamp(
                    (candidate.bubble.min_clearance - current_bubble.min_clearance) / 0.35,
                    -1.0,
                    1.0);
                const double escape_alignment_score = escape_vector_valid
                    ? std::clamp(0.5 + 0.5 * std::cos(normaliseAngle(heading - escape_angle)), 0.0, 1.0)
                    : 0.0;

                // During repositioning, the aim is not to prove the bubble is already
                // clear. It is to choose motion that makes the blocked bubble better
                // without losing the human. This gives strong weight to movement away
                // from the blocked side, while still penalising camera-view loss.
                candidate.score =
                    6.0 * blocked_score +
                    2.0 * clearance_score +
                    4.5 * human_center_score +
                    0.8 * human_distance_score +
                    1.0 * parallel_arc_score +
                    7.0 * blocked_improvement_score +
                    5.0 * clearance_improvement_score +
                    4.0 * escape_strength * escape_alignment_score -
                    1.2 * (abs_heading / kInteractionMaxCandidateAngle) -
                    9.0 * human_view_penalty;

                if (!candidate.path_clear)
                {
                    candidate.score -= 20.0;
                }

                const double candidate_turn = std::clamp(
                    candidate.heading * kInteractionRepositionTurnGain,
                    -kInteractionRepositionMaxAngularSpeed,
                    kInteractionRepositionMaxAngularSpeed);

                // Blend the clearance escape turn with human-centering. The previous
                // version weighted human centering too strongly, so it kept choosing
                // tiny 8 degree corrections and never escaped the wall-side edge case.
                candidate.angular = std::clamp(
                    0.65 * candidate_turn + 0.35 * human_keep_turn,
                    -kInteractionRepositionMaxAngularSpeed,
                    kInteractionRepositionMaxAngularSpeed);

                candidate.linear = std::clamp(
                    kInteractionRepositionSpeed * move_scale,
                    0.05,
                    kInteractionRepositionSpeed);

                if (candidate.score > best_candidate.score)
                {
                    best_candidate = candidate;
                }
            }

            if (best_candidate.score > -1.0)
            {
                twist.linear.x = best_candidate.linear;
                twist.angular.z = best_candidate.angular;

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    750,
                    "HUMAN_REPOSITION: Interaction bubble blocked (%d beams, min %.2f m). "
                    "Escape %.1f deg strength %.2f. Best heading %.1f deg -> future blocked %d beams, min %.2f m. cmd linear=%.2f angular=%.2f",
                    current_bubble.blocked_beams,
                    current_bubble.min_clearance,
                    escape_angle * 180.0 / M_PI,
                    escape_strength,
                    best_candidate.heading * 180.0 / M_PI,
                    best_candidate.bubble.blocked_beams,
                    best_candidate.bubble.min_clearance,
                    twist.linear.x,
                    twist.angular.z);
            }
            else
            {
                // If there is no safe reposition movement, at least keep the
                // human in view and do not enter arm interaction.
                twist.linear.x = 0.0;
                twist.angular.z = std::clamp(human_keep_turn, -kInteractionRepositionMaxAngularSpeed, kInteractionRepositionMaxAngularSpeed);

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "HUMAN_REPOSITION: No safe movement found to clear arm bubble. Holding and keeping human centred.");
            }

            break;
        }

        // Arm bubble is clear, so normal human approach/hold behaviour can run.
        // If we have latched a good interaction pose, stay still. This prevents
        // the robot from creeping forward and losing the detector right when it
        // reaches the usable arm-interaction distance.
        if (g_interaction_hold_active && interaction_pose_valid)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            const double interaction_elapsed = g_interaction_session_active
                ? (this->now() - g_interaction_session_start_time).seconds()
                : 0.0;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "HUMAN_HOLD: Interaction pose valid. Holding at range %.2f m, offset %.2f, bubble min %.2f m. Session %.1f / %.1f s.",
                estimated_human_range,
                human_centre_offset_,
                current_bubble.min_clearance,
                interaction_elapsed,
                kHumanInteractionDurationSeconds);
            break;
        }

        twist.angular.z = human_keep_turn;

        if (!lidar_distance_valid && !(human_distance_m_ > 0.0 && std::isfinite(human_distance_m_)))
        {
            twist.linear.x = 0.0;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "HUMAN_DETECTED: No valid distance estimate. Turning only to keep human centred.");
        }
        else if (at_human_target_distance)
        {
            twist.linear.x = 0.0;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "HUMAN_DETECTED: Human at target distance %.2f m and 1.5 m arm bubble clear. Ready for interaction.",
                estimated_human_range);
        }
        else
        {
            twist.linear.x = std::clamp(
                kHumanForwardGain * distance_error,
                -kHumanMaxReverseSpeed,
                kHumanMaxForwardSpeed);
        }

        if (estimated_human_range < kHumanLidarStopDistance && twist.linear.x > 0.0)
        {
            twist.linear.x = 0.0;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "HUMAN_DETECTED: offset=%.2f distance=%.2f arm_clear=%s blocked=%d min_clearance=%.2f cmd linear=%.2f angular=%.2f",
            human_centre_offset_,
            estimated_human_range,
            interaction_bubble_clear ? "yes" : "no",
            current_bubble.blocked_beams,
            current_bubble.min_clearance,
            twist.linear.x,
            twist.angular.z);

        break;
    }

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

void MechelangeloBehaviour::stopRobot(geometry_msgs::msg::Twist &twist)
{
    const double linear_step = kStopLinearDecel * kControlPeriodSeconds;
    const double angular_step = kStopAngularDecel * kControlPeriodSeconds;

    auto rampTowardZero = [](double value, double max_step)
    {
        if (std::fabs(value) <= max_step)
        {
            return 0.0;
        }

        return value - std::copysign(max_step, value);
    };

    twist.linear.x = rampTowardZero(current_twist_.linear.x, linear_step);
    twist.linear.y = rampTowardZero(current_twist_.linear.y, linear_step);
    twist.linear.z = rampTowardZero(current_twist_.linear.z, linear_step);

    twist.angular.x = rampTowardZero(current_twist_.angular.x, angular_step);
    twist.angular.y = rampTowardZero(current_twist_.angular.y, angular_step);
    twist.angular.z = rampTowardZero(current_twist_.angular.z, angular_step);
}

void MechelangeloBehaviour::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    const sensor_msgs::msg::LaserScan filtered_scan = filterLaserScan(*msg);

    latest_scan_ = filtered_scan;
    latest_segments_ = buildLaserSegments(filtered_scan);

    filtered_scan_publisher_->publish(filtered_scan);
}

void MechelangeloBehaviour::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    latest_imu_ = *msg;
    imu_available_ = true;
}

void MechelangeloBehaviour::humanDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
    {
        return;
    }

    if (humanDetectionCooldownActive(this->now()))
    {
        g_human_detection_ignored_cooldown_count++;
        writeHumanInteractionStatsLog("manual_human_detection_ignored_during_cooldown", this->now());

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Human detection ignored during cooldown. Remaining %.1f s.",
            humanDetectionCooldownRemaining(this->now()));
        return;
    }

    if (g_human_detection_cooldown_active)
    {
        g_human_detection_cooldown_active = false;
    }

    RCLCPP_WARN(
        this->get_logger(),
        "Manual human detection trigger received. Interrupting autonomous behaviour.");

    current_state_ = NavigationState::HUMAN_DETECTED;
}

void MechelangeloBehaviour::humanTrackingCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 3)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Received invalid /human_tracking message. Expected [detected, centre_offset, distance_m].");
        return;
    }

    const bool detected = msg->data[0] > 0.5F;

    if (detected && humanDetectionCooldownActive(this->now()))
    {
        g_human_detection_ignored_cooldown_count++;
        human_locked_ = false;
        writeHumanInteractionStatsLog("camera_human_detection_ignored_during_cooldown", this->now());

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Camera human detection ignored during cooldown. Remaining %.1f s.",
            humanDetectionCooldownRemaining(this->now()));
        return;
    }

    if (g_human_detection_cooldown_active && !humanDetectionCooldownActive(this->now()))
    {
        g_human_detection_cooldown_active = false;
        writeHumanInteractionStatsLog("human_detection_cooldown_finished", this->now());
    }

    human_locked_ = detected;
    human_centre_offset_ = static_cast<double>(msg->data[1]);
    human_distance_m_ = static_cast<double>(msg->data[2]);
    last_human_tracking_time_ = this->now();

    if (human_locked_)
    {
        g_human_detection_accepted_count++;
        g_last_visible_human_valid = true;
        g_last_visible_human_centre_offset = human_centre_offset_;
        g_last_visible_human_distance_m = human_distance_m_;
        g_last_visible_human_time = last_human_tracking_time_;
        current_state_ = NavigationState::HUMAN_DETECTED;
    }
}

sensor_msgs::msg::LaserScan MechelangeloBehaviour::filterLaserScan(
    const sensor_msgs::msg::LaserScan &raw_scan)
{
    sensor_msgs::msg::LaserScan neighbour_filtered = raw_scan;
    sensor_msgs::msg::LaserScan final_filtered = raw_scan;

    if (raw_scan.ranges.empty() || raw_scan.angle_increment == 0.0)
    {
        return final_filtered;
    }

    const int scan_count = static_cast<int>(raw_scan.ranges.size());
    std::vector<double> x_points(scan_count, std::numeric_limits<double>::quiet_NaN());
    std::vector<double> y_points(scan_count, std::numeric_limits<double>::quiet_NaN());
    std::vector<bool> usable(scan_count, false);

    // Convert valid polar points to local Cartesian points.
    for (int i = 0; i < scan_count; ++i)
    {
        const double range = raw_scan.ranges[i];

        if (!isRangeUsableForFiltering(raw_scan, range))
        {
            neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
            final_filtered.ranges[i] = std::numeric_limits<float>::infinity();
            continue;
        }

        const double angle = raw_scan.angle_min + static_cast<double>(i) * raw_scan.angle_increment;
        x_points[i] = range * std::cos(angle);
        y_points[i] = range * std::sin(angle);
        usable[i] = true;
    }

    // Stage 1: suppress isolated points that do not have nearby neighbours.
    for (int i = 0; i < scan_count; ++i)
    {
        if (!usable[i])
        {
            neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
            continue;
        }

        int close_neighbour_count = 0;
        const int start_index = std::max(0, i - kNoiseNeighbourWindow);
        const int end_index = std::min(scan_count - 1, i + kNoiseNeighbourWindow);

        for (int j = start_index; j <= end_index; ++j)
        {
            if (j == i || !usable[j])
            {
                continue;
            }

            const double distance = std::hypot(x_points[i] - x_points[j], y_points[i] - y_points[j]);

            if (distance <= kNoiseNeighbourDistance)
            {
                close_neighbour_count++;
            }
        }

        if (close_neighbour_count < kNoiseMinNeighbourCount)
        {
            neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
        }
    }

    // Stage 2: build LaserProcessing-style segments and only keep points
    // that belong to a real segment.
    const std::vector<LaserSegment> accepted_segments = buildLaserSegments(neighbour_filtered);

    std::fill(final_filtered.ranges.begin(), final_filtered.ranges.end(), std::numeric_limits<float>::infinity());

    for (const LaserSegment &segment : accepted_segments)
    {
        for (int i = segment.start_index; i <= segment.end_index; ++i)
        {
            if (i >= 0 && i < scan_count && std::isfinite(neighbour_filtered.ranges[i]))
            {
                final_filtered.ranges[i] = neighbour_filtered.ranges[i];
            }
        }
    }

    return final_filtered;
}

std::vector<LaserSegment> MechelangeloBehaviour::buildLaserSegments(
    const sensor_msgs::msg::LaserScan &scan) const
{
    std::vector<LaserSegment> segments;

    if (scan.ranges.empty() || scan.angle_increment == 0.0)
    {
        return segments;
    }

    const int scan_count = static_cast<int>(scan.ranges.size());

    bool segment_active = false;
    LaserSegment current_segment;
    geometry_msgs::msg::Point previous_point;

    auto finish_segment = [&]()
    {
        if (!segment_active)
        {
            return;
        }

        current_segment.midpoint.x = 0.5 * (current_segment.start_point.x + current_segment.end_point.x);
        current_segment.midpoint.y = 0.5 * (current_segment.start_point.y + current_segment.end_point.y);
        current_segment.midpoint.z = 0.0;
        current_segment.length = std::hypot(
            current_segment.end_point.x - current_segment.start_point.x,
            current_segment.end_point.y - current_segment.start_point.y);
        current_segment.midpoint_angle = std::atan2(
            current_segment.midpoint.y,
            current_segment.midpoint.x);

        if (current_segment.point_count >= kSegmentMinPoints &&
            current_segment.length >= kSegmentMinLength)
        {
            segments.push_back(current_segment);
        }

        segment_active = false;
        current_segment = LaserSegment();
    };

    for (int i = 0; i < scan_count; ++i)
    {
        const double range = scan.ranges[i];

        if (!isRangeUsableForFiltering(scan, range))
        {
            finish_segment();
            continue;
        }

        const geometry_msgs::msg::Point point = polarToPoint(scan, i);

        if (!segment_active)
        {
            segment_active = true;
            current_segment = LaserSegment();
            current_segment.start_index = i;
            current_segment.end_index = i;
            current_segment.point_count = 1;
            current_segment.start_point = point;
            current_segment.end_point = point;
            current_segment.min_range = range;
            previous_point = point;
            continue;
        }

        const double gap = std::hypot(point.x - previous_point.x, point.y - previous_point.y);

        if (gap <= kSegmentJoinDistance)
        {
            current_segment.end_index = i;
            current_segment.end_point = point;
            current_segment.point_count++;
            current_segment.min_range = std::min(current_segment.min_range, range);
            previous_point = point;
        }
        else
        {
            finish_segment();

            segment_active = true;
            current_segment = LaserSegment();
            current_segment.start_index = i;
            current_segment.end_index = i;
            current_segment.point_count = 1;
            current_segment.start_point = point;
            current_segment.end_point = point;
            current_segment.min_range = range;
            previous_point = point;
        }
    }

    finish_segment();
    return segments;
}

geometry_msgs::msg::Point MechelangeloBehaviour::polarToPoint(
    const sensor_msgs::msg::LaserScan &scan,
    int index) const
{
    geometry_msgs::msg::Point point;

    if (index < 0 || index >= static_cast<int>(scan.ranges.size()))
    {
        return point;
    }

    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const double range = scan.ranges[index];

    point.x = range * std::cos(angle);
    point.y = range * std::sin(angle);
    point.z = 0.0;

    return point;
}

bool MechelangeloBehaviour::isRangeUsableForFiltering(
    const sensor_msgs::msg::LaserScan &scan,
    double range) const
{
    if (!std::isfinite(range))
    {
        return false;
    }

    if (range <= kMinValidRange)
    {
        return false;
    }

    if (scan.range_min > 0.0 && range < scan.range_min)
    {
        return false;
    }

    if (scan.range_max > 0.0 && range > scan.range_max)
    {
        return false;
    }

    return true;
}

bool MechelangeloBehaviour::isRangeValid(double range) const
{
    return std::isfinite(range) && range > kMinValidRange;
}

double MechelangeloBehaviour::getMinimumRange(double start_angle, double end_angle) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return std::numeric_limits<double>::infinity();
    }

    double min_range = std::numeric_limits<double>::infinity();

    for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
    {
        const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

        if (!angleInsideWindow(angle, start_angle, end_angle))
        {
            continue;
        }

        const double range = latest_scan_.ranges[i];

        if (isRangeValid(range) && range < min_range)
        {
            min_range = range;
        }
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
    {
        return false;
    }

    const int scan_count = static_cast<int>(latest_scan_.ranges.size());
    const double angle_step = std::fabs(latest_scan_.angle_increment);

    if (scan_count <= 0 || angle_step <= 0.0)
    {
        return false;
    }

    // ------------------------------------------------------
    // Step 1: classify beams as open or blocked.
    //
    // Open means either:
    //   - infinity: the LiDAR did not hit anything within range, or
    //   - a finite return farther than the clearance distance.
    // ------------------------------------------------------
    std::vector<bool> open_beam(scan_count, false);

    for (int i = 0; i < scan_count; ++i)
    {
        const double range = latest_scan_.ranges[i];

        if (std::isinf(range))
        {
            open_beam[i] = true;
        }
        else if (std::isfinite(range) && range >= kDvdOpenClearanceDistance)
        {
            open_beam[i] = true;
        }
    }

    auto angleForIndex = [&](int index)
    {
        return normaliseAngle(
            latest_scan_.angle_min + static_cast<double>(index) * latest_scan_.angle_increment);
    };

    auto absoluteAngleForIndex = [&](int index)
    {
        return std::fabs(angleForIndex(index));
    };

    auto rangeForLog = [&](int index)
    {
        const double range = latest_scan_.ranges[index];

        if (std::isinf(range))
        {
            return std::numeric_limits<double>::infinity();
        }

        if (std::isfinite(range))
        {
            return range;
        }

        return 0.0;
    };

    auto isPreferredBounceAngle = [&](int index)
    {
        const double abs_angle = absoluteAngleForIndex(index);
        return abs_angle >= kDvdPreferredMinTurnAngle &&
               abs_angle <= kDvdPreferredMaxTurnAngle;
    };

    auto isFallbackBounceAngle = [&](int index)
    {
        const double abs_angle = absoluteAngleForIndex(index);
        return abs_angle >= kDvdAvoidFrontAngle &&
               abs_angle <= kDvdAvoidReverseAngle;
    };

    // ------------------------------------------------------
    // Step 2: extract open sectors and collect safe candidate beams.
    //
    // Preferred candidates are side-bounce angles, roughly +/-55 to +/-150 deg.
    // General fallback candidates simply avoid the front wall and exact reverse.
    // Left and right candidates are stored separately so one long side of the
    // room does not dominate every decision.
    // ------------------------------------------------------
    std::vector<int> preferred_left_candidates;
    std::vector<int> preferred_right_candidates;
    std::vector<int> fallback_left_candidates;
    std::vector<int> fallback_right_candidates;
    std::vector<int> fallback_all_candidates;

    int accepted_sector_count = 0;
    int widest_sector_count = 0;

    auto collectCandidatesFromSector = [&](int start_index, int count)
    {
        if (count <= 0)
        {
            return;
        }

        const double sector_width = static_cast<double>(count) * angle_step;
        widest_sector_count = std::max(widest_sector_count, count);

        if (sector_width < kDvdMinSectorWidth)
        {
            return;
        }

        accepted_sector_count++;

        int edge_margin_count = static_cast<int>(std::ceil(kDvdSectorEdgeMargin / angle_step));

        // Never let the edge margin remove the entire sector.
        edge_margin_count = std::min(edge_margin_count, std::max(0, (count - 1) / 2));

        for (int offset = edge_margin_count; offset < count - edge_margin_count; ++offset)
        {
            const int index = (start_index + offset) % scan_count;
            const double angle = angleForIndex(index);

            if (isPreferredBounceAngle(index))
            {
                if (angle >= 0.0)
                {
                    preferred_left_candidates.push_back(index);
                }
                else
                {
                    preferred_right_candidates.push_back(index);
                }
            }

            if (isFallbackBounceAngle(index))
            {
                fallback_all_candidates.push_back(index);

                if (angle >= 0.0)
                {
                    fallback_left_candidates.push_back(index);
                }
                else
                {
                    fallback_right_candidates.push_back(index);
                }
            }
        }
    };

    const bool all_open = std::all_of(open_beam.begin(), open_beam.end(),
        [](bool value) { return value; });

    if (all_open)
    {
        collectCandidatesFromSector(0, scan_count);
    }
    else
    {
        for (int start_index = 0; start_index < scan_count; ++start_index)
        {
            if (!open_beam[start_index])
            {
                continue;
            }

            const int previous_index = (start_index - 1 + scan_count) % scan_count;
            if (open_beam[previous_index])
            {
                continue;
            }

            int count = 0;
            while (count < scan_count && open_beam[(start_index + count) % scan_count])
            {
                count++;
            }

            collectCandidatesFromSector(start_index, count);
        }
    }

    // ------------------------------------------------------
    // Step 3: choose a random safe bounce angle.
    //
    // Preferred behaviour:
    //   - use the side-bounce band if possible;
    //   - choose left/right with a 50/50 coin flip when both are available;
    //   - choose a random beam inside that side's safe candidate list.
    // This produces a DVD-screensaver style movement instead of always going
    // down the longest wall/side of the room.
    // ------------------------------------------------------
    static thread_local std::mt19937 rng(std::random_device{}());

    auto chooseFromCandidates = [&](const std::vector<int> &candidates)
    {
        std::uniform_int_distribution<int> index_dist(
            0, static_cast<int>(candidates.size()) - 1);
        return candidates[index_dist(rng)];
    };

    auto chooseBalancedSide = [&](const std::vector<int> &left_candidates,
                                  const std::vector<int> &right_candidates,
                                  bool &used_left_side,
                                  bool &used_right_side)
    {
        used_left_side = false;
        used_right_side = false;

        if (!left_candidates.empty() && !right_candidates.empty())
        {
            std::uniform_int_distribution<int> side_dist(0, 1);

            if (side_dist(rng) == 0)
            {
                used_left_side = true;
                return chooseFromCandidates(left_candidates);
            }

            used_right_side = true;
            return chooseFromCandidates(right_candidates);
        }

        if (!left_candidates.empty())
        {
            used_left_side = true;
            return chooseFromCandidates(left_candidates);
        }

        used_right_side = true;
        return chooseFromCandidates(right_candidates);
    };

    int selected_index = -1;
    bool used_preferred_bounce_band = false;
    bool used_left_side = false;
    bool used_right_side = false;

    if (!preferred_left_candidates.empty() || !preferred_right_candidates.empty())
    {
        selected_index = chooseBalancedSide(
            preferred_left_candidates,
            preferred_right_candidates,
            used_left_side,
            used_right_side);
        used_preferred_bounce_band = true;
    }
    else if (!fallback_left_candidates.empty() || !fallback_right_candidates.empty())
    {
        selected_index = chooseBalancedSide(
            fallback_left_candidates,
            fallback_right_candidates,
            used_left_side,
            used_right_side);
        used_preferred_bounce_band = false;
    }
    else if (!fallback_all_candidates.empty())
    {
        selected_index = chooseFromCandidates(fallback_all_candidates);
        used_left_side = angleForIndex(selected_index) >= 0.0;
        used_right_side = !used_left_side;
        used_preferred_bounce_band = false;
    }

    if (selected_index >= 0)
    {
        g_dvd_heading_selection_count++;
        out_angle = angleForIndex(selected_index);
        out_range = rangeForLog(selected_index);

        {
            std::ofstream log(debugLogPath(kDvdDebugLogFilename));
            log << std::fixed << std::setprecision(4);
            log << "=== DVD Bounce Heading Selection ===\n";
            log << "DVD heading selections so far: " << g_dvd_heading_selection_count << "\n";
            log << "Successful interaction entries: " << g_human_interaction_success_count << "\n";
            log << "Completed 30 s interactions: " << g_human_interaction_completed_count << "\n";
            log << "Abandoned before interaction: " << g_human_abandoned_before_interaction_count << "\n";
            log << "Abandoned after interaction: " << g_human_abandoned_after_interaction_count << "\n";
            log << "Cooldown ignored detections: " << g_human_detection_ignored_cooldown_count << "\n\n";
            log << "Open beam rule: inf OR range >= " << kDvdOpenClearanceDistance << " m\n";
            log << "Minimum accepted sector width: " << kDvdMinSectorWidth * 180.0 / M_PI << " deg\n";
            log << "Sector edge margin: " << kDvdSectorEdgeMargin * 180.0 / M_PI << " deg\n";
            log << "Preferred bounce band: +/-"
                << kDvdPreferredMinTurnAngle * 180.0 / M_PI << " to +/-"
                << kDvdPreferredMaxTurnAngle * 180.0 / M_PI << " deg\n";
            log << "Fallback bounce band: +/-"
                << kDvdAvoidFrontAngle * 180.0 / M_PI << " to +/-"
                << kDvdAvoidReverseAngle * 180.0 / M_PI << " deg\n\n";

            log << "Accepted open sectors: " << accepted_sector_count << "\n";
            log << "Widest open sector beams: " << widest_sector_count << "\n";
            log << "Preferred left candidates: " << preferred_left_candidates.size() << "\n";
            log << "Preferred right candidates: " << preferred_right_candidates.size() << "\n";
            log << "Fallback candidates: " << fallback_all_candidates.size() << "\n\n";

            log << "=== Result ===\n";
            log << "Selected behaviour: "
                << (used_preferred_bounce_band ? "preferred random side-bounce" : "fallback random safe bounce")
                << "\n";
            log << "Selected side: "
                << (used_left_side ? "left/positive" : (used_right_side ? "right/negative" : "unknown"))
                << "\n";
            log << "Selected index: " << selected_index << "\n";
            log << "Selected angle: " << out_angle * 180.0 / M_PI << " deg\n";
            log << "Representative range: " << out_range << " m\n\n";

            log << "=== Laser Scan Values ===\n";
            for (int i = 0; i < scan_count; ++i)
            {
                const double angle_deg = angleForIndex(i) * 180.0 / M_PI;
                log << "  [" << std::setw(4) << i << "]  angle="
                    << std::setw(9) << angle_deg << " deg  range="
                    << latest_scan_.ranges[i] << " m  open="
                    << (open_beam[i] ? "yes" : "no") << "  preferred="
                    << (isPreferredBounceAngle(i) ? "yes" : "no") << "\n";
            }
        }

        return true;
    }

    // ------------------------------------------------------
    // Step 4: fallback to the old longest finite scan if the room is too
    // cluttered for a safe random bounce heading.
    // ------------------------------------------------------
    double max_finite_range = 0.0;
    int max_finite_index = -1;

    for (int i = 0; i < scan_count; ++i)
    {
        const double range = latest_scan_.ranges[i];

        if (isRangeValid(range) && range > max_finite_range)
        {
            max_finite_range = range;
            max_finite_index = i;
        }
    }

    if (max_finite_index < 0)
    {
        return false;
    }

    const double tied_range_threshold =
        std::max(kMinValidRange, max_finite_range - kLongestRangeTieTolerance);

    std::vector<bool> near_longest(scan_count, false);

    for (int i = 0; i < scan_count; ++i)
    {
        const double range = latest_scan_.ranges[i];
        near_longest[i] = isRangeValid(range) && range >= tied_range_threshold;
    }

    int best_start_index = max_finite_index;
    int best_count = 0;

    for (int start_index = 0; start_index < scan_count; ++start_index)
    {
        if (!near_longest[start_index])
        {
            continue;
        }

        const int previous_index = (start_index - 1 + scan_count) % scan_count;
        if (near_longest[previous_index])
        {
            continue;
        }

        int count = 0;
        while (count < scan_count && near_longest[(start_index + count) % scan_count])
        {
            count++;
        }

        if (count > best_count)
        {
            best_count = count;
            best_start_index = start_index;
        }
    }

    if (best_count == 0)
    {
        best_count = 1;
        best_start_index = max_finite_index;
    }

    const double best_mid_index = std::fmod(
        static_cast<double>(best_start_index) +
            0.5 * static_cast<double>(std::max(0, best_count - 1)),
        static_cast<double>(scan_count));

    g_dvd_heading_selection_count++;
    out_angle = normaliseAngle(latest_scan_.angle_min + best_mid_index * latest_scan_.angle_increment);
    out_range = max_finite_range;

    {
        std::ofstream log(debugLogPath(kDvdDebugLogFilename));
        log << std::fixed << std::setprecision(4);
        log << "=== DVD Bounce Heading Selection ===\n";
        log << "DVD heading selections so far: " << g_dvd_heading_selection_count << "\n";
        log << "Successful interaction entries: " << g_human_interaction_success_count << "\n";
        log << "Completed 30 s interactions: " << g_human_interaction_completed_count << "\n";
        log << "Abandoned before interaction: " << g_human_abandoned_before_interaction_count << "\n";
        log << "Abandoned after interaction: " << g_human_abandoned_after_interaction_count << "\n";
        log << "Cooldown ignored detections: " << g_human_detection_ignored_cooldown_count << "\n\n";
        log << "Selected behaviour: fallback longest finite scan\n";
        log << "Longest finite range: " << out_range << " m\n";
        log << "Selected angle: " << out_angle * 180.0 / M_PI << " deg\n";
    }

    return true;
}

int MechelangeloBehaviour::angleToIndex(double angle_rad) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return -1;
    }

    double capped_angle = angle_rad;

    if (capped_angle < latest_scan_.angle_min)
    {
        capped_angle = latest_scan_.angle_min;
    }

    if (capped_angle > latest_scan_.angle_max)
    {
        capped_angle = latest_scan_.angle_max;
    }

    int index = static_cast<int>(
        std::round((capped_angle - latest_scan_.angle_min) / latest_scan_.angle_increment));

    index = std::clamp(index, 0, static_cast<int>(latest_scan_.ranges.size()) - 1);
    return index;
}

double MechelangeloBehaviour::normaliseAngle(double angle_rad) const
{
    while (angle_rad > M_PI)
    {
        angle_rad -= 2.0 * M_PI;
    }

    while (angle_rad < -M_PI)
    {
        angle_rad += 2.0 * M_PI;
    }

    return angle_rad;
}

bool MechelangeloBehaviour::angleInsideWindow(
    double angle_rad,
    double start_angle,
    double end_angle) const
{
    const double angle = normaliseAngle(angle_rad);
    const double start = normaliseAngle(start_angle);
    const double end = normaliseAngle(end_angle);

    if (start <= end)
    {
        return angle >= start && angle <= end;
    }

    return angle >= start || angle <= end;
}

bool MechelangeloBehaviour::segmentOverlapsAngleWindow(
    const LaserSegment &segment,
    double start_angle,
    double end_angle) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return false;
    }

    for (int i = segment.start_index; i <= segment.end_index; ++i)
    {
        if (i < 0 || i >= static_cast<int>(latest_scan_.ranges.size()))
        {
            continue;
        }

        const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

        if (angleInsideWindow(angle, start_angle, end_angle))
        {
            return true;
        }
    }

    return false;
}

bool MechelangeloBehaviour::findBlockingObstaclesInFront(
    std::vector<LaserSegment> &blocking_segments) const
{
    blocking_segments.clear();

    for (const LaserSegment &segment : latest_segments_)
    {
        if (segment.min_range <= stop_distance_m_ &&
            segmentOverlapsAngleWindow(segment, -kFrontCheckAngle, kFrontCheckAngle))
        {
            blocking_segments.push_back(segment);
        }
    }

    return !blocking_segments.empty();
}

void MechelangeloBehaviour::publishObstacleMarkers(
    const std::vector<LaserSegment> &blocking_segments)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
    clear_marker.header.stamp = this->get_clock()->now();
    clear_marker.ns = "behaviour_blocking_obstacles";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int marker_id = 1;

    for (const LaserSegment &segment : blocking_segments)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "behaviour_blocking_obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = segment.midpoint.x;
        marker.pose.position.y = segment.midpoint.y;
        marker.pose.position.z = 0.15;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Make marker size scale slightly with the observed segment length.
        const double marker_width = std::clamp(segment.length + 0.15, 0.20, 0.80);
        marker.scale.x = marker_width;
        marker.scale.y = marker_width;
        marker.scale.z = 0.30;

        // Red/orange transparent marker for blocking obstacle.
        marker.color.a = 0.75F;
        marker.color.r = 1.0F;
        marker.color.g = 0.15F;
        marker.color.b = 0.0F;

        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 400000000; // 0.4 s

        marker_array.markers.push_back(marker);
    }

    obstacle_marker_publisher_->publish(marker_array);
}

void MechelangeloBehaviour::clearObstacleMarkers()
{
    if (!obstacle_marker_publisher_)
    {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
    clear_marker.header.stamp = this->get_clock()->now();
    clear_marker.ns = "behaviour_blocking_obstacles";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    obstacle_marker_publisher_->publish(marker_array);
}

void MechelangeloBehaviour::longestLaserScan()
{
    double longest_angle = 0.0;
    double longest_range = 0.0;

    if (!getLongestRange(longest_angle, longest_range))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "No valid filtered laser scan data available for longest scan calculation.");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Selected exploration heading: Representative range = %.2f m at Angle = %.2f degrees",
        longest_range,
        longest_angle * 180.0 / M_PI);
}

double MechelangeloBehaviour::getHumanLidarRange(double centre_offset) const
{
    const double estimated_human_angle = -centre_offset * kCameraHorizontalFov;
    const double start_angle = estimated_human_angle - kHumanLidarWindow;
    const double end_angle = estimated_human_angle + kHumanLidarWindow;

    return getMinimumRange(start_angle, end_angle);
}

void MechelangeloBehaviour::captureSafetyZoneBaseline()
{
    safety_zone_baseline_scan_ = latest_scan_;
    safety_zone_baseline_captured_ = true;
    RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Filtered background baseline captured.");
}

bool MechelangeloBehaviour::isSafetyZoneViolated(double human_bearing_rad) const
{
    if (!safety_zone_baseline_captured_)
    {
        return false;
    }

    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return false;
    }

    if (safety_zone_baseline_scan_.ranges.size() != latest_scan_.ranges.size())
    {
        return false;
    }

    for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
    {
        const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
        const double angle_diff = normaliseAngle(angle - human_bearing_rad);

        // Exclude the window around the tracked human so they do not self-trigger.
        if (std::fabs(angle_diff) <= kSafetyZoneHumanExclusionAngle)
        {
            continue;
        }

        const double current_range = latest_scan_.ranges[i];

        if (!std::isfinite(current_range) || current_range <= kMinValidRange)
        {
            continue;
        }

        const double required_safety_radius = humanModeSafetyRadiusForAngle(angle);

        if (current_range >= required_safety_radius)
        {
            continue;
        }

        const double baseline_range = std::isfinite(safety_zone_baseline_scan_.ranges[i])
            ? safety_zone_baseline_scan_.ranges[i]
            : required_safety_radius;

        if (current_range < (baseline_range - kSafetyZoneIntruderThreshold))
        {
            return true;
        }
    }

    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MechelangeloBehaviour>();

    // true = simulation mode
    // false = real robot mode
    node->run(true);

    rclcpp::shutdown();
    return 0;
}
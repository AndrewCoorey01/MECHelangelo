// #include "behaviour.hpp"

// #include <iostream>
// #include <functional>
// #include <chrono>
// #include <cmath>
// #include <algorithm>
// #include <limits>
// #include <memory>
// #include <vector>

// #include "std_msgs/msg/float32_multi_array.hpp"

// using std::cout;
// using std::endl;
// using namespace std::chrono_literals;

// // -------------------------------
// // Behaviour constants
// // -------------------------------

// // Control loop runs every 100 ms.
// static constexpr double kControlPeriodSeconds = 0.1;

// // Movement tuning
// static constexpr double kForwardSpeed = 0.26;       // m/s
// static constexpr double kTurnSpeed = 0.6;           // rad/s
// static constexpr double kAngleGain = 0.8;           // proportional turning gain
// static constexpr double kAlignmentTolerance = 0.10; // radians, about 5.7 degrees

// // Stop 1.5 m away from the object/wall in front
// static constexpr double stop_distance_m_ = 1.5; // m for simulation
// // static constexpr double stop_distance_m_ = 0.75; // m for physical

// // 30 loops x 0.1 s = 3 seconds
// static constexpr int kStopDurationLoops = 30;

// // Ignore laser points closer than 0.5 m, this eliminates the blind spots of the robot interfering with navigation.
// static constexpr double kMinValidRange = 0.5; // m

// // Front scan window used while moving forward
// static constexpr double kFrontCheckAngle = 15.0 * M_PI / 180.0; // +/- 15 degrees

// // LaserScan noise suppression.
// // Random speckle dots are usually isolated single returns with no nearby neighbouring beams.
// // This filter keeps points that belong to a small local cluster and suppresses isolated returns.
// static constexpr int kNoiseNeighbourWindow = 3;             // check +/- 3 laser beams around each point
// static constexpr int kNoiseMinNeighbourCount = 1;          // require at least 1 nearby neighbour to keep a point
// static constexpr double kNoiseNeighbourDistance = 0.25;    // metres, max XY distance to count as a nearby neighbour

// // Human tracking tuning
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

// // LiDAR validation for human distance
// static constexpr double kCameraHorizontalFov = 60.0 * M_PI / 180.0; // approximate camera FOV
// static constexpr double kHumanLidarWindow = 10.0 * M_PI / 180.0;    // +/- 10 deg around estimated human direction
// static constexpr double kLidarCameraMaxDisagreement = 0.4;          // m
// static constexpr double kHumanLidarStopDistance = 1.5;              // m
// static constexpr double kHumanLidarStopTolerance = 0.15;            // m

// // Safety zone: full 360-degree perimeter check during interaction.
// // A background scan is taken when HUMAN_DETECTED is first entered.
// // An intruder is flagged only when a live reading is kSafetyZoneIntruderThreshold
// // closer than the baseline at the same angle AND within kSafetyZoneRadius.
// // This means walls and robot frame supports (which are in the baseline) are
// // never flagged; only new objects that move into the space trigger the freeze.
// static constexpr double kSafetyZoneRadius = 1.5;                               // m  — max range of interest
// static constexpr double kSafetyZoneIntruderThreshold = 0.3;                    // m  — must be this much closer than baseline to count
// static constexpr double kSafetyZoneHumanExclusionAngle = 25.0 * M_PI / 180.0; // +/- 25 deg around tracked human bearing

// // Kept as file-scope variables so behaviour.hpp does not need to change for this test.
// static rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr g_human_tracking_subscriber;
// static bool g_human_locked = false;
// static double g_human_centre_offset = 0.0;
// static double g_human_distance_m = -1.0;
// static rclcpp::Time g_last_human_tracking_time;

// // Published for RViz/debugging. Add /scan_filtered as a LaserScan display in RViz to see
// // the suppressed scan. The behaviour logic also uses this filtered scan internally.
// static rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr g_filtered_scan_publisher;

// static bool isLaserRangeUsableForFiltering(
//     const sensor_msgs::msg::LaserScan &scan,
//     double range)
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

// static sensor_msgs::msg::LaserScan suppressIsolatedLaserNoise(
//     const sensor_msgs::msg::LaserScan &raw_scan)
// {
//     sensor_msgs::msg::LaserScan filtered_scan = raw_scan;

//     if (raw_scan.ranges.empty() || raw_scan.angle_increment == 0.0)
//     {
//         return filtered_scan;
//     }

//     const int scan_count = static_cast<int>(raw_scan.ranges.size());
//     std::vector<double> x_points(scan_count, std::numeric_limits<double>::quiet_NaN());
//     std::vector<double> y_points(scan_count, std::numeric_limits<double>::quiet_NaN());
//     std::vector<bool> usable(scan_count, false);

//     for (int i = 0; i < scan_count; ++i)
//     {
//         const double range = raw_scan.ranges[i];

//         if (!isLaserRangeUsableForFiltering(raw_scan, range))
//         {
//             continue;
//         }

//         const double angle = raw_scan.angle_min + i * raw_scan.angle_increment;
//         x_points[i] = range * std::cos(angle);
//         y_points[i] = range * std::sin(angle);
//         usable[i] = true;
//     }

//     for (int i = 0; i < scan_count; ++i)
//     {
//         if (!usable[i])
//         {
//             filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
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

//             const double dx = x_points[i] - x_points[j];
//             const double dy = y_points[i] - y_points[j];
//             const double distance = std::hypot(dx, dy);

//             if (distance <= kNoiseNeighbourDistance)
//             {
//                 close_neighbour_count++;
//             }
//         }

//         if (close_neighbour_count < kNoiseMinNeighbourCount)
//         {
//             filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
//         }
//     }

//     return filtered_scan;
// }

// MechelangeloBehaviour::MechelangeloBehaviour()
//     : Node("mechelangelo_behaviour"),
//       blind_autonomous_active_(false),
//       safety_zone_violated_(false),
//       safety_zone_baseline_captured_(false),
//       current_state_(NavigationState::SEARCHING),
//       target_angle_(0.0),
//       target_range_(0.0),
//       stop_counter_(0),
//       random_engine_(std::random_device{}()),
//       turn_dist_(-1.0, 1.0)
// {
//     laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         10,
//         std::bind(&MechelangeloBehaviour::laserScanCallback, this, std::placeholders::_1));

//     cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
//         "/cmd_vel",
//         10);

//     g_filtered_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>(
//         "/scan_filtered",
//         10);

//     human_detected_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
//         "/human_detected",
//         10,
//         std::bind(&MechelangeloBehaviour::humanDetectedCallback, this, std::placeholders::_1));

//     g_last_human_tracking_time = this->now();

//     g_human_tracking_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>(
//         "/human_tracking",
//         10,
//         [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
//         {
//             if (msg->data.size() < 3)
//             {
//                 RCLCPP_WARN_THROTTLE(
//                     this->get_logger(),
//                     *this->get_clock(),
//                     2000,
//                     "Received invalid /human_tracking message. Expected [detected, centre_offset, distance_m].");
//                 return;
//             }

//             g_human_locked = msg->data[0] > 0.5;
//             g_human_centre_offset = static_cast<double>(msg->data[1]);
//             g_human_distance_m = static_cast<double>(msg->data[2]);
//             g_last_human_tracking_time = this->now();

//             if (g_human_locked)
//             {
//                 current_state_ = NavigationState::HUMAN_DETECTED;
//             }
//         });

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
//     RCLCPP_INFO(this->get_logger(), "Executing blind autonomous behavior.");

//     blind_autonomous_active_ = true;
//     safety_zone_violated_ = false;
//     safety_zone_baseline_captured_ = false;
//     current_state_ = NavigationState::SEARCHING;
//     target_angle_ = 0.0;
//     target_range_ = 0.0;
//     stop_counter_ = 0;
// }

// void MechelangeloBehaviour::mappedAutonomous()
// {
//     RCLCPP_INFO(this->get_logger(), "Executing mapped autonomous behavior.");
// }

// void MechelangeloBehaviour::controlLoop()
// {
//     if (!blind_autonomous_active_)
//     {
//         return;
//     }

//     geometry_msgs::msg::Twist twist;

//     // Safety: wait until valid LaserScan data exists
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         RCLCPP_WARN_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             2000,
//             "Waiting for valid LaserScan data...");

//         stopRobot(twist);
//         current_twist_ = twist;
//         cmd_vel_publisher_->publish(twist);
//         return;
//     }

//     switch (current_state_)
//     {

//     // ------------------------------------------------------
//     // SEARCHING:
//     // Find the longest valid laser ray once.
//     // This gives the robot its chosen direction.
//     // ------------------------------------------------------
//     case NavigationState::SEARCHING:
//     {
//         stopRobot(twist);

//         double longest_angle = 0.0;
//         double longest_range = 0.0;

//         if (!getLongestRange(longest_angle, longest_range))
//         {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 2000,
//                 "SEARCHING: No valid LaserScan range found.");
//             break;
//         }

//         target_angle_ = longest_angle;
//         target_range_ = longest_range;

//         RCLCPP_INFO(
//             this->get_logger(),
//             "SEARCHING: Longest scan found at %.2f deg, range %.2f m",
//             target_angle_ * 180.0 / M_PI,
//             target_range_);

//         current_state_ = NavigationState::ALIGNING;
//         break;
//     }

//     // ------------------------------------------------------
//     // ALIGNING:
//     // Rotate toward the saved longest laser direction.
//     //
//     // Since this version does not use odometry, it estimates
//     // the remaining angle by subtracting the commanded angular
//     // movement every 100 ms.
//     // ------------------------------------------------------
//     case NavigationState::ALIGNING:
//     {
//         twist.linear.x = 0.0;

//         if (std::fabs(target_angle_) <= kAlignmentTolerance)
//         {
//             RCLCPP_INFO(
//                 this->get_logger(),
//                 "ALIGNING: Aligned with chosen laser direction. Starting forward movement.");

//             twist.angular.z = 0.0;
//             current_state_ = NavigationState::MOVING;
//             break;
//         }

//         double turn_cmd = std::clamp(
//             target_angle_ * kAngleGain,
//             -kTurnSpeed,
//             kTurnSpeed);

//         twist.angular.z = turn_cmd;

//         // Estimate how much the robot turned during this control step.
//         // Positive angular.z reduces a positive target angle.
//         target_angle_ -= turn_cmd * kControlPeriodSeconds;

//         RCLCPP_INFO_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             1000,
//             "ALIGNING: Remaining angle %.2f deg, angular command %.2f rad/s",
//             target_angle_ * 180.0 / M_PI,
//             twist.angular.z);

//         break;
//     }

//         // ------------------------------------------------------
//         // MOVING:
//         // Move forward only.
//         // Stop when the front LaserScan distance reaches 1.5 m.
//         // ------------------------------------------------------
//     case NavigationState::MOVING:
//     {
//         double front_range = getFrontRange();

//         // If front_range is infinity, that means there is no valid obstacle
//         // in the front check cone. Treat that as clear space.
//         if (std::isinf(front_range))
//         {
//             twist.linear.x = kForwardSpeed;
//             twist.angular.z = 0.0;

//             RCLCPP_INFO_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "MOVING: Front is clear. Driving forward.");

//             break;
//         }

//         // NaN is genuinely bad scan data.
//         if (std::isnan(front_range))
//         {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "MOVING: Front scan is NaN. Returning to SEARCHING for safety.");

//             stopRobot(twist);
//             current_state_ = NavigationState::SEARCHING;
//             break;
//         }

//         // Stop when a valid front obstacle/wall is within 1.5 m.
//         if (front_range <= stop_distance_m_)
//         {
//             RCLCPP_INFO(
//                 this->get_logger(),
//                 "MOVING: Reached safety stop distance. Front range = %.2f m",
//                 front_range);

//             stopRobot(twist);
//             stop_counter_ = 0;
//             current_state_ = NavigationState::STOPPED;
//             break;
//         }

//         // Otherwise keep moving forward.
//         twist.linear.x = kForwardSpeed;
//         twist.angular.z = 0.0;

//         RCLCPP_INFO_THROTTLE(
//             this->get_logger(),
//             *this->get_clock(),
//             1000,
//             "MOVING: Driving forward. Front range = %.2f m",
//             front_range);

//         break;
//     }

//     // ------------------------------------------------------
//     // STOPPED:
//     // Pause for 3 seconds, then look for the next longest scan.
//     // ------------------------------------------------------
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

//     // ------------------------------------------------------
//     // HUMAN_DETECTED:
//     // TARGET HUMAN, move towards target and stop 1.5m away from them.
//     //
//     // Safety zone interrupt: if any unexpected object enters the 1.5 m radius
//     // (anywhere except the direction of the tracked human) all motion stops
//     // until the zone is clear.  This prevents the robot from driving through
//     // bystanders or performing arm movements while someone is too close.
//     // ------------------------------------------------------
//     case NavigationState::HUMAN_DETECTED:
//     {
//         const double time_since_tracking =
//             (this->now() - g_last_human_tracking_time).seconds();

//         if (!g_human_locked || time_since_tracking > kHumanLostTimeout)
//         {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: Human lost. Returning to blind autonomous search.");

//             stopRobot(twist);
//             g_human_locked = false;
//             safety_zone_violated_ = false;
//             safety_zone_baseline_captured_ = false;
//             current_state_ = NavigationState::SEARCHING;
//             break;
//         }

//         // Capture the room background (walls, frame supports, everything static)
//         // on the very first control tick after entering HUMAN_DETECTED.
//         // All subsequent safety zone checks compare against this snapshot so
//         // only genuinely new objects trigger a freeze.
//         if (!safety_zone_baseline_captured_)
//         {
//             captureSafetyZoneBaseline();
//         }

//         // Estimate the tracked human's bearing from the camera offset.
//         // This bearing is excluded from the safety zone check so the tracked
//         // human does not false-trigger the interrupt.
//         const double human_bearing_rad = -g_human_centre_offset * kCameraHorizontalFov;

//         // Safety zone interrupt: freeze all interaction if an unexpected intruder
//         // appears significantly closer than the baseline at any angle.
//         // Resumes automatically once the intruder leaves.
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
//                 RCLCPP_INFO(
//                     this->get_logger(),
//                     "SAFETY ZONE: Clear. Resuming interaction.");
//             }
//         }

//         if (safety_zone_violated_)
//         {
//             stopRobot(twist);
//             break;
//         }

//         // Turn dynamically to keep the human close to the centre of the camera frame.
//         // If your physical robot turns away from the human, flip the sign on this command.
//         if (std::fabs(g_human_centre_offset) <= kHumanCentreDeadZone)
//         {
//             twist.angular.z = 0.0;
//         }
//         else
//         {
//             twist.angular.z = std::clamp(
//                 -kHumanTurnGain * g_human_centre_offset,
//                 -kHumanMaxTurnSpeed,
//                 kHumanMaxTurnSpeed);
//         }

//         // Move forward/backward until the estimated person distance is close to 1.5 m.
//         // Use LiDAR as the trusted distance source.
//         // Camera distance is only used as a fallback/debug comparison.
//         const double human_lidar_range = getHumanLidarRange(g_human_centre_offset);
//         const bool lidar_distance_valid = std::isfinite(human_lidar_range);

//         if (!lidar_distance_valid)
//         {
//             // If LiDAR cannot see anything in the human direction, do not drive forward.
//             // Still allow turning so the camera can re-centre the human.
//             twist.linear.x = 0.0;

//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 1000,
//                 "HUMAN_DETECTED: No valid LiDAR return near human bearing. Turning only.");
//         }
//         else
//         {
//             // Optional diagnostic: compare camera distance and LiDAR distance.
//             if (g_human_distance_m > 0.0 && std::isfinite(g_human_distance_m))
//             {
//                 const double disagreement = std::fabs(g_human_distance_m - human_lidar_range);

//                 if (disagreement > kLidarCameraMaxDisagreement)
//                 {
//                     RCLCPP_WARN_THROTTLE(
//                         this->get_logger(),
//                         *this->get_clock(),
//                         1000,
//                         "HUMAN_DETECTED: Camera/LiDAR distance disagreement. Camera=%.2f m, LiDAR=%.2f m",
//                         g_human_distance_m,
//                         human_lidar_range);
//                 }
//             }

//             // Control distance using LiDAR, not camera.
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

//             // Absolute safety limit:
//             // If anything is closer than the minimum allowed distance, never drive forward.
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
//             g_human_centre_offset,
//             g_human_distance_m,
//             twist.linear.x,
//             twist.angular.z);

//         break;
//     }

//     default:
//     {
//         RCLCPP_WARN(
//             this->get_logger(),
//             "Unknown navigation state. Returning to SEARCHING.");

//         stopRobot(twist);
//         current_state_ = NavigationState::SEARCHING;
//         break;
//     }
//     }

//     current_twist_ = twist;
//     cmd_vel_publisher_->publish(twist);
// }

// void MechelangeloBehaviour::stopRobot(geometry_msgs::msg::Twist &twist)
// {
//     twist.linear.x = 0.0;
//     twist.linear.y = 0.0;
//     twist.linear.z = 0.0;

//     twist.angular.x = 0.0;
//     twist.angular.y = 0.0;
//     twist.angular.z = 0.0;
// }

// double MechelangeloBehaviour::getMinimumRange(double start_angle, double end_angle) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return std::numeric_limits<double>::infinity();
//     }

//     int start_index = angleToIndex(start_angle);
//     int end_index = angleToIndex(end_angle);

//     if (start_index < 0 || end_index < 0)
//     {
//         return std::numeric_limits<double>::infinity();
//     }

//     if (start_index > end_index)
//     {
//         std::swap(start_index, end_index);
//     }

//     double min_range = std::numeric_limits<double>::infinity();

//     for (int i = start_index; i <= end_index; ++i)
//     {
//         if (i < 0 || i >= static_cast<int>(latest_scan_.ranges.size()))
//         {
//             continue;
//         }

//         double range = latest_scan_.ranges[i];

//         if (isRangeValid(range) && range < min_range)
//         {
//             min_range = range;
//         }
//     }

//     return min_range;
// }

// double MechelangeloBehaviour::getFrontRange() const
// {
//     // Use a small cone in front of the robot instead of exactly one ray.
//     // This is more stable because one LaserScan reading can be noisy.
//     return getMinimumRange(-kFrontCheckAngle, kFrontCheckAngle);
// }

// bool MechelangeloBehaviour::getLongestRange(double &out_angle, double &out_range) const
// {
//     if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
//     {
//         return false;
//     }

//     double max_range = 0.0;
//     int max_index = -1;

//     for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
//     {
//         double range = latest_scan_.ranges[i];

//         if (isRangeValid(range) && range > max_range)
//         {
//             max_range = range;
//             max_index = static_cast<int>(i);
//         }
//     }

//     if (max_index < 0)
//     {
//         return false;
//     }

//     out_angle = latest_scan_.angle_min + max_index * latest_scan_.angle_increment;
//     out_range = max_range;

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

//     if (index < 0)
//     {
//         index = 0;
//     }

//     if (index >= static_cast<int>(latest_scan_.ranges.size()))
//     {
//         index = static_cast<int>(latest_scan_.ranges.size()) - 1;
//     }

//     return index;
// }

// bool MechelangeloBehaviour::isRangeValid(double range) const
// {
//     return std::isfinite(range) && range > kMinValidRange;
// }

// void MechelangeloBehaviour::laserScanCallback(
//     const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//     sensor_msgs::msg::LaserScan filtered_scan = suppressIsolatedLaserNoise(*msg);

//     latest_scan_ = filtered_scan;

//     if (g_filtered_scan_publisher)
//     {
//         g_filtered_scan_publisher->publish(filtered_scan);
//     }
// }

// void MechelangeloBehaviour::humanDetectedCallback(
//     const std_msgs::msg::Bool::SharedPtr msg)
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

// void MechelangeloBehaviour::longestLaserScan()
// {
//     double longest_angle = 0.0;
//     double longest_range = 0.0;

//     if (!getLongestRange(longest_angle, longest_range))
//     {
//         RCLCPP_WARN(
//             this->get_logger(),
//             "No valid laser scan data available for longest scan calculation.");
//         return;
//     }

//     RCLCPP_INFO(
//         this->get_logger(),
//         "Longest valid laser scan: Range = %.2f m at Angle = %.2f degrees",
//         longest_range,
//         longest_angle * 180.0 / M_PI);
// }

// double MechelangeloBehaviour::getHumanLidarRange(double centre_offset) const
// {
//     // Convert camera image offset into approximate bearing angle.
//     // centre_offset: -0.5 left, 0 centre, +0.5 right.
//     //
//     // NOTE:
//     // Depending on your camera/LiDAR frame convention, you may need to flip this sign.
//     double estimated_human_angle = -centre_offset * kCameraHorizontalFov;

//     double start_angle = estimated_human_angle - kHumanLidarWindow;
//     double end_angle = estimated_human_angle + kHumanLidarWindow;

//     return getMinimumRange(start_angle, end_angle);
// }

// void MechelangeloBehaviour::captureSafetyZoneBaseline()
// {
//     safety_zone_baseline_scan_ = latest_scan_;
//     safety_zone_baseline_captured_ = true;
//     RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Background baseline captured.");
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
//         const double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;

//         // Exclude the window around the tracked human so they don't self-trigger.
//         double angle_diff = angle - human_bearing_rad;
//         while (angle_diff >  M_PI) angle_diff -= 2.0 * M_PI;
//         while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
//         if (std::fabs(angle_diff) <= kSafetyZoneHumanExclusionAngle)
//         {
//             continue;
//         }

//         const double current_range = latest_scan_.ranges[i];

//         // Skip robot blind spot (≤ 0.5 m) and invalid readings.
//         if (!std::isfinite(current_range) || current_range <= kMinValidRange)
//         {
//             continue;
//         }

//         // Only care about objects within the safety zone radius.
//         if (current_range >= kSafetyZoneRadius)
//         {
//             continue;
//         }

//         // Use the baseline reading at this angle as the expected background distance.
//         // If no baseline return existed here, treat the zone radius as the reference
//         // (i.e. anything appearing within kSafetyZoneRadius at that angle is new).
//         const double baseline_range = std::isfinite(safety_zone_baseline_scan_.ranges[i])
//             ? safety_zone_baseline_scan_.ranges[i]
//             : kSafetyZoneRadius;

//         // Flag as intruder only if significantly closer than the background.
//         // This threshold ignores LiDAR noise and the robot's own frame supports
//         // since those appear at the same distance in both the baseline and live scan.
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



#include "behaviour.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <functional>
#include <memory>
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
static constexpr double kHumanMaxForwardSpeed = 0.22;   // m/s
static constexpr double kHumanMaxReverseSpeed = 0.12;   // m/s
static constexpr double kHumanMaxTurnSpeed = 0.7;       // rad/s
static constexpr double kHumanTurnGain = 1.8;           // image offset to angular speed
static constexpr double kHumanForwardGain = 0.35;       // distance error to linear speed
static constexpr double kHumanCentreDeadZone = 0.06;    // normalised image width
static constexpr double kHumanLostTimeout = 1.0;        // seconds

// LiDAR validation for human distance.
static constexpr double kCameraHorizontalFov = 60.0 * M_PI / 180.0;
static constexpr double kHumanLidarWindow = 10.0 * M_PI / 180.0;
static constexpr double kLidarCameraMaxDisagreement = 0.4;
static constexpr double kHumanLidarStopDistance = 1.5;
static constexpr double kHumanLidarStopTolerance = 0.15;

// Safety zone.
static constexpr double kSafetyZoneRadius = 1.5;
static constexpr double kSafetyZoneIntruderThreshold = 0.3;
static constexpr double kSafetyZoneHumanExclusionAngle = 25.0 * M_PI / 180.0;

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
            "SEARCHING: Longest filtered scan found at %.2f deg, range %.2f m",
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

        if (!human_locked_ || time_since_tracking > kHumanLostTimeout)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "HUMAN_DETECTED: Human lost. Returning to blind autonomous search.");

            stopRobot(twist);
            human_locked_ = false;
            safety_zone_violated_ = false;
            safety_zone_baseline_captured_ = false;
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
                    "SAFETY ZONE: Object detected within %.1f m. Interaction paused.",
                    kSafetyZoneRadius);
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

        if (std::fabs(human_centre_offset_) <= kHumanCentreDeadZone)
        {
            twist.angular.z = 0.0;
        }
        else
        {
            twist.angular.z = std::clamp(
                -kHumanTurnGain * human_centre_offset_,
                -kHumanMaxTurnSpeed,
                kHumanMaxTurnSpeed);
        }

        const double human_lidar_range = getHumanLidarRange(human_centre_offset_);
        const bool lidar_distance_valid = std::isfinite(human_lidar_range);

        if (!lidar_distance_valid)
        {
            twist.linear.x = 0.0;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "HUMAN_DETECTED: No valid filtered LiDAR return near human bearing. Turning only.");
        }
        else
        {
            if (human_distance_m_ > 0.0 && std::isfinite(human_distance_m_))
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

            const double distance_error = human_lidar_range - kHumanLidarStopDistance;

            if (std::fabs(distance_error) <= kHumanLidarStopTolerance)
            {
                twist.linear.x = 0.0;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "HUMAN_DETECTED: LiDAR validated target distance %.2f m. Ready for arm mimicry.",
                    human_lidar_range);
            }
            else
            {
                twist.linear.x = std::clamp(
                    kHumanForwardGain * distance_error,
                    -kHumanMaxReverseSpeed,
                    kHumanMaxForwardSpeed);
            }

            if (human_lidar_range < kHumanLidarStopDistance && twist.linear.x > 0.0)
            {
                twist.linear.x = 0.0;
            }
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "HUMAN_DETECTED: offset=%.2f distance=%.2f cmd linear=%.2f angular=%.2f",
            human_centre_offset_,
            human_distance_m_,
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

    human_locked_ = msg->data[0] > 0.5F;
    human_centre_offset_ = static_cast<double>(msg->data[1]);
    human_distance_m_ = static_cast<double>(msg->data[2]);
    last_human_tracking_time_ = this->now();

    if (human_locked_)
    {
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

    double max_range = 0.0;
    int max_index = -1;

    for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
    {
        const double range = latest_scan_.ranges[i];

        if (isRangeValid(range) && range > max_range)
        {
            max_range = range;
            max_index = i;
        }
    }

    if (max_index < 0)
    {
        return false;
    }

    const double tied_range_threshold = std::max(kMinValidRange, max_range - kLongestRangeTieTolerance);
    const int scan_count = static_cast<int>(latest_scan_.ranges.size());
    std::vector<bool> near_longest(scan_count, false);

    for (int i = 0; i < scan_count; ++i)
    {
        const double range = latest_scan_.ranges[i];
        near_longest[i] = isRangeValid(range) && range >= tied_range_threshold;
    }

    int best_start_index = max_index;
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
        best_count = scan_count;
        best_start_index = 0;
    }

    const double best_mid_index = std::fmod(
        static_cast<double>(best_start_index) + 0.5 * static_cast<double>(std::max(0, best_count - 1)),
        static_cast<double>(scan_count));

    out_angle = normaliseAngle(latest_scan_.angle_min + best_mid_index * latest_scan_.angle_increment);
    out_range = max_range;

    {
        std::ofstream log("/home/andy/git/MECHelangelo/src/mechelangelo_behaviour/debug/longest_scan_debug.txt");
        log << std::fixed << std::setprecision(4);
        log << "=== Laser Scan Values ===\n";
        for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
        {
            const double angle_deg = (latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment) * 180.0 / M_PI;
            log << "  [" << std::setw(4) << i << "]  angle=" << std::setw(9) << angle_deg << " deg  range=" << latest_scan_.ranges[i] << " m\n";
        }
        log << "\n=== Result ===\n";
        log << "  Longest range: " << out_range << " m\n";
        log << "  At angle:      " << (out_angle * 180.0 / M_PI) << " deg\n";
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
        "Longest valid filtered laser scan: Range = %.2f m at Angle = %.2f degrees",
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

        if (current_range >= kSafetyZoneRadius)
        {
            continue;
        }

        const double baseline_range = std::isfinite(safety_zone_baseline_scan_.ranges[i])
            ? safety_zone_baseline_scan_.ranges[i]
            : kSafetyZoneRadius;

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

//behaviour algorithm using inf scans with longest scan to keep the robot turning into the most open space, imu check still in there

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
// static constexpr double kStopDistance = 1.5; // m for simulation
// // static constexpr double kStopDistance = 0.75; // m for physical

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
//         if (front_range <= kStopDistance)
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



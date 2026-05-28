// =============================================================================
// SCRATCH COPY — behaviour.hpp as of 2026-05-28
// Saved before safety-zone interrupt was added.
//
// FUNCTIONALITY OF THIS VERSION:
//   Navigation state machine with five states:
//     SEARCHING  — robot stops and finds the longest valid LiDAR ray to use as
//                  the next travel direction.
//     ALIGNING   — robot rotates in place toward the chosen direction using open-
//                  loop angle estimation (no odometry).
//     MOVING     — robot drives straight forward until the front LiDAR cone
//                  (±15°) returns a distance ≤ kStopDistance.
//     STOPPED    — robot pauses for ~3 s then transitions back to SEARCHING
//                  (sentry patrol loop).
//     HUMAN_DETECTED — triggered by /human_detected bool or /human_tracking data.
//                  Robot turns to centre the human in the camera frame and drives
//                  to ~1.5 m away using LiDAR-validated distance.  Returns to
//                  SEARCHING if the human is lost for >1 s.
//
//   There is NO safety zone interrupt in this version.  Any object entering
//   the 1.5 m radius during interaction is not detected and will not pause
//   the robot.
// =============================================================================

#ifndef BEHAVIOUR_HPP
#define BEHAVIOUR_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <random>


enum class NavigationState {
    SEARCHING,
    ALIGNING,
    MOVING,
    STOPPED,
    HUMAN_DETECTED
};


class MechelangeloBehaviour : public rclcpp::Node
{
public:
    MechelangeloBehaviour();
    ~MechelangeloBehaviour();

    void run(bool sim_mode);

private:
    // ------------------------------------------------------
    // Behaviour modes
    // ------------------------------------------------------
    void blindAutonomous();
    void mappedAutonomous();

    // ------------------------------------------------------
    // Main control loop
    // ------------------------------------------------------
    void controlLoop();

    // ------------------------------------------------------
    // ROS callbacks
    // ------------------------------------------------------
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // ------------------------------------------------------
    // Movement helpers
    // ------------------------------------------------------
    void stopRobot(geometry_msgs::msg::Twist &twist);

    // ------------------------------------------------------
    // LaserScan helper functions
    // ------------------------------------------------------
    bool getLongestRange(double &out_angle, double &out_range) const;

    double getMinimumRange(double start_angle, double end_angle) const;

    double getFrontRange() const;

    int angleToIndex(double angle_rad) const;

    bool isRangeValid(double range) const;

    void longestLaserScan();

    double getHumanLidarRange(double centre_offset) const;

    // ------------------------------------------------------
    // ROS publishers/subscribers/timers
    // ------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Manual human detection trigger
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_detected_subscriber_;

    void humanDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr control_timer_;

    // ------------------------------------------------------
    // Stored sensor and command data
    // ------------------------------------------------------
    sensor_msgs::msg::LaserScan latest_scan_;

    geometry_msgs::msg::Twist current_twist_;

    // ------------------------------------------------------
    // Behaviour state
    // ------------------------------------------------------
    bool blind_autonomous_active_;

    NavigationState current_state_;

    double target_angle_;

    double target_range_;

    int stop_counter_;

    // ------------------------------------------------------
    // Random tools
    // Kept from your original structure in case you use them
    // later for mapped/random autonomous behaviour.
    // ------------------------------------------------------
    std::default_random_engine random_engine_;

    std::uniform_real_distribution<double> turn_dist_;
};

#endif  // BEHAVIOUR_HPP

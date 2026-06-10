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

// Main navigation state machine.
enum class NavigationState
{
    SEARCHING,
    ALIGNING,
    MOVING,
    STOPPED,
    HUMAN_DETECTED
};

// A valid cluster/segment of neighbouring LaserScan points.
// This is based on the previous LaserProcessing countSegments() idea:
// neighbouring points are joined only when their Cartesian gap is small.
struct LaserSegment
{
    int start_index = 0;
    int end_index = 0;
    int point_count = 0;

    geometry_msgs::msg::Point start_point;
    geometry_msgs::msg::Point end_point;
    geometry_msgs::msg::Point midpoint;

    double min_range = 0.0;
    double midpoint_angle = 0.0;
    double length = 0.0;
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
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void humanDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void humanTrackingCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // ------------------------------------------------------
    // Movement helpers
    // ------------------------------------------------------
    void stopRobot(geometry_msgs::msg::Twist &twist);

    // ------------------------------------------------------
    // LaserScan filtering and helper functions
    // ------------------------------------------------------
    sensor_msgs::msg::LaserScan filterLaserScan(const sensor_msgs::msg::LaserScan &raw_scan);
    std::vector<LaserSegment> buildLaserSegments(const sensor_msgs::msg::LaserScan &scan) const;
    geometry_msgs::msg::Point polarToPoint(const sensor_msgs::msg::LaserScan &scan, int index) const;

    bool isRangeUsableForFiltering(const sensor_msgs::msg::LaserScan &scan, double range) const;
    bool isRangeValid(double range) const;
    bool getLongestRange(double &out_angle, double &out_range) const;
    double getMinimumRange(double start_angle, double end_angle) const;
    double getFrontRange() const;
    int angleToIndex(double angle_rad) const;
    double normaliseAngle(double angle_rad) const;
    bool angleInsideWindow(double angle_rad, double start_angle, double end_angle) const;

    bool findBlockingObstaclesInFront(std::vector<LaserSegment> &blocking_segments) const;
    bool segmentOverlapsAngleWindow(const LaserSegment &segment, double start_angle, double end_angle) const;

    void longestLaserScan();
    double getHumanLidarRange(double centre_offset) const;

    // ------------------------------------------------------
    // Safety-zone helpers
    // ------------------------------------------------------
    void captureSafetyZoneBaseline();
    bool isSafetyZoneViolated(double human_bearing_rad) const;

    // ------------------------------------------------------
    // RViz marker helpers
    // ------------------------------------------------------
    void publishObstacleMarkers(const std::vector<LaserSegment> &blocking_segments);
    void clearObstacleMarkers();

    // ------------------------------------------------------
    // ROS publishers/subscribers/timers
    // ------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Filtered scan debug output. View this in RViz instead of /scan.
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_publisher_;

    // Marker output for obstacles that stop the behaviour.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_publisher_;

    // Manual human detection trigger.
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_detected_subscriber_;

    // Optional camera/person tracking input.
    // Message format: [detected, centre_offset, distance_m]
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr human_tracking_subscriber_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    // ------------------------------------------------------
    // Stored sensor and command data
    // ------------------------------------------------------
    sensor_msgs::msg::LaserScan latest_scan_;
    std::vector<LaserSegment> latest_segments_;
    sensor_msgs::msg::Imu latest_imu_;
    geometry_msgs::msg::Twist current_twist_;

    // ------------------------------------------------------
    // Human tracking data
    // ------------------------------------------------------
    bool human_locked_;
    double human_centre_offset_;
    double human_distance_m_;
    rclcpp::Time last_human_tracking_time_;

    // ------------------------------------------------------
    // Behaviour state
    // ------------------------------------------------------
    bool blind_autonomous_active_;

    // Set to true while an unexpected object is inside the safety zone.
    bool safety_zone_violated_;

    // Baseline scan captured on first entry to HUMAN_DETECTED.
    sensor_msgs::msg::LaserScan safety_zone_baseline_scan_;
    bool safety_zone_baseline_captured_;

    NavigationState current_state_;

    double target_angle_;
    double target_range_;
    double stop_distance_m_;
    int stop_counter_;

    // IMU-assisted rotation tracking for ALIGNING state.
    bool imu_available_;        // true once at least one /imu message has arrived
    double align_start_yaw_;    // yaw recorded at the moment ALIGNING begins
    bool align_yaw_initialised_; // true once align_start_yaw_ has been captured

    // ------------------------------------------------------
    // Random tools
    // ------------------------------------------------------
    std::default_random_engine random_engine_;
    std::uniform_real_distribution<double> turn_dist_;
};

#endif  // MECHELANGELO_BEHAVIOUR__BEHAVIOUR_HPP_
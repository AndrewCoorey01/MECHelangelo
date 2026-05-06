#include "behaviour.hpp"

#include <iostream>
#include <functional>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>

using std::cout;
using std::endl;
using namespace std::chrono_literals;

// -------------------------------
// Behaviour constants
// -------------------------------

// Control loop runs every 100 ms.
static constexpr double kControlPeriodSeconds = 0.1;

// Movement tuning
static constexpr double kForwardSpeed = 0.26;       // m/s
static constexpr double kTurnSpeed = 0.6;           // rad/s
static constexpr double kAngleGain = 0.8;           // proportional turning gain
static constexpr double kAlignmentTolerance = 0.10; // radians, about 5.7 degrees

// Stop 1.5 m away from the object/wall in front
static constexpr double kStopDistance = 1.5; // m

// 30 loops x 0.1 s = 3 seconds
static constexpr int kStopDurationLoops = 30;

// Ignore laser points very close to the robot body
static constexpr double kMinValidRange = 0.5; // m

// Front scan window used while moving forward
static constexpr double kFrontCheckAngle = 15.0 * M_PI / 180.0; // +/- 15 degrees

MechelangeloBehaviour::MechelangeloBehaviour()
    : Node("mechelangelo_behaviour"), blind_autonomous_active_(false), random_engine_(std::random_device{}()), turn_dist_(-1.0, 1.0), current_state_(NavigationState::SEARCHING), target_angle_(0.0), target_range_(0.0), stop_counter_(0)
{
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&MechelangeloBehaviour::laserScanCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);

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
    RCLCPP_INFO(this->get_logger(), "Executing blind autonomous behavior.");

    blind_autonomous_active_ = true;
    current_state_ = NavigationState::SEARCHING;
    target_angle_ = 0.0;
    target_range_ = 0.0;
    stop_counter_ = 0;
}

void MechelangeloBehaviour::mappedAutonomous()
{
    RCLCPP_INFO(this->get_logger(), "Executing mapped autonomous behavior.");
}

void MechelangeloBehaviour::controlLoop()
{
    if (!blind_autonomous_active_)
    {
        return;
    }

    geometry_msgs::msg::Twist twist;

    // Safety: wait until valid LaserScan data exists
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Waiting for valid LaserScan data...");

        stopRobot(twist);
        current_twist_ = twist;
        cmd_vel_publisher_->publish(twist);
        return;
    }

    switch (current_state_)
    {

    // ------------------------------------------------------
    // SEARCHING:
    // Find the longest valid laser ray once.
    // This gives the robot its chosen direction.
    // ------------------------------------------------------
    case NavigationState::SEARCHING:
    {
        stopRobot(twist);

        double longest_angle = 0.0;
        double longest_range = 0.0;

        if (!getLongestRange(longest_angle, longest_range))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "SEARCHING: No valid LaserScan range found.");
            break;
        }

        target_angle_ = longest_angle;
        target_range_ = longest_range;

        RCLCPP_INFO(
            this->get_logger(),
            "SEARCHING: Longest scan found at %.2f deg, range %.2f m",
            target_angle_ * 180.0 / M_PI,
            target_range_);

        current_state_ = NavigationState::ALIGNING;
        break;
    }

    // ------------------------------------------------------
    // ALIGNING:
    // Rotate toward the saved longest laser direction.
    //
    // Since this version does not use odometry, it estimates
    // the remaining angle by subtracting the commanded angular
    // movement every 100 ms.
    // ------------------------------------------------------
    case NavigationState::ALIGNING:
    {
        twist.linear.x = 0.0;

        if (std::fabs(target_angle_) <= kAlignmentTolerance)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "ALIGNING: Aligned with chosen laser direction. Starting forward movement.");

            twist.angular.z = 0.0;
            current_state_ = NavigationState::MOVING;
            break;
        }

        double turn_cmd = std::clamp(
            target_angle_ * kAngleGain,
            -kTurnSpeed,
            kTurnSpeed);

        twist.angular.z = turn_cmd;

        // Estimate how much the robot turned during this control step.
        // Positive angular.z reduces a positive target angle.
        target_angle_ -= turn_cmd * kControlPeriodSeconds;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "ALIGNING: Remaining angle %.2f deg, angular command %.2f rad/s",
            target_angle_ * 180.0 / M_PI,
            twist.angular.z);

        break;
    }

        // ------------------------------------------------------
        // MOVING:
        // Move forward only.
        // Stop when the front LaserScan distance reaches 1.5 m.
        // ------------------------------------------------------
    case NavigationState::MOVING:
    {
        double front_range = getFrontRange();

        // If front_range is infinity, that means there is no valid obstacle
        // in the front check cone. Treat that as clear space.
        if (std::isinf(front_range))
        {
            twist.linear.x = kForwardSpeed;
            twist.angular.z = 0.0;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "MOVING: Front is clear. Driving forward.");

            break;
        }

        // NaN is genuinely bad scan data.
        if (std::isnan(front_range))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "MOVING: Front scan is NaN. Returning to SEARCHING for safety.");

            stopRobot(twist);
            current_state_ = NavigationState::SEARCHING;
            break;
        }

        // Stop when a valid front obstacle/wall is within 1.5 m.
        if (front_range <= kStopDistance)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "MOVING: Reached safety stop distance. Front range = %.2f m",
                front_range);

            stopRobot(twist);
            stop_counter_ = 0;
            current_state_ = NavigationState::STOPPED;
            break;
        }

        // Otherwise keep moving forward.
        twist.linear.x = kForwardSpeed;
        twist.angular.z = 0.0;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "MOVING: Driving forward. Front range = %.2f m",
            front_range);

        break;
    }

    // ------------------------------------------------------
    // STOPPED:
    // Pause for 3 seconds, then look for the next longest scan.
    // ------------------------------------------------------
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


    // ------------------------------------------------------
    // HUMAN_DETECTED:
    // TARGET HUMAN, move towards target and stop 1.5m away from them.
    // ------------------------------------------------------
    case NavigationState::HUMAN_DETECTED:
    {

    }

    default:
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Unknown navigation state. Returning to SEARCHING.");

        stopRobot(twist);
        current_state_ = NavigationState::SEARCHING;
        break;
    }
    }

    current_twist_ = twist;
    cmd_vel_publisher_->publish(twist);
}

void MechelangeloBehaviour::stopRobot(geometry_msgs::msg::Twist &twist)
{
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;

    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
}

double MechelangeloBehaviour::getMinimumRange(double start_angle, double end_angle) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return std::numeric_limits<double>::infinity();
    }

    int start_index = angleToIndex(start_angle);
    int end_index = angleToIndex(end_angle);

    if (start_index < 0 || end_index < 0)
    {
        return std::numeric_limits<double>::infinity();
    }

    if (start_index > end_index)
    {
        std::swap(start_index, end_index);
    }

    double min_range = std::numeric_limits<double>::infinity();

    for (int i = start_index; i <= end_index; ++i)
    {
        if (i < 0 || i >= static_cast<int>(latest_scan_.ranges.size()))
        {
            continue;
        }

        double range = latest_scan_.ranges[i];

        if (isRangeValid(range) && range < min_range)
        {
            min_range = range;
        }
    }

    return min_range;
}

double MechelangeloBehaviour::getFrontRange() const
{
    // Use a small cone in front of the robot instead of exactly one ray.
    // This is more stable because one LaserScan reading can be noisy.
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

    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
    {
        double range = latest_scan_.ranges[i];

        if (isRangeValid(range) && range > max_range)
        {
            max_range = range;
            max_index = static_cast<int>(i);
        }
    }

    if (max_index < 0)
    {
        return false;
    }

    out_angle = latest_scan_.angle_min + max_index * latest_scan_.angle_increment;
    out_range = max_range;

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

    if (index < 0)
    {
        index = 0;
    }

    if (index >= static_cast<int>(latest_scan_.ranges.size()))
    {
        index = static_cast<int>(latest_scan_.ranges.size()) - 1;
    }

    return index;
}

bool MechelangeloBehaviour::isRangeValid(double range) const
{
    return std::isfinite(range) && range > kMinValidRange;
}

void MechelangeloBehaviour::laserScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = *msg;
}

void MechelangeloBehaviour::longestLaserScan()
{
    double longest_angle = 0.0;
    double longest_range = 0.0;

    if (!getLongestRange(longest_angle, longest_range))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "No valid laser scan data available for longest scan calculation.");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Longest valid laser scan: Range = %.2f m at Angle = %.2f degrees",
        longest_range,
        longest_angle * 180.0 / M_PI);
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
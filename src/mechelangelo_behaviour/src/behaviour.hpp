#ifndef BEHAVIOUR_HPP
#define BEHAVIOUR_HPP

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// #include <geometry_msgs/msg/point.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include <cmath>
// #include <algorithm>
// #include "visualization_msgs/msg/marker_array.hpp"
// #include <chrono>
// #include <thread>
// #include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
// #include "std_srvs/srv/set_bool.hpp"


class MechelangeloBehaviour : public rclcpp::Node
{
public:
    MechelangeloBehaviour();

    ~MechelangeloBehaviour();

    void run(bool sim_mode);

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    // void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);    
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;

};  

#endif // BEHAVIOUR_HPP




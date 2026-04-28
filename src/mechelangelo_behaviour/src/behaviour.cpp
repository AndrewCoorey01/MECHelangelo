#include "behaviour.hpp"

#include <iostream>
#include <functional>

using std::cout;
using std::endl;

MechelangeloBehaviour::MechelangeloBehaviour() : Node("mechelangelo_behaviour")
{
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&MechelangeloBehaviour::laserScanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been started.");
}


MechelangeloBehaviour::~MechelangeloBehaviour()
{
    RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been stopped.");
}


void MechelangeloBehaviour::run(bool sim_mode)
{
    RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node is running.");
    rclcpp::spin(shared_from_this());

    if(sim_mode == true) {
        RCLCPP_INFO(this->get_logger(), "Running in simulation mode.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Running in real robot mode.");
    }

}

void MechelangeloBehaviour::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received laser scan data with %zu ranges.", msg->ranges.size());
    // Process the laser scan data here
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MechelangeloBehaviour>();
    node->run(true); // Set to true for simulation mode, false for real robot mode use input or parameter to determine this
    rclcpp::shutdown();
    return 0;
}
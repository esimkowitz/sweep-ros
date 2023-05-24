/*
Copyright (c) 2023, Evan Simkowitz
See LICENSE.md in the root directory of this repo for license information.
*/

#include "rclcpp/rclcpp.hpp"
#include "../include/sweep_ros.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    // Initialize Node and handles
    rclcpp::init(argc, argv);
    std::shared_ptr<sweep_ros::SweepRos> node = std::make_shared<sweep_ros::SweepRos>();
    rclcpp::spin(node);

    // Stop Scanning & Destroy Driver
    node->device.stop_scanning();
    rclcpp::shutdown();
    return 0;
}

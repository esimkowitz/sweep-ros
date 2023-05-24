/*
Copyright (c) 2023, Evan Simkowitz
See LICENSE.md in the root directory of this repo for license information.
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include <pcl/point_types.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sweep/sweep.hpp"

namespace sweep_ros {

    class SweepRos : public rclcpp::Node
    {
    public:
        SweepRos();

        sweep::sweep device = nullptr;

    private:
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Clock ros_clock_;

        void timer_callback();

        void publish_scan(const sweep::scan *scan, std::string frame_id);
    };
};
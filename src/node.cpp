
/*The MIT License (MIT)
 *
 * Copyright (c) 2017, Scanse, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include <pcl/point_types.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sweep/sweep.hpp"

using namespace std;

class SweepNode : public rclcpp::Node
{
public:
    SweepNode() : Node("sweep_node")
    {
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("serial_baudrate", 115200);
        this->declare_parameter("rotation_speed", 5);
        this->declare_parameter("sample_rate", 500);
        this->declare_parameter("frame_id", "laser_frame");
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sweep_pc2", 1000);
        ros_clock_ = rclcpp::Clock(RCL_ROS_TIME);

        try
        {
            device = sweep::sweep(this->get_parameter("serial_port").as_string().c_str(), (int32_t)(this->get_parameter("serial_baudrate").as_int()));

            // Send Rotation Speed
            device.set_motor_speed((int32_t)(this->get_parameter("rotation_speed").as_int()));

            // Send Sample Rate
            device.set_sample_rate((int32_t)(this->get_parameter("sample_rate").as_int()));

            RCLCPP_INFO(this->get_logger(), "expected rotation frequency: %d (Hz)", (int32_t)(this->get_parameter("rotation_speed").as_int()));

            // Start Scan
            device.start_scanning();
        }
        catch (const sweep::device_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
            throw e;
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(this->get_parameter("sample_rate").as_int()), std::bind(&SweepNode::timer_callback, this));
    }

    sweep::sweep device = nullptr;

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock ros_clock_;

    void timer_callback()
    {
        try
        {
            const sweep::scan scan = device.get_scan();
            publish_scan(&scan, this->get_parameter("frame_id").as_string());
        }
        catch (const sweep::device_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    void publish_scan(const sweep::scan *scan, std::string frame_id)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        rclcpp::Time ros_now = ros_clock_.now();

        float angle;
        int32_t range;
        float x;
        float y;
        int i = 0;

        cloud.height = 1;
        cloud.width = scan->samples.size();
        cloud.points.resize(cloud.width * cloud.height);

        for (const sweep::sample &sample : scan->samples)
        {
            range = sample.distance;
            angle = ((float)sample.angle / 1000); // millidegrees to degrees

            // Polar to Cartesian Conversion
            x = (range * cos(DEG2RAD(angle))) / 100;
            y = (range * sin(DEG2RAD(angle))) / 100;

            cloud.points[i].x = x;
            cloud.points[i].y = y;
            i++;
        }

        // Convert pcl PC to ROS PC2
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = ros_now;

        RCLCPP_DEBUG(this->get_logger(), "Publishing a full scan");
        publisher_->publish(cloud_msg);
    }
};

int main(int argc, char *argv[])
{
    // Initialize Node and handles
    rclcpp::init(argc, argv);
    std::shared_ptr<SweepNode> node = std::make_shared<SweepNode>();
    rclcpp::spin(node);

    // Stop Scanning & Destroy Driver
    node->device.stop_scanning();
    rclcpp::shutdown();
    return 0;
}

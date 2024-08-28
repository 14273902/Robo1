// src/laser_scan_processor.cpp

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class LaserScanProcessor : public rclcpp::Node
{
public:
    LaserScanProcessor()
    : Node("laser_scan_processor")
    {
        // Subscriber to the laser scan data
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor::laserScanCallback, this, std::placeholders::_1));

        // Publisher for filtered laser scan data
        filtered_laser_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/filtered_scan", 10);

        // Define the angle of interest (in radians)
        angle_of_interest_ = 0.0; // Example angle, adjust as needed
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr filtered_laser_pub_;
    double angle_of_interest_;

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Get the range readings
        const auto& ranges = msg->ranges;

        // Compute index for the given angle
        double angle_min = msg->angle_min;
        double angle_increment = msg->angle_increment;
        int index = static_cast<int>((angle_of_interest_ - angle_min) / angle_increment);

        if (index >= 0 && index < ranges.size())
        {
            RCLCPP_INFO(this->get_logger(), "Range at angle %f: %f", angle_of_interest_, ranges[index]);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Angle %f is out of range", angle_of_interest_);
        }

        // Select a subset of range values (e.g., from 0 to 90 degrees)
        double angle_start = 0.0;
        double angle_end = 1.57; // 90 degrees in radians
        int start_index = static_cast<int>((angle_start - angle_min) / angle_increment);
        int end_index = static_cast<int>((angle_end - angle_min) / angle_increment);

        auto filtered_ranges = std::make_shared<std_msgs::msg::Float32MultiArray>();
        filtered_ranges->data.clear();

        for (int i = start_index; i <= end_index && i < ranges.size(); ++i)
        {
            filtered_ranges->data.push_back(ranges[i]);
        }

        // Republish the subset of range values
        filtered_laser_pub_->publish(*filtered_ranges);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}

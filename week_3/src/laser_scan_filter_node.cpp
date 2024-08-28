#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanFilterNode : public rclcpp::Node
{
public:
    LaserScanFilterNode()
    : Node("laser_scan_filter_node"), n_(5) // Set n to 5 or any desired value
    {
        // Subscribe to the original laser scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LaserScanFilterNode::scan_callback, this, std::placeholders::_1));
       
        // Create a publisher for the filtered laser scan data
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_scan = sensor_msgs::msg::LaserScan();

        // Copy header and other unchanged fields
        filtered_scan.header = msg->header;
        filtered_scan.angle_min = msg->angle_min;
        filtered_scan.angle_max = msg->angle_max;
        filtered_scan.angle_increment = msg->angle_increment;
        filtered_scan.time_increment = msg->time_increment;
        filtered_scan.scan_time = msg->scan_time;
        filtered_scan.range_min = msg->range_min;
        filtered_scan.range_max = msg->range_max;

        // Filter the ranges and intensities
        size_t original_size = msg->ranges.size();
        filtered_scan.ranges.reserve((original_size + n_ - 1) / n_); // Allocate memory for filtered ranges
        filtered_scan.intensities.reserve((original_size + n_ - 1) / n_); // Allocate memory for filtered intensities

        for (size_t i = 0; i < original_size; i += n_)
        {
            filtered_scan.ranges.push_back(msg->ranges[i]);
            if (!msg->intensities.empty())
            {
                filtered_scan.intensities.push_back(msg->intensities[i]);
            }
        }

        // Adjust the angle increment to reflect the reduced number of points
        filtered_scan.angle_increment = msg->angle_increment * n_;

        // Publish the filtered laser scan
        publisher_->publish(filtered_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    size_t n_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanFilterNode>());
    rclcpp::shutdown();
    return 0;
}

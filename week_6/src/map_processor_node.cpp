#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
        : Node("map_processor_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        odom_covar_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&MapProcessorNode::poseCallback, this, std::placeholders::_1));

        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MapProcessorNode::odomCallback, this, std::placeholders::_1));

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::cout << "scanCallback" << std::endl;

        // Save the latest laser scan data
        last_scan_msg_ = msg;

        // Convert laser scan to image (Image C)
        cv::Mat img = laserScanToMat(msg);

        if (image_b_captured_) // Only proceed if Image B (entire map edges) is ready
        {
            if (!image_c_captured_)
            {
                image_c = img.clone();
                image_c_captured_ = true;

                // Display Image C (Laser scan image)
                cv::rotate(image_c, image_c, cv::ROTATE_90_COUNTERCLOCKWISE);
                cv::imshow("Image C (Laser Scan)", image_c);
                cv::waitKey(1);
            }
            else
            {
                // Update Image C with the new scan data
                image_c = img.clone();
                cv::rotate(image_c, image_c, cv::ROTATE_90_COUNTERCLOCKWISE);
                cv::imshow("Image C (Laser Scan)", image_c);
                cv::waitKey(1);

                // Estimate yaw change between Image B (full map edges) and Image C (laser scan)
                calculateYawChange();
                relative_orientaion_ += angle_difference_;
                RCLCPP_INFO(this->get_logger(), "Relative Orientation %f", relative_orientaion_);
                moveRobot();
            }
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr poseMsg){
        estX = poseMsg->pose.pose.position.x;
        estY = poseMsg->pose.pose.position.y;

        estimated_pose_found = true;

        findDistanceToActual();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr poseMsg){
        actuX = poseMsg->pose.pose.position.x;
        actuY = poseMsg->pose.pose.position.y;

        actual_pose_found = true;

        findDistanceToActual();
    }

    void findDistanceToActual(){
        if(estimated_pose_found && actual_pose_found){
            diffX = actuX - estX;
            diffY = actuY - estY;
            distance_calculated = true;

        }else{
            distance_calculated = false;
            RCLCPP_INFO(this->get_logger(), "Not Enough Odometrey Information to find distance");
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        if (!last_scan_msg_)
        {
            // If no laser scan data available, return
            RCLCPP_WARN(this->get_logger(), "No laser scan data available yet.");
            return;
        }

        // Convert the full occupancy grid to an image (Full Map)
        cv::Mat fullMapImage = occupancyGridToImage(mapMsg);

        // Extract a section of the map around the robot (Image A)
        cv::Mat imageA = extractMapAroundRobot(mapMsg, last_scan_msg_);

        // Apply Canny edge detection on the full map to create Image B
        cv::Mat edges;
        double lowerThreshold = 50;
        double upperThreshold = 150;
        cv::Canny(fullMapImage, edges, lowerThreshold, upperThreshold);

        // Update image_b to store the full map edges (Image B)
        image_b = edges.clone();

        // Display Image A (Map section around the robot)
        cv::rotate(imageA, imageA, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(imageA, imageA, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::imshow("Image A (Map around robot)", imageA);

        // Display Image B (Edges of the full map)
        cv::rotate(image_b, image_b, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::imshow("Image B (Full map edges)", image_b);
        image_b_captured_ = true;
        cv::waitKey(1);
    }

        void moveRobot()
    {
        // if(!move_robot_){
            RCLCPP_INFO(this->get_logger(), "angle difference between image b and c %f", angle_difference_);

            
            geometry_msgs::msg::Twist cmd_msg;
            if(angle_difference_ <= 0.50 || angle_difference_ >= 0.5){
                // Create a Twist message to move the robot
                
                cmd_msg.angular.z = 0.2; // Rotate the robot at a small angular velocity
                cmd_publisher_->publish(cmd_msg);
                RCLCPP_INFO(this->get_logger(), "Robot is rotating...");

                // Set a flag to stop after a small rotation
                // move_robot_ = true;

                // Add a timer to stop the robot after a short period of rotation (e.g., 2 seconds)
                auto stop_timer = this->create_wall_timer(
                    std::chrono::milliseconds(500), [this, &cmd_msg]()
                    {
                        cmd_msg.angular.z = 0.0; // Stop rotation
                        cmd_publisher_->publish(cmd_msg);
                        RCLCPP_INFO(this->get_logger(), "Robot stopped.");

                        // move_robot_ = false;

                        // reprocessAfterMovement();
                        // stopRobot();
                    }
                );
            }else{
                cmd_msg.angular.z = 0.0;
                cmd_publisher_->publish(cmd_msg);
            }

            if(distance_calculated){
                if(diffX < 0.05){
                    cmd_msg.linear.x = -0.4;
                }else if(diffX > 0.05){
                    cmd_msg.linear.x = 0.4;
                }else{
                    cmd_msg.linear.x = 0.0;
                }

                if(diffY < 0.05){
                    cmd_msg.linear.x = -0.4;
                }else if(diffY > 0.05){
                    cmd_msg.linear.x = 0.4;
                }else{
                    cmd_msg.linear.x = 0.0;
                }
            }
            cmd_publisher_->publish(cmd_msg);
        // }
    }

    // Function to stop the robot after it rotates
    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.angular.z = 0.0; // Stop rotation
        cmd_publisher_->publish(cmd_msg);
        RCLCPP_INFO(this->get_logger(), "Robot stopped.");

        // Reset the flag to allow the next movement after processing
        move_robot_ = false;

        // Re-run the localization process after the robot has moved
        reprocessAfterMovement();
    }

    // Re-run the entire process after the robot has moved
    void reprocessAfterMovement()
    {
        if (image_b_captured_ && image_c_captured_)
        {
            // Reset flags and re-trigger the scan and map callbacks
            image_c_captured_ = false;

            // Wait for new data from the laser scan and occupancy grid
            RCLCPP_INFO(this->get_logger(), "Re-running image processing and yaw calculation.");
        }
    }

    // Converts the full occupancy grid to a grayscale image (binary map)
    cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

        cv::Mat temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        for (row = 0; row < grid->info.height; row++)
        {
            for (col = 0; col < grid->info.width; col++)
            {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1)
                {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                }
                else
                {
                    temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        // Apply morphological operation (erosion) to clean up the image
        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                           0, 1, 0,
                           0, 0, 0);
        cv::erode(temp_img, temp_img, kernel);

        return temp_img;
    }

    // Extracts a section of the map around the robot's estimated position
    cv::Mat extractMapAroundRobot(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg, const sensor_msgs::msg::LaserScan::SharedPtr scanMsg)
    {
        // Get map dimensions and resolution
        int mapWidth = mapMsg->info.width;
        int mapHeight = mapMsg->info.height;
        double mapResolution = mapMsg->info.resolution;

        // Convert occupancy grid to an image
        cv::Mat mapImage = occupancyGridToImage(mapMsg);

        // Estimate robot position from laser scan data (using closest obstacle)
        double angle = scanMsg->angle_min;
        double closestRange = scanMsg->range_max;
        double closestAngle = 0.0;

        for (size_t i = 0; i < scanMsg->ranges.size(); ++i)
        {
            double range = scanMsg->ranges[i];
            if (range < closestRange && range > scanMsg->range_min)
            {
                closestRange = range;
                closestAngle = angle;
            }
            angle += scanMsg->angle_increment;
        }

        // Convert closest obstacle position to Cartesian coordinates
        double robotX = closestRange * cos(closestAngle);
        double robotY = closestRange * sin(closestAngle);

        // Convert robot's position (in meters) to pixel coordinates in the map
        int robotPx = static_cast<int>((robotX - mapMsg->info.origin.position.x) / mapResolution);
        int robotPy = static_cast<int>((robotY - mapMsg->info.origin.position.y) / mapResolution);

        // Define region around the robot (100x100 pixels for example)
        int regionSize = 100;
        int xStart = std::max(0, robotPx - regionSize / 2);
        int yStart = std::max(0, robotPy - regionSize / 2);
        int width = std::min(regionSize, mapWidth - xStart);
        int height = std::min(regionSize, mapHeight - yStart);

        // Extract the region from the map
        cv::Rect region(xStart, yStart, width, height);
        return mapImage(region);
    }

    // Converts laser scan data to an image (Image C)
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {
        int image_size = 500;
        float max_range = scan->range_max;
        cv::Mat img = cv::Mat::zeros(image_size, image_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max)
            {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle) * image_size) / (2 * max_range)) + image_size / 2;
                int y = static_cast<int>((range * sin(angle) * image_size) / (2 * max_range)) + image_size / 2;
                if (x >= 0 && x < image_size && y >= 0 && y < image_size)
                {
                    img.at<uchar>(y, x) = 255;
                }
            }
        }

        return img;
    }

    void calculateYawChange()
    {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(image_b, image_c, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try
        {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (!transform_matrix.empty())
            {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }


    // void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
    //                             std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
    // {
    //     cv::Ptr<cv::ORB> orb = cv::ORB::create();
    //     std::vector<cv::KeyPoint> keypoints1, keypoints2;
    //     cv::Mat descriptors1, descriptors2;

    //     orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    //     orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

    //     cv::BFMatcher matcher(cv::NORM_HAMMING);
    //     std::vector<cv::DMatch> matches;
    //     matcher.match(descriptors1, descriptors2, matches);

    //     // Sort matches based on distance
    //     std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
    //               { return a.distance < b.distance; });

    //     // Keep the best 30% of matches
    //     size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.3);
    //     std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

    //     // Extract the matched points
    //     for (const auto &match : goodMatches)
    //     {
    //         srcPoints.push_back(keypoints1[match.queryIdx].pt);
    //         dstPoints.push_back(keypoints2[match.trainIdx].pt);
    //     }
    // }

        void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : matches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_covar_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat image_b, image_c;
    bool image_b_captured_ = false;
    bool image_c_captured_ = false;

    double estX, estY, actuX, actuY;
    double diffX, diffY;
    bool estimated_pose_found = false;
    bool actual_pose_found = false;
    bool distance_calculated = false;

    double angle_difference_;
    double relative_orientaion_ = 0.0;
        bool move_robot_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
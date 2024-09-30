// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp>
// #include <opencv2/calib3d.hpp>
// #include <vector>

// class ScanToImageNode : public rclcpp::Node {
// public:
//     ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientaion_(0.0) {
//         scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
//         cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//         RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
//     }

// private:
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//         cv::Mat current_image = laserScanToMat(msg);

//         if (!first_image_captured_) {
//             first_image_ = current_image;
//             first_image_captured_ = true;
//             RCLCPP_INFO(this->get_logger(), "First image captured.");
//         } else if (!second_image_captured_) {
//             second_image_ = current_image;
//             second_image_captured_ = true;
//             RCLCPP_INFO(this->get_logger(), "Second image captured.");
//         } else {
//             second_image_ = current_image;
//             calculateYawChange();
//             first_image_ = second_image_;
//             second_image_captured_ = true;
//         }

//         cv::imshow("Laser Scan Image", current_image);
//         cv::waitKey(1);
//     }

//     cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
//         //cv::imshow("Current Image", current_image);
//         //cv::waitKey(1);

//         int width = 500, height = 500;
//         cv::Mat image(height, width, CV_8UC1, cv::Scalar(0));

//         double angle_min = scan->angle_min;
//         double angle_increment = scan->angle_increment;
//         std::vector<float> ranges = scan->ranges;

//         for (size_t i = 0; i < ranges.size(); ++i) {
//             double angle = angle_min + i * angle_increment;
//             float range = ranges[i];
//             int x = static_cast<int>(width / 2 + range * cos(angle));
//             int y = static_cast<int>(height / 2 - range * sin(angle));
//             if (x >= 0 && x < width && y >= 0 && y < height) {
//                 image.at<uchar>(y, x) = 255;
//             }
//         }

//         return image;
//     }

//     void calculateYawChange() {
//         std::vector<cv::Point2f> srcPoints, dstPoints;
//         detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

//         if (srcPoints.size() < 4 || dstPoints.size() < 4) {
//             RCLCPP_WARN(this->get_logger(), "Not enough points for transformation.");
//             return;
//         }

//         cv::Mat inliers;
//         cv::Mat H = cv::findHomography(srcPoints, dstPoints, cv::RANSAC, 3, inliers);

//         if (H.empty()) {
//             RCLCPP_WARN(this->get_logger(), "Homography matrix is empty.");
//             return;
//         }

//         double angle = std::atan2(H.at<double>(1, 0), H.at<double>(0, 0)) * 180.0 / CV_PI;
//         angle_difference_ = angle;
//         relative_orientaion_ += angle_difference_;

//         RCLCPP_INFO(this->get_logger(), "Estimated yaw change: %.2f degrees", angle_difference_);
//         RCLCPP_INFO(this->get_logger(), "Relative orientation: %.2f degrees", relative_orientaion_);
//     }

//     void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
//                                 std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
//         cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
//         std::vector<cv::KeyPoint> keypoints1, keypoints2;
//         cv::Mat descriptors1, descriptors2;

//         orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
//         orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

//         cv::BFMatcher matcher(cv::NORM_HAMMING);
//         std::vector<cv::DMatch> matches;
//         matcher.match(descriptors1, descriptors2, matches);

//         std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
//             return a.distance < b.distance;
//         });

//         size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
//         std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

//         for (const auto& match : goodMatches) {
//             srcPoints.push_back(keypoints1[match.queryIdx].pt);
//             dstPoints.push_back(keypoints2[match.trainIdx].pt);
//         }
//     }

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

//     cv::Mat first_image_, second_image_;
//     bool first_image_captured_ = false;
//     bool second_image_captured_ = false;

//     double angle_difference_;
//     double relative_orientaion_ = 0.0;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ScanToImageNode>());
//     rclcpp::shutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ScanToImageNode : public rclcpp::Node {
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientation_(0.0) {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    // void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    //     cv::Mat image = laserScanToMat(msg);

    //     if (!first_image_captured_) {
    //         first_image_ = image;
    //         first_image_captured_ = true;
    //         cv::imshow("First Image", first_image_);
    //         cv::waitKey(1);  // Wait for display
    //     } else if (!second_image_captured_) {
    //         second_image_ = image;
    //         second_image_captured_ = true;
    //         cv::imshow("Second Image", second_image_);
    //         cv::waitKey(1);  // Wait for display
    //         calculateYawChange();
    //     } else {
    //         first_image_ = second_image_;
    //         second_image_ = image;
    //         calculateYawChange();
    //         cv::imshow("Updated Image", second_image_);
    //         cv::waitKey(1);  // Reduced to 1ms
    //     }
    // }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        cv::Mat img = laserScanToMat(msg);

        if(first_image_captured_){
            first_image_ = img.clone();
            first_image_captured_ = true;
            cv::imshow("First Image", first_image_);
            cv::waitKey(1);
        }
        else if (!second_image_captured_){
            second_image_ = img.clone();
            second_image_captured_ = true;
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1);
        }
        else {
            first_image_ = second_image_;
            second_image_ = img.clone();
            cv::imshow("Next Image", second_image_);
            cv::waitKey(1);
            calculateYawChange();
            relative_orientation_ = relative_orientation_ + angle_difference_;
            RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f", relative_orientation_);
        }
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        cv::Mat img = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        float angle = scan->angle_min;
        for (const auto& range : scan->ranges) {
            if (range >= scan->range_min && range <= scan->range_max) {
                int x = static_cast<int>((range * cos(angle)) * 100) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * 100) + img_size / 2;

                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    img.at<uchar>(y, x) = 255;
                }
            }
            angle += scan->angle_increment;
        }

        return img;
    }

    void calculateYawChange() {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough features to calculate yaw change.");
            return;
        }

        cv::Mat affine = cv::estimateAffine2D(srcPoints, dstPoints);

        if (!affine.empty()) {
            double angle = atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
            angle_difference_ = angle * 180.0 / CV_PI;
            relative_orientation_ += angle_difference_;
            RCLCPP_INFO(this->get_logger(), "Yaw change: %.2f degrees, Relative orientation: %.2f degrees",
                        angle_difference_, relative_orientation_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to estimate affine transformation.");
        }
    }

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

        // Determine the number of top matches to keep (15% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 15%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat first_image_, second_image_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;
    double relative_orientation_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}

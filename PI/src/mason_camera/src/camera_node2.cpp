// #include <mason_camera/detection.h>

#include <cv_bridge/cv_bridge.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#define MILLISECONDS 100000

class CameraNode : public rclcpp::Node {
   public:
    CameraNode() : Node("camera_node") {
        RCLCPP_INFO(this->get_logger(), "Camera node created");

        auto camera_info_callback =
            [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) -> void {
                this->K = cv::Mat k{ 3, 3, CV_64F, msg.k };
                this->P = cv::Mat p{ 3, 4, CV_64F, msg.p };
            };

        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", 10,
            std::bind(&CameraNode::depth_callback, this, std::placeholders::_1));
        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10, camera_info_callback);
    }
    
    struct Joint {
        double x;
        double y;
        double probability;
    };
   private:
    cv::Mat preProcess(cv::Mat depth_image) {
        auto height = depth_image.rows;
        auto width = depth_image.cols;

        auto trimmed_image = depth_image.colRange(INVALID_DEPTH_BAND, width);

        cv::Mat normalized_image;

        cv::blur(trimmed_image, normalized_image, cv::Size(2, 2));

        return normalized_image;
    }

    cv::Mat removeZeros(cv::Mat depth_image) {
        cv::Mat new_image = depth_image.clone();
        // auto view = new_image.reshape(1, new_image.total());

        // std::for_each(std::execution::par, view.begin<uchar>(), view.end<uchar>(), [](uchar& pixel) {
        //     if (pixel == 0) {
        //         pixel = 255;
        //     }
        // });
        for (int i = 0; i < depth_image.rows; i++) {
            for (int j = 0; j < depth_image.cols; j++) {
                if (depth_image.at<uint16_t>(i, j) == 0) {
                    new_image.at<uint16_t>(i, j) = 255;
                }
            }
        }

        return new_image;
    }

    std::vector<Joint> getJoints(cv::Mat depth_image, int frame_no) {
        std::vector<Joint> joints;

        depth_image = preProcess(depth_image);

        auto height = depth_image.rows;
        auto width = depth_image.cols;

        cv::Mat min_depth_image;
        cv::Mat max_depth_image;

        // auto element = getStructuringElement(cv::MORPH_RECT, cv::Size(160, 160));  
        // cv::erode(removeZeros(depth_image), min_depth_image, element);
        // cv::dilate(depth_image, max_depth_image, element);

        // new image with same height and width, all values are black
        cv::Mat new_image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                // double ratio =
                //     (depth_image.at<uint16_t>(i, j) - min_depth_image.at<uint16_t>(i, j)) /
                //     (max_depth_image.at<uint16_t>(i, j) - min_depth_image.at<uint16_t>(i, j));
                // if (ratio > 0.45) {
                //     new_image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
                //     joints.push_back(Joint(i, j, ratio));
                // }
                if (depth_image.at<uint16_t>(i, j) > 125) {
                    new_image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
                    joints.push_back(Joint(i, j, 1));
                }
                
            }
        }

        std::string filename = "src/frames/frame_" + std::to_string(frame_no) + ".png";
        imwrite(filename, new_image);

        return joints;
    }
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

        if (msg->header.stamp.nanosec - last_timestamp < 5000 * MILLISECONDS) return;

        RCLCPP_INFO(this->get_logger(), "Received depth image");

        last_timestamp = msg->header.stamp.nanosec;
        frame_no++;

        auto image = msg->data;
        // RCLCPP_INFO(this->get_logger(), "Image size: %ld", image.size());

        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg);
        this->depth_image = cv_ptr->image;

        // time it

        auto start = std::chrono::high_resolution_clock::now();
        auto joints = getJoints(depth_image, frame_no);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        
        std::cout << "Time taken: " << duration << "ms" << std::endl;

        std::cout << "Number of joints: " << joints.size() << std::endl;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    // I'm not entirely sure whether this is necessary
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr tf_sub;
    bool b = true;
    bool c = true;
    cv::Mat depth_image;
    cv::Mat color_image;
    cv::Mat K;
    cv::Mat P;
    uint32_t last_timestamp = 0;
    int frame_no = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
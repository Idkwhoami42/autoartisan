#include <mason_camera/detection.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#define MILLISECONDS 100000

class CameraNode : public rclcpp::Node {
   public:
    CameraNode() : Node("camera_node") {
        RCLCPP_INFO(this->get_logger(), "Camera node created");
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", 10,
            std::bind(&CameraNode::depth_callback, this, std::placeholders::_1));
    }

   private:
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
        auto joints = Detection::getJoints(depth_image, frame_no);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        
        std::cout << "Time taken: " << duration << "ms" << std::endl;

        std::cout << "Number of joints: " << joints.size() << std::endl;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
    bool b = true;
    bool c = true;
    cv::Mat depth_image;
    cv::Mat color_image;
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
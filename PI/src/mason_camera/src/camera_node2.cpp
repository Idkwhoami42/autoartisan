// #include <mason_camera/detection.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cmath>

#define MILLISECONDS 100000
#define WINDOW_SIZE 80
#define INVALID_DEPTH_BAND 72
#define SCALE_MM_TO_ROTATIONS 1.0

class CameraNode : public rclcpp::Node {
   public:
    CameraNode() : Node("camera_node") {
        RCLCPP_INFO(this->get_logger(), "Camera node created");

        this->depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", 10,
            std::bind(&CameraNode::depth_callback, this, std::placeholders::_1));
        this->camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10, 
            [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) -> void {
                // this->K = cv::Mat k{ 3, 3, CV_64F, msg->k };
                // this->P = cv::Mat p{ 3, 4, CV_64F, msg->p };
                this->K.assign(msg->k.begin(), msg->k.end());
                this->P.assign(msg->p.begin(), msg->p.end());
            });
        this->position_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odrive_position", 10,
            [this](std_msgs::msg::Float32MultiArray::UniquePtr msg) -> void {
                this->current_motor_pos_x = msg->data[0];
                this->current_motor_pos_y = msg->data[1];
            }
        );
        this->joint_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "detected_joints", 10);
    }
    
    struct Joint {
        double x;
        double y;
        double probability;
    };
   private:
    cv::Mat preProcess(cv::Mat depth_image) {
        // auto height = depth_image.rows;
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
        double centre_x = height / 2.0;
        double centre_y = width / 2.0;

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

                    if (get_distance(Joint(i, j, 1), centre_x, centre_y) < 50.0f) {
                        calculate_motor_pos(depth_image.at<float>(i, j), Joint(i, j, 1));
                    }
                }
                
            }
        }

        std::string filename = "src/frames/frame_" + std::to_string(frame_no) + ".png";
        imwrite(filename, new_image);

        return joints;
    }

    float get_distance(Joint j, double centre_x, double centre_y) {
        float diff_x = static_cast<float>(j.x - centre_x);
        float diff_y = static_cast<float>(j.y - centre_y);
        return std::sqrt(diff_x*diff_x + diff_y*diff_y);
    }

    void calculate_motor_pos(float Z, Joint j) {
        // Subscription is to /image_rect_raw - since the image is rectified, distortion is removed 
        // So inverse of K should be correct to use over pseudo-inverse of P
        
        // K = [fx 0 cx]     inverse(K) = [1/fx 0 -cx/fx]
        //     [0 fy cy]                  [0 1/fy -cy/fy]
        //     [0  0  1]                  [0     0     1]

        // inverse(K)*[x, y, 1] = [((x - cx) / fx), ((y - cy) / fy), 1]
        // Scale to real-world metrics: [X_mm, Y_mm, Z_mm] = Z*[((x - cx) / fx), ((y - cy) / fy), 1]

        // Since the toolhead and the camera move together (as they are attached) using a translation offset should be sufficient:
        // Translation diff. between camera centre and toolhead centre: [h_x, h_y, h_z]
        // Therefore total diff between current motor_pos and position motor will have to return to = [X_mm + h_x, Y_mm + h_y, Z_mm + h_z]
        // Although we can disregard the Z: (X_d, Y_d) = [(X_mm + h_x), (Y_mm + h_y)]
        // Returned motor coords: [motor_pos_x, motor_pos_y] = [curr_motor_pos_x, curr_motor_pos_y] + SCALE_MM_TO_ROTATIONS[X_d, Y_d]

        float X_mm = Z*(j.x - this->K[2]) / this->K[0];
        float Y_mm = Z*(j.y - this->K[5]) / this->K[4];

        RCLCPP_INFO(this->get_logger(), "3D Point in Camera Frame: (%.3f, %.3f, %.3f)", X_mm, Y_mm, Z);

        float motor_pos_x = this->current_motor_pos_x + SCALE_MM_TO_ROTATIONS*(X_mm + 0.0);
        float motor_pos_y = this->current_motor_pos_y + SCALE_MM_TO_ROTATIONS*(Y_mm + 0.0);

        RCLCPP_INFO(this->get_logger(), "Motor position of detected point: (%.3f, %.3f)", motor_pos_x, motor_pos_y);

        auto point_pos_msg = std_msgs::msg::Float32MultiArray();
        point_pos_msg.data = {motor_pos_x, motor_pos_y};

        this->joint_pub->publish(point_pos_msg);
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
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_sub;
    // I'm not entirely sure whether this is necessary
    // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr tf_sub;
    bool b = true;
    bool c = true;
    cv::Mat depth_image;
    cv::Mat color_image;
    // cv::Mat K;
    // cv::Mat P;
    float current_motor_pos_x = std::nanf(""), current_motor_pos_y = std::nan("");
    std::vector<float> K;
    std::vector<float> P;
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
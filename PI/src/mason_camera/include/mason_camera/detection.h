#ifndef MASON_CAMERA_DETECTION_H
#define MASON_CAMERA_DETECTION_H

#include <opencv2/opencv.hpp>

namespace Detection {
struct Joint {
    double x;
    double y;
    double probability;
};

std::vector<Joint> getJoints(cv::Mat depth_image, int frame_no);

cv::Mat removeZeros(cv::Mat depth_image);

cv::Mat preProcess(cv::Mat depth_image);

}  // namespace Detection

#endif  // MASON_CAMERA_DETECTION_H
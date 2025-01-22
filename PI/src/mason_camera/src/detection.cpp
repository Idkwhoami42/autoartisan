#include "mason_camera/detection.h"

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define WINDOW_SIZE 80
#define INVALID_DEPTH_BAND 72

cv::Mat Detection::preProcess(cv::Mat depth_image) {
    auto height = depth_image.rows;
    auto width = depth_image.cols;

    auto trimmed_image = depth_image.colRange(INVALID_DEPTH_BAND, width);

    cv::Mat normalized_image;

    cv::blur(trimmed_image, normalized_image, cv::Size(2, 2));

    return normalized_image;
}

cv::Mat Detection::removeZeros(cv::Mat depth_image) {
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

std::vector<Detection::Joint> Detection::getJoints(cv::Mat depth_image, int frame_no) {
    std::vector<Detection::Joint> joints;

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
            //     joints.push_back(Detection::Joint(i, j, ratio));
            // }
            if (depth_image.at<uint16_t>(i, j) > 125) {
                new_image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
                joints.push_back(Detection::Joint(i, j, 1));
            }
            
        }
    }

    std::string filename = "src/frames/frame_" + std::to_string(frame_no) + ".png";
    imwrite(filename, new_image);

    return joints;
}
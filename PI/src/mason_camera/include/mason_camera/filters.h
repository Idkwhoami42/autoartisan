#include <opencv2/opencv.hpp>

namespace filters {
cv::Mat max_window_filter(cv::Mat& image, int window_size);
cv::Mat min_window_filter(cv::Mat& image, int window_size);
}  // namespace filters
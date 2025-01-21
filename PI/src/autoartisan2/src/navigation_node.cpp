#include <memory>
#include <chrono>
#include <vector>
#include <cfloat>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
// #include "rclcpp/subscriber.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

struct JointingPath {
    std::pair<float, float> start = std::make_pair(0, 0);
    std::pair<float, float> end = std::make_pair(0, 0);
    bool traversed = false;
};

class Update {
public:
    Update() = default;

    Update(std::pair<float, float> currentPos) {
        this->t = std::chrono::system_clock::now();
        this->position = currentPos;
    }

    void updatePos(float x, float y) {
        this->t = std::chrono::system_clock::now();
        this->position = std::make_pair(x, y);
    }
private:
    std::chrono::time_point<std::chrono::system_clock> t;
    std::pair<float, float> position;
};

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("nav_node") {
        // /forward_position_controller/commands std_msgs/msg/Float64MultiArray
        this->update = Update(std::make_pair(0.0, 0.0));
        
        auto homing_callback =
            [this](geometry_msgs::msg::Point::UniquePtr msg) -> void {
                // In the hardware interface z is set based on whether homing was successful
                if (msg->z) {
                    this->wallSpanDegX = msg->x;
                    this->wallSpanDegY = msg->y;
                }
            };

        auto tracking_callback =
            [this](std_msgs::msg::Float64MultiArray::UniquePtr msg) -> void {
                // In the hardware interface z is set based on whether homing was successful
                this->update.updatePos(msg->data[0], msg->data[1]);
            };

        auto img_proc_callback =
            [this](sensor_msgs::msg::PointCloud::UniquePtr msg) -> void {
                // Call the member function from within the lambda
                this->imgProcCallback(std::move(msg));
            };
        
        this->homing_subscription_ = this->create_subscription<geometry_msgs::msg::Point>("limits", 10, homing_callback);
        this->position_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("forward_position_controller/position", 10, tracking_callback);
        // this->position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("forward_position_controller/position", 10, trackingCallback);
        this->img_proc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud>("detection", 10, img_proc_callback);
    }

    int calcLength(const std::vector<int> &points) {
        return points[points.size()-1] - points[0];
    }

    // 60 deg/s 
    // updates every 15 deg - 1cm of travel

    bool checkIfInsideJoint(float x1, float y1, float x4, float y4, float x, float y) {
        return x >= x1 && x <= x4 && y >= y1 && y <= y4;
    }

    double getDistance(float x1, float y1, float x2, float y2) {
        const auto dx = static_cast<double>(x1 - x2);
        const auto dy = static_cast<double>(y1 - y2);
        return sqrt(dx * dx + dy * dy);
    }

    std::vector<std::pair<float, float>> getStats(std::vector<geometry_msgs::msg::Point32> points) {
        auto sum = std::pair<float, float>(0, 0);
        auto min = std::pair<float, float>(FLT_MAX, FLT_MAX);
        auto max = std::pair<float, float>(FLT_MIN, FLT_MIN);
        for (auto it = points.begin(); it != points.end(); it++) {
            sum.first += it->x;
            sum.second += it->y;
            min.first = std::min(min.first, it->x);
            min.second = std::min(min.second, it->y);
            max.first = std::max(max.first, it->x);
            max.second = std::max(max.second, it->y);
        }
        return {std::make_pair(sum.first / static_cast<float>(points.size()), sum.second / static_cast<float>(points.size())),
            std::make_pair(min.first, min.second), std::make_pair(max.first, max.second)};
    }

    std::pair<float, float> getAvg(std::vector<std::pair<float, float>> points) {
        const auto sum = std::accumulate(points.begin(), points.end(), std::pair<float, float>(0, 0),
        [](std::pair<float, float> acc, const std::pair<float, float>& point) {
                    return std::make_pair(acc.first + point.first, acc.second + point.second);
                });
        return std::pair<float, float>((sum.first / static_cast<float>(points.size())), (sum.second/static_cast<float>(points.size())));
    }

    int checkPreviousClusters(std::vector<std::vector<std::pair<float, float>>> clusters, int j, float x, float y, float threshold) {
        float dynamicThreshold = threshold;
        int index = -1;
        while (j > 0) {
            auto centroid = getAvg(clusters[j]);
            double diff = getDistance(x, y, centroid.first, centroid.second);
            if (diff < dynamicThreshold) {
                index = j;
                dynamicThreshold = static_cast<float>(diff);
            }
            j -= 1;
        }
        return index;
    }

    // OPTION 2: CLUSTERING
    std::vector<std::vector<std::pair<float, float>>> clusteringPoints(std::vector<geometry_msgs::msg::Point32> points) {
        std::sort(points.begin(), points.end(), [](geometry_msgs::msg::Point32 a, geometry_msgs::msg::Point32 b)
                                    {
                                        if (a.x == b.x) return a.y < b.y;
                                        return a.x < b.x;
                                    });

        std::vector<std::vector<std::pair<float, float>>> result = {{std::make_pair(points[0].x, points[0].y)}};
        std::vector<std::pair<float, float>> stats = getStats(points);
        const float min_y = stats[1].second;
        const float min_x = points[0].x;
        float max_x = points[points.size()-1].x;
        float avgY = stats[0].second;
        float max_y = avgY;
        float distThresh = (stats[2].second - stats[1].second) / 4.0;
        // std::cout << "min_point: (" << min_x << ", " << min_y << ")" << std::endl;
        // std::cout << "max_point: (" << max_x << ", " << max_y << ")" << std::endl;

        int j = 1;
        for (int i = 1; i < static_cast<int>(points.size()); i++) {
            float x = points[i].x;
            float y = points[i].y;

            bool check = checkIfInsideJoint(min_x, min_y, max_x, max_y, x, y);
            if (check) {
                // result[0].push_back(points[i]);
                result[0].push_back(std::make_pair(points[i].x, points[i].y));
            } else {
                if (x > max_x && y < max_y) {
                    // result[0].push_back(points[i]);
                    result[0].push_back(std::make_pair(points[i].x, points[i].y));
                } else {
                    int cluster = checkPreviousClusters(result, j-1, x, y, distThresh);
                    if (cluster != -1) {
                        // result[cluster].push_back(points[i]);
                        result[cluster].push_back(std::make_pair(points[i].x, points[i].y));
                    } else {
                        // result.push_back({points[i]});
                        result.push_back({std::make_pair(points[i].x, points[i].y)});
                        j += 1;
                    }
                }
            }
        }
        // for (int i= 0; i < result.size(); i++) {
        //     auto centroid = getAvg(result[i]);
        //     std::cout <<"Cluster" << i+1 << ": (" << centroid.first << ", " << centroid.second << ")" << std::endl;
        //     printPoints(result[i]);
        // }
        return result;
    }

    std::pair<std::pair<float, float>, std::pair<float, float>> calculateBoundingRectangle(const std::vector<std::pair<float, float>>& cluster) {
        float min_x = cluster[0].first;
        float max_x = cluster[0].first;
        float min_y = cluster[0].second;
        float max_y = cluster[0].second;

        for (const auto& point : cluster) {
            if (point.first < min_x) min_x = point.first;
            if (point.first > max_x) max_x = point.first;
            if (point.second < min_y) min_y = point.second;
            if (point.second > max_y) max_y = point.second;
        }
        return {{min_x, min_y}, {max_x, max_y}};
    }

    // Function to calculate the centroid of a rectangle
    std::pair<float, float> calculateCentroid(const std::pair<std::pair<float, float>, std::pair<float, float>>& rect) {
        float centroid_x = (rect.first.first + rect.second.first) / 2.0;
        float centroid_y = (rect.first.second + rect.second.second) / 2.0;
        return {centroid_x, centroid_y};
    }

    std::pair<std::pair<float, float>, std::pair<float, float>> getBisector(const std::pair<std::pair<float, float>, std::pair<float, float>>& points) {
        std::pair<float, float> centre = calculateCentroid(points);
        float lengthX = points.second.first - points.first.first;
        float lengthY = points.second.second - points.first.second;
        if (lengthX > lengthY) {
            return std::pair(std::pair<float, float>(points.first.first, centre.second), std::pair<float, float>(points.second.first, centre.second));
        }
        return std::pair(std::pair<float, float>(centre.first, points.first.second), std::pair<float, float>(centre.first, points.second.second));
    }

    JointingPath merge(JointingPath p1, JointingPath p2) {
        JointingPath merged;
        if (p1.start.first == p1.end.first && p2.start.first == p2.end.first) {
            float avg_x = (p1.start.first + p2.start.first) / 2.0;
            float min_y = std::min(p1.start.second, p2.start.second);
            float max_y = std::max(p1.end.second, p2.end.second);
            merged.start = std::make_pair(avg_x, min_y);
            merged.end = std::make_pair(avg_x, max_y);
        } else {
            float avg_y = (p1.start.second + p2.start.second) / 2.0;
            float min_x = std::min(p1.start.first, p2.start.first);
            float max_x = std::max(p1.end.first, p2.end.first);
            merged.start = std::make_pair(min_x, avg_y);
            merged.end = std::make_pair(max_x, avg_y);
        }
        return merged;
    }


    // void homingCallback(geometry_msgs::msg::Point msg) {
    //     // In the hardware interface z is set based on whether homing was successful
    //     if (msg.z) {
    //         this->wallSpanDegX = msg.x;
    //         this->wallSpanDegY = msg.y;
    //     }
    // }

    // void trackingCallback(std_msgs::msg::Float64MultiArray msg) {
    //     this->update.updatePos(msg.data[0], msg.data[1]);
    // }

    void imgProcCallback(sensor_msgs::msg::PointCloud::UniquePtr msg) {
        auto clusters = clusteringPoints(msg->points);
        std::vector<JointingPath> paths;

        for (const auto& cluster : clusters) {
            auto rectangle = calculateBoundingRectangle(cluster);
            auto bisector = getBisector(rectangle);
            JointingPath new_path;
            new_path.start = bisector.first;
            new_path.end = bisector.second;

            this->paths.push_back(new_path);
        }

        for (const auto& cluster : clusters) {
            auto rectangle = calculateBoundingRectangle(cluster);
            auto bisector = getBisector(rectangle);
            JointingPath new_path;
            new_path.start = bisector.first;
            new_path.end = bisector.second;
            this->paths.push_back(new_path);
        }

        std::vector<std::pair<int, int>> pathsToMerge = {};
        for (int i = 0; i < static_cast<int>(this->paths.size()); i++) {
            for (int j = i + 1; j < static_cast<int>(this->paths.size()); j++) {
                double d1 = getDistance(paths[i].start.first, paths[i].start.second, paths[j].end.first, paths[j].end.second);
                double d2 = getDistance(paths[i].end.first, paths[i].end.second, paths[j].start.first, paths[j].start.second);
                if (d1 < 15 || d2 < 15) {
                    pathsToMerge.emplace_back(i, j);
                }
            }
        }

        for (auto &[fst, snd] : pathsToMerge) {
            JointingPath p1 = paths[fst];
            JointingPath p2 = paths[snd];
            paths.erase(paths.begin() + fst);
            paths.erase(paths.begin() + snd);
            JointingPath merged = merge(p1, p2);
            this->paths.push_back(merged);
        }

        // for (auto path : paths) {
        //     std::cout << "Bisector: start: (" << path.start.first << ", " << path.start.second << "), end: (" << path.end.first << ", " << path.end.second << ")" << std::endl;
        // }
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr homing_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr img_proc_subscription_;
    double wallSpanDegX = -1;
    double wallSpanDegY = -1;
    Update update;
    std::vector<JointingPath> paths = {};
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}

#include <memory>
#include <chrono>
#include <vector>
#include <cfloat>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "mason_navigation/srv/homing_sequence.hpp"

#define DEG_PASSED true
#define WALL_H_CM 91.75
#define WALL_W_CM 99.5

using namespace std::chrono_literals;

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

    std::pair<float, float> getPos() {
        return this->position;
    }
private:
    std::chrono::time_point<std::chrono::system_clock> t;
    std::pair<float, float> position;
};

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("nav_node") {
        this->update = Update(std::make_pair(0.0, 0.0));
        
        // auto homing_callback =
        //     [this](geometry_msgs::msg::Point::UniquePtr msg) -> void {
        //         // In the hardware interface z is set based on whether homing was successful
        //         if (msg->z) {
        //             this->wallSpanDegX = msg->x;
        //             this->wallSpanDegY = msg->y;
        //             this->multiplier_x = (DEG_PASSED) ? 1 : static_cast<float>(msg->x / WALL_W_CM);
        //             this->multiplier_y = (DEG_PASSED) ? 1 : static_cast<float>(msg->y / WALL_H_CM);
        //         }
        //     };

        auto tracking_callback =
            [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
                float y = (msg->name[0] == "MotorJointLeft") ? msg->position[0] : msg->position[1];
                float x = (msg->name[0] == "MotorTop") ? msg->position[0] : msg->position[1];
                this->update.updatePos(x, y);
            };

        auto img_proc_callback =
            [this](sensor_msgs::msg::PointCloud::UniquePtr msg) -> void {
                this->imgProcCallback(std::move(msg));
            };
        
        // this->homing_subscription_ = this->create_subscription<geometry_msgs::msg::Point>("limits", 10, homing_callback);
        this->position_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_state", 10, tracking_callback);
        this->position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("forward_position_controller/commands", 10);
        this->path_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("path", 10);
        this->img_proc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud>("detection", 10, img_proc_callback);
    }

    bool serviceIsAvailable(rclcpp::Client<mason_navigation::srv::HomingSequence>::SharedPtr client) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting...");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        return true;
    }
    
    int calcLength(const std::vector<int> &points) {
        return points[points.size()-1] - points[0];
    }

    bool checkIfInsideJoint(float x1, float y1, float x4, float y4, float x, float y) {
        return x >= x1 && x <= x4 && y >= y1 && y <= y4;
    }

    double getDistance(float x1, float y1, float x2, float y2) {
        const auto dx = static_cast<double>(x1 - x2);
        const auto dy = static_cast<double>(y1 - y2);
        return sqrt(dx * dx + dy * dy);
    }

    std::vector<std::pair<float, float>> getPairs(std::vector<geometry_msgs::msg::Point32> points) {
        std::vector<std::pair<float, float>> res;
        for (auto point : points) {
            res.emplace_back(point.x, point.y);
        }
        return res;
    }

    std::vector<std::pair<float, float>> getStats(std::vector<std::pair<float, float>> points) {
        auto sum = std::pair<float, float>(0, 0);
        auto min = std::pair<float, float>(FLT_MAX, FLT_MAX);
        auto max = std::pair<float, float>(FLT_MIN, FLT_MIN);
        for (const auto& point : points) {
            sum.first += point.first;
            sum.second += point.second;
            min.first = std::min(min.first, point.first);
            min.second = std::min(min.second, point.second);
            max.first = std::max(max.first, point.first);
            max.second = std::max(max.second, point.second);
        }
        return {
            {sum.first / points.size(), sum.second / points.size()},
            {min.first, min.second},
            {max.first, max.second}
        };
    }


    std::pair<float, float> getAvg(std::vector<std::pair<float, float>> points) {
        const auto sum = std::accumulate(points.begin(), points.end(), std::pair<float, float>(0, 0),
        [](std::pair<float, float> acc, const std::pair<float, float>& point) {
                    return std::make_pair(acc.first + point.first, acc.second + point.second);
                });
        return std::pair<float, float>((sum.first / static_cast<float>(points.size())), (sum.second/static_cast<float>(points.size())));
    }

    int checkPreviousClusters(const std::vector<std::vector<std::pair<float, float>>>& clusters, int j,
                            float x, float y, float distThreshX, float distThreshY) {
        int index = -1;
        float dynamicThreshold = FLT_MAX;
        while (j >= 0) {
            auto centroid = getAvg(clusters[j]);
            float dx = std::abs(x - centroid.first);
            float dy = std::abs(y - centroid.second);
            if ((dx < distThreshX && dy < distThreshY) && (dx + dy < dynamicThreshold)) {
                index = j;
                dynamicThreshold = dx + dy;
            }
            j--;
        }
        return index;
    }

    void printPoints(std::vector<std::pair<float, float>> points) {
        std::cout << "[(" << points[0].first << ", " << points[0].second << ")";
        for (auto it = points.begin()+1; it != points.end(); it++) {
            std::cout << ", " << std::endl << "(" << it->first << ", " << it->second << ")";
        }
        std::cout << "]," << std::endl;
    }

    std::vector<std::vector<std::pair<float, float>>> clusteringPoints(std::vector<geometry_msgs::msg::Point32> points) {
        std::sort(points.begin(), points.end(), [](geometry_msgs::msg::Point32 a, geometry_msgs::msg::Point32 b)
                                    {
                                        if (a.x == b.x) return a.y < b.y;
                                        return a.x < b.x;
                                    });

        std::vector<std::vector<std::pair<float, float>>> result = {{std::make_pair(points[0].x, points[0].y)}};
        auto stats = getStats(getPairs(points));
        const float distThreshX = (stats[2].first - stats[1].first) / 4.0;
        const float distThreshY = (stats[2].second - stats[1].second) / 4.0;

        int j = 1;
        for (size_t i = 1; i < points.size(); ++i) {
            float x = points[i].x;
            float y = points[i].y;

            int cluster = checkPreviousClusters(result, j - 1, x, y, distThreshX, distThreshY);
            if (cluster != -1) {
                result[cluster].emplace_back(points[i].x, points[i].y);
            } else {
                result.push_back({std::make_pair(points[i].x, points[i].y)});
                j++;
            }
        }

        // for (size_t i = 0; i < result.size(); ++i) {
        //     auto centroid = getAvg(result[i]);
        //     std::cout <<"Cluster" << i+1 << ": ";//"(" << centroid.first << ", " << centroid.second << ")" << std::endl;
        //     printPoints(result[i]);
        // }
        return result;
    }

    std::vector<std::pair<std::pair<float, float>, std::pair<float, float>>> clustersToPaths(std::vector<std::vector<std::pair<float, float>>> clusters) {
        std::vector<std::pair<std::pair<float, float>, std::pair<float, float>>> paths;

        for (const auto& cluster : clusters) {
            if (cluster.empty()) continue;

            auto stats = getStats(cluster);
            auto avg = stats[0];
            auto min = stats[1];
            auto max = stats[2];
        
            bool isHorizontal = (max.first - min.first) > (max.second - min.second);

            if (isHorizontal) {
                paths.emplace_back(std::make_pair(std::make_pair(min.first, avg.second),
                                                std::make_pair(max.first, avg.second)));
            } else {
                paths.emplace_back(std::make_pair(std::make_pair(avg.first, min.second),
                                                std::make_pair(avg.first, max.second)));
            }
        }

        return paths;
    }


    void imgProcCallback(sensor_msgs::msg::PointCloud::UniquePtr msg) {
        std::vector<std::vector<std::pair<float, float>>> clusters = clusteringPoints(msg->points);
        this->paths = clustersToPaths(clusters);
    }

    void jointingPathPublisher() {
        auto pathMsg = std_msgs::msg::Float64MultiArray();
        auto posMsg = std_msgs::msg::Float64MultiArray();
        for (auto path : this->paths) {
            float currentX = this->update.getPos().first;
            float currentY = this->update.getPos().second;

            
            if (abs(path.first.first - currentX) < 1e-1 && abs(path.first.second - currentY) < 1e-1) {
                pathMsg.data = {path.first.first*this->multiplier_x, path.first.second*this->multiplier_y, 
                    path.second.first*this->multiplier_x, path.second.second*this->multiplier_y};
            } else if (abs(path.second.first - currentX) < 1e-1 && abs(path.second.second - currentY) < 1e-1) {
                pathMsg.data = {path.second.first*this->multiplier_x, path.second.second*this->multiplier_y, 
                    path.first.first*this->multiplier_x, path.first.second*this->multiplier_y};
            } else {
                pathMsg.data = {path.first.first, path.first.second, path.second.first, path.second.second};
                posMsg.data = {path.first.first, path.first.second};
                this->position_publisher_->publish(posMsg);
                while (abs(path.first.first - currentX) > 1e-1 || abs(path.first.second - currentY) > 1e-1) {
                    currentX = this->update.getPos().first;
                    currentY = this->update.getPos().second;
                }
            }
            this->path_publisher_->publish(pathMsg);
            while (abs(pathMsg.data[2] - currentX) > 1e-1 || abs(pathMsg.data[3] - currentY) > 1e-1) {
                currentX = this->update.getPos().first;
                currentY = this->update.getPos().second;
            }
        }
    }

    float wallSpanDegX = -1;
    float wallSpanDegY = -1;
    float min_x = 0;
    float min_y = 0;
    float max_x = 0;
    float max_y = 0;

private:
    // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr homing_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr path_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr img_proc_subscription_;
    
    float multiplier_x = 1;
    float multiplier_y = 1;
    Update update;
    std::vector<std::pair<std::pair<float, float>, std::pair<float, float>>> paths = {};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto nav_node = std::make_shared<NavigationNode>();
    auto request = std::make_shared<mason_navigation::srv::HomingSequence::Request>();

    rclcpp::Client<mason_navigation::srv::HomingSequence>::SharedPtr homing_client = 
      nav_node->create_client<mason_navigation::srv::HomingSequence>("home_mason");
    rclcpp::Client<mason_navigation::srv::HomingSequence>::SharedPtr return_to_start_client = 
      nav_node->create_client<mason_navigation::srv::HomingSequence>("return_mason_to_start");

    bool ready = false;
    while(!ready) nav_node->serviceIsAvailable(return_to_start_client);

    auto result = return_to_start_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(nav_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(nav_node->get_logger(), "min_x: %f, min_y: %f", result.get()->x_limit, result.get()->y_limit);
        nav_node->min_x = result.get()->x_limit;
        nav_node->min_y = result.get()->y_limit;
    } else {
        RCLCPP_ERROR(nav_node->get_logger(), "Failed to call service return_mason_to_start");
    }

    ready = false;
    while(!ready) nav_node->serviceIsAvailable(homing_client);

    result = homing_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(nav_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(nav_node->get_logger(), "max_x: %f, max_y: %f", result.get()->x_limit, result.get()->y_limit);
        nav_node->max_x = result.get()->x_limit;
        nav_node->max_y = result.get()->y_limit;

    } else {
        RCLCPP_ERROR(nav_node->get_logger(), "Failed to call service home_mason");
    }

    rclcpp::spin(nav_node);
    rclcpp::shutdown();
    return 0;
}

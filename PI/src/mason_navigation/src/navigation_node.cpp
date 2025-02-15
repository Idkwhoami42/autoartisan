#include <memory>
#include <chrono>
#include <vector>
#include <cfloat>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "mason_test/srv/float.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/point32.hpp>
// #include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "mason_navigation/srv/homing_sequence.hpp"

#define DEG_PASSED true
#define WALL_H_CM 91.75
#define WALL_W_CM 99.5

int run_count = 0;

using namespace std::chrono_literals;

// if (msg_led->data == 0) {
//     activateBrush();
// } else if (msg_led->data == 1) {
//     deactivateBrush();
// } else if (msg_led->data == 2) {
//     activateVerticalSmoothing();
// } else if (msg_led->data == 3) {
//     activateHorizontalSmoothing();
// } else if (msg_led->data == 4) {
//     deactivateVerticalSmoothing();
// } else if (msg_led->data == 5) {
//     deactivateHorizontalSmoothing();
// } else if (msg_led->data == 6) {
//     goForward(1000);
// } else if (msg_led->data == 7) {
//     goBack(1000);
// } else if (msg_led->data == 8) {
//     activateExtruderMotor();
// } else if (msg_led->data == 9) {
//     deactivateExtruderMotor();
// }

struct Path {
    std::pair<float, float> start;
    std::pair<float, float> end;
    bool joint_filled;
    bool joint_brushed;
    int orientation[2]; // vertical = {2, 4}, horizontal = {3, 5}

    Path(std::pair<float, float> s, std::pair<float, float> e,
        bool filled, bool brushed, std::array<int, 2> orient)
        : start(s), end(e), joint_filled(filled), joint_brushed(brushed) {
            orientation[0] = orient[0];
            orientation[1] = orient[1];
    }
};

class Update {
public:
    Update() = default;

    Update(std::pair<float, float> currentPos) {
        this->t = std::chrono::steady_clock::now();
        this->position = currentPos;
    }

    void updatePos(float x, float y) {
        this->t = std::chrono::steady_clock::now();
        this->position = std::make_pair(x, y);
    }

    std::pair<float, float> getPos() {
        return this->position;
    }

    std::chrono::time_point<std::chrono::steady_clock> getTimestamp() {
        return this->t;
    }
private:
    std::chrono::time_point<std::chrono::steady_clock> t;
    std::pair<float, float> position;
};

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("nav_node") {
        this->update = Update(std::make_pair(0.0, 0.0));
        
        this->position_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odrive_position", 10,
            [this](const std_msgs::msg::Float32MultiArray::UniquePtr msg) -> void {
                this->update.updatePos(msg->data[0], msg->data[1]);
            });
        this->joint_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "detected_joints", 10,
            [this](const std_msgs::msg::Float32MultiArray::UniquePtr msg) -> void {
                this->detected_joints.push_back(Update(std::make_pair(msg->data[0], msg->data[1])));
                auto last_time = this->detected_joints.back().getTimestamp();
                auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(last_time.time_since_epoch()).count();
                if (diff >= 120 || run_count > 0) {
                    std::vector<std::vector<std::pair<float, float>>> clusters = clusteringPoints();
                    clustersToPaths(clusters);
                    jointPath();
                }
            });
        this->tool_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            "/mason_fsm_publisher/state", 10);

        this->contact_sensor_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/contact_sensor_triggered", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "%s CONTACT SENSOR TRIGGERED.", msg->data.c_str());
                this->contact_sensor_callback(msg->data);
            }
        );

        this->position_y_client_ = this->create_client<mason_test::srv::Float>("odrive_position_y");
        this->position_x_client_ = this->create_client<mason_test::srv::Float>("odrive_position_x");
        this->stop_y_client_ = this->create_client<std_srvs::srv::Empty>("odrive_stop_y");
        this->stop_x_client_ = this->create_client<std_srvs::srv::Empty>("odrive_stop_x");
    }

    void contact_sensor_callback(std::string sensor) {
        bool ready = false;
        while (!ready) {
            ready = positionServiceIsAvailable(this->position_x_client_);
            ready = ready && positionServiceIsAvailable(this->position_y_client_);
            ready = ready && stoppingServiceIsAvailable(this->stop_x_client_);
            ready = ready && stoppingServiceIsAvailable(this->stop_y_client_);
        }

        auto position_request = std::make_shared<mason_test::srv::Float::Request>();
        auto stop_request = std::make_shared<std_srvs::srv::Empty::Request>();

        switch (this->sensorMap[sensor]) {
            case 0:
                if (!std::isnan(this->min_x)) {
                    float diff = std::abs(this->update.getPos().first - this->min_x);
                    if (diff > 0.5f) {
                        float new_pos = this->update.getPos().first + 0.75f;
                        position_request->input_pos = new_pos;
                        this->position_x_client_->async_send_request(position_request);
                        // std::this_thread::sleep_for(std::chrono::seconds(1));
                        RCLCPP_ERROR(this->get_logger(),
                            "LIMB TRAPPED: LEFT CONTACT SENSOR TRIGGERED, BUT CURRENT_POS_X != MIN_X");
                    }
                    this->stop_x_client_->async_send_request(stop_request);
                }
                break;
            case 1:
                if (!std::isnan(this->max_x)) {
                    float diff = std::abs(this->update.getPos().first - this->max_x);
                    if (diff > 0.5f) {
                        float new_pos = this->update.getPos().first - 0.75f;
                        position_request->input_pos = new_pos;
                        this->position_x_client_->async_send_request(position_request);
                        // std::this_thread::sleep_for(std::chrono::seconds(1));
                        RCLCPP_ERROR(this->get_logger(),
                            "LIMB TRAPPED: RIGHT CONTACT SENSOR TRIGGERED, BUT CURRENT_POS_X != MAX_X");
                    }
                    this->stop_x_client_->async_send_request(stop_request);
                }
                break;
            case 2:
                if (!std::isnan(this->min_y)) {
                    float diff = std::abs(this->update.getPos().second - this->min_y);
                    if (diff > 0.5f) {
                        float new_pos = this->update.getPos().second + 0.75f;
                        position_request->input_pos = new_pos;
                        this->position_y_client_->async_send_request(position_request);
                        // std::this_thread::sleep_for(std::chrono::seconds(1));
                        RCLCPP_ERROR(this->get_logger(),
                            "LIMB TRAPPED: BOTTOM CONTACT SENSOR TRIGGERED, BUT CURRENT_POS_Y != MIN_Y");
                    }
                    this->stop_y_client_->async_send_request(stop_request);
                }
                break;
            case 3:
                if (!std::isnan(this->max_y)) {
                    float diff = std::abs(this->update.getPos().second - this->max_y);
                    if (diff > 0.5f) {
                        float new_pos = this->update.getPos().second - 0.75f;
                        position_request->input_pos = new_pos;
                        this->position_y_client_->async_send_request(position_request);
                        // std::this_thread::sleep_for(std::chrono::seconds(1));
                        RCLCPP_ERROR(this->get_logger(),
                            "LIMB TRAPPED: TOP CONTACT SENSOR TRIGGERED, BUT CURRENT_POS_Y != MAX_Y");
                    }
                    this->stop_y_client_->async_send_request(stop_request);
                }
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "UNIDENTIFIED CONTACT SENSOR TRIGGERED");
                break;
        }
    }

    bool positionServiceIsAvailable(rclcpp::Client<mason_test::srv::Float>::SharedPtr client) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for position service. Exiting...");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Position service not available, waiting again...");
        }
        return true;
    }

    bool stoppingServiceIsAvailable(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for stopping service. Exiting...");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Stopping service not available, waiting again...");
        }
        return true;
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
    
    void clustersToPaths(std::vector<std::vector<std::pair<float, float>>> clusters) {
        // std::vector<std::pair<std::pair<float, float>, std::pair<float, float>>> paths;

        for (const auto& cluster : clusters) {
            if (cluster.empty()) continue;

            auto stats = getStats(cluster);
            auto avg = stats[0];
            auto min = stats[1];
            auto max = stats[2];
        
            bool isHorizontal = (max.first - min.first) > (max.second - min.second);

            if (isHorizontal) {
                this->paths.emplace_back(Path(std::make_pair(min.first, avg.second),
                                                std::make_pair(max.first, avg.second),
                                                false, false, {3, 5}));
            } else {
                this->paths.emplace_back(Path(std::make_pair(avg.first, min.second),
                                                std::make_pair(avg.first, max.second),
                                                false, false, {2, 4}));
            }
        }
    }

    bool checkIfInsideJoint(float x1, float y1, float x4, float y4, float x, float y) {
        return x >= x1 && x <= x4 && y >= y1 && y <= y4;
    }

    double getDistance(float x1, float y1, float x2, float y2) {
        const auto dx = static_cast<double>(x1 - x2);
        const auto dy = static_cast<double>(y1 - y2);
        return std::hypot(dx, dy);
    }

    std::vector<std::pair<float, float>> getPairs() {
        std::vector<std::pair<float, float>> res;
        // res.reserve(this->detected_joints.size());
        for (auto point : this->detected_joints) {
            res.emplace_back(point.getPos());
        }
        return res;
    }

    std::vector<std::pair<float, float>> getStats(const std::vector<std::pair<float, float>>& points) {
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


    std::pair<float, float> getAvg(const std::vector<std::pair<float, float>>& points) {
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

    std::vector<std::vector<std::pair<float, float>>> clusteringPoints() {
        std::sort(this->detected_joints.begin(), this->detected_joints.end(), [](Update a, Update b)
                                    {
                                        auto a_cd = a.getPos();
                                        auto b_cd = b.getPos();
                                        auto ta = std::chrono::duration_cast<std::chrono::nanoseconds>(a.getTimestamp().time_since_epoch()).count();
                                        auto tb = std::chrono::duration_cast<std::chrono::nanoseconds>(b.getTimestamp().time_since_epoch()).count();
                                        if (ta == tb) {
                                            if (a_cd.first == b_cd.first) return a_cd.second < b_cd.second;
                                            return a_cd.first < b_cd.first;
                                        }
                                        return ta < tb;
                                    });

        std::vector<std::vector<std::pair<float, float>>> result = {{std::make_pair(this->detected_joints[0].getPos().first, this->detected_joints[0].getPos().second)}};
        auto stats = getStats(getPairs());
        const float distThreshX = (stats[2].first - stats[1].first) / 4.0;
        const float distThreshY = (stats[2].second - stats[1].second) / 4.0;

        int j = 1;
        for (size_t i = 1; i < this->detected_joints.size(); ++i) {
            float x = this->detected_joints[i].getPos().first;
            float y = this->detected_joints[i].getPos().second;

            int cluster = checkPreviousClusters(result, j - 1, x, y, distThreshX, distThreshY);
            if (cluster != -1) {
                // result[cluster].emplace_back(this->detected_joints[i].getPos().first, this->detected_joints[i].getPos().second);
                result[cluster].emplace_back(x, y);
            } else {
                // result.push_back({std::make_pair(this->detected_joints[i].getPos().first, this->detected_joints[i].getPos().second)});
                result.push_back({std::make_pair(x, y)});
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

    void moveToXY(float currentX, float currentY, float targetX, float targetY) {
        auto pos_request_x = std::make_shared<mason_test::srv::Float::Request>();
        auto pos_request_y = std::make_shared<mason_test::srv::Float::Request>();
        auto stop_request = std::make_shared<std_srvs::srv::Empty::Request>();

        bool ready = false;
        while (!ready) {
            ready = positionServiceIsAvailable(this->position_x_client_);
            ready = ready && positionServiceIsAvailable(this->position_y_client_);
            ready = ready && stoppingServiceIsAvailable(this->stop_x_client_);
            ready = ready && stoppingServiceIsAvailable(this->stop_y_client_);
        }

        pos_request_x->input_pos = targetX;
        pos_request_y->input_pos = targetY;

        this->position_x_client_->async_send_request(pos_request_x);
        this->position_y_client_->async_send_request(pos_request_y);
        
        while (std::abs(targetX - currentX) > 1e-1 || std::abs(targetY - currentY) > 1e-1) {
            currentX = this->update.getPos().first;
            currentY = this->update.getPos().second;
        }

        this->stop_x_client_->async_send_request(stop_request);
        this->stop_y_client_->async_send_request(stop_request);
    }

    void publish_tool_msg(int state) {
        auto tool_msg = std_msgs::msg::Int32();
        tool_msg.data = state;
        this->tool_publisher_->publish(tool_msg);
    }

    void jointPath() {
        for (auto p : this->paths) {
            std::vector<float> path = {p.start.first, p.start.second, p.end.first, p.end.second};
            /*if (abs(p.start.first - currentX) < 1e-1 && abs(p.start.second - currentY) < 1e-1) {
                path = {p.start.first, p.start.second, p.end.first, p.end.second};
            } else */
            if (abs(p.end.first - this->update.getPos().first) < 1e-1 
                && abs(p.end.second - this->update.getPos().second) < 1e-1) {
                path = {p.end.first, p.end.second, p.start.first, p.start.second};
            }
                
            moveToXY(this->update.getPos().first, this->update.getPos().second,
                path[0], path[1]);

            if (!p.joint_filled) {
                publish_tool_msg(p.orientation[0]);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                publish_tool_msg(8);
            } else {
                publish_tool_msg(0);
            }
            
            moveToXY(this->update.getPos().first, this->update.getPos().second,
                    path[2], path[3]);
            
            if (!p.joint_filled) {
                publish_tool_msg(9);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                publish_tool_msg(p.orientation[1]);
                p.joint_filled = true;
            } else {
                publish_tool_msg(1);
                p.joint_brushed = true;
            }
        }

        run_count += 1;
        const auto count = std::erase_if(this->paths, 
        [](const Path& p) {
            return p.joint_filled && p.joint_brushed;
        });

        RCLCPP_INFO(this->get_logger(), "%ld paths finished.", count);
    }

    float min_x = std::nanf(""), min_y = std::nanf(""), max_x = std::nanf(""), max_y = std::nanf("");

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tool_publisher_;
    rclcpp::Client<mason_test::srv::Float>::SharedPtr position_x_client_;
    rclcpp::Client<mason_test::srv::Float>::SharedPtr position_y_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_x_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_y_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr contact_sensor_sub_;
    
    float multiplier_x = 1;
    float multiplier_y = 1;
    std::unordered_map<std::string, int> sensorMap = {
        {"LEFT", 0},
        {"RIGHT", 1},
        {"BOTTOM", 2},
        {"TOP", 3}
    };
    Update update;
    std::vector<Path> paths = {};
    std::vector<Update> detected_joints = {};
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

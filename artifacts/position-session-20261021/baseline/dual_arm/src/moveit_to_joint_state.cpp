#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "dual_arm/msg/joint_command.hpp"
#include <vector>
#include <map>

class JointStateProcesser : public rclcpp::Node {
public:
    JointStateProcesser() : Node("joint_state_reader") {
        // 订阅者
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1000, std::bind(&JointStateProcesser::jointStateCallback, this, std::placeholders::_1));
        
        // 发布者
        pub_ = this->create_publisher<dual_arm::msg::JointCommand>("joint_commands", 1000);
        
        // 初始化关节ID映射
        joint_id_map_ = {
            {"r_joint_1", 0},
            {"r_joint_2", 1},
            {"r_joint_3", 2},
            {"r_joint_4", 3},
            {"r_joint_5", 4},
            {"r_joint_6", 5},
            {"r_joint_7", 6}
        };
        
        // 初始化位置数组
        joint_positions_.resize(7, 0.0); // 索引1-7对应电机ID1-7
        // 初始化默认控制参数
        default_kp_ = {250.0f, 120.0f, 120.0f, 120.0f, 15.0f, 15.0f, 15.0f};  // 默认kp
        default_kd_ = {30.0f, 16.0f, 16.0f, 16.0f, 2.0f, 2.0f, 2.0f};                // 默认kd
        default_speed_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};             // 默认速度 (rad/s)
        default_torque_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};     // 默认力矩 (Nm)
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if(msg->name.empty() || msg->position.empty()) {
            RCLCPP_WARN(this->get_logger(), "No date");
            return;
        }
        auto cmd_msg = std::make_unique<dual_arm::msg::JointCommand>();

        // 初始化所有数据
        for (int i = 0; i < 7; i++) {
            cmd_msg->position[i] = 0.0f;
            cmd_msg->kp[i] = default_kp_[i];
            cmd_msg->kd[i] = default_kd_[i];
            cmd_msg->speed[i] = default_speed_[i];
            cmd_msg->torque[i] = default_torque_[i];
        }

        // 更新位置数据
        for (size_t i = 0; i < msg->name.size(); i++) {
            auto it = joint_id_map_.find(msg->name[i]);
            if (it != joint_id_map_.end()) {
                int joint_idx = it->second;
                if (joint_idx >= 0 && joint_idx < 7) {
                    cmd_msg->position[joint_idx] = msg->position[i];
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Pub Command:");
        for (int i = 0; i < 7; i++) {
            RCLCPP_INFO(this->get_logger(), "MotorID%d: pos=%.3f, kp=%.1f, kd=%.1f, speed=%.2f, torque=%.2f",
                    i + 1, cmd_msg->position[i], cmd_msg->kp[i], cmd_msg->kd[i],
                    cmd_msg->speed[i], cmd_msg->torque[i]);
        }

        pub_->publish(*cmd_msg);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<dual_arm::msg::JointCommand>::SharedPtr pub_;
    std::vector<double> joint_positions_; 
    std::map<std::string, int> joint_id_map_;
    std::vector<double> default_kp_;
    std::vector<double> default_kd_;
    std::vector<double> default_speed_;
    std::vector<double> default_torque_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateProcesser>();
    RCLCPP_INFO(node->get_logger(), "begin");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
/**
 * @brief 躯干/头部电机控制 + joint_states 发布 (RViz 仿真)
 *
 * /body/cmd, /head/cmd → 透传 servo_node + 发布 joint_states
 * 消息格式: Float64MultiArray[2] = [joint_1_pos, joint_2_pos] (纯位置模式)
 *          (同时兼容旧版 [pos,spd,tor,kp,kd] x 2 格式)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class BodyHeadControlNode : public rclcpp::Node
{
public:
    BodyHeadControlNode() : Node("body_head_control_node")
    {
        // joint limits from URDF (defaults, overridable via YAML/launch)
        body_lo_[0] = declare_parameter("body_j1_lo", -1.17);  body_hi_[0] = declare_parameter("body_j1_hi", 1.17);
        body_lo_[1] = declare_parameter("body_j2_lo", -0.79);  body_hi_[1] = declare_parameter("body_j2_hi", 0.55);
        head_lo_[0] = declare_parameter("head_j1_lo", -0.791); head_hi_[0] = declare_parameter("head_j1_hi", 0.791);
        head_lo_[1] = declare_parameter("head_j2_lo", -0.791); head_hi_[1] = declare_parameter("head_j2_hi", 0.791);

        torso_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/body/motor_cmd", 10);
        head_cmd_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>("/head/motor_cmd", 10);
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // /body/cmd: Float64MultiArray[2] = [pos1, pos2]
        body_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/body/cmd", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                auto clamped = clamp_and_format_msg(msg, body_lo_, body_hi_);
                torso_cmd_pub_->publish(clamped);
                if (clamped.data.size() >= 1) body_target_[0] = clamped.data[0];
                if (clamped.data.size() >= 2) body_target_[1] = clamped.data[1];
            });
        // /head/cmd: Float64MultiArray[2] = [pos1, pos2]
        head_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/head/cmd", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                auto clamped = clamp_and_format_msg(msg, head_lo_, head_hi_);
                head_cmd_pub_->publish(clamped);
                if (clamped.data.size() >= 1) head_target_[0] = clamped.data[0];
                if (clamped.data.size() >= 2) head_target_[1] = clamped.data[1];
            });

        // 50Hz 定时发布 joint_states (目标位置，和手臂一样)
        joint_state_timer_ = create_wall_timer(
            std::chrono::milliseconds(20), [this]() { publish_joint_states(); });

        RCLCPP_INFO(get_logger(), "Body/head control node started");
    }

private:
    void publish_joint_states() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"body_joint_1", "body_joint_2", "head_joint_1", "head_joint_2"};
        msg.position = {body_target_[0], body_target_[1], head_target_[0], head_target_[1]};
        joint_state_pub_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr body_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torso_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    double body_target_[2] = {0.0, 0.0};
    double head_target_[2] = {0.0, 0.0};
    double body_lo_[2], body_hi_[2], head_lo_[2], head_hi_[2];

    static std_msgs::msg::Float64MultiArray clamp_and_format_msg(
        const std_msgs::msg::Float64MultiArray::SharedPtr in,
        const double lo[2], const double hi[2]) {
        std_msgs::msg::Float64MultiArray out;
        out.data.resize(2, 0.0);
        // 如果是 2 元素数组: [pos0, pos1]
        if (in->data.size() == 2) {
            out.data[0] = in->data[0];
            out.data[1] = in->data[1];
        }
        // 如果是 10 元素数组 (旧版 5*2): [pos0, spd, tor, kp, kd, pos1, ...]
        else if (in->data.size() >= 10) {
            out.data[0] = in->data[0];
            out.data[1] = in->data[5];
        }
        // 其它长度
        else {
            for (size_t i = 0; i < in->data.size() && i < 2; i++) {
                out.data[i] = in->data[i];
            }
        }

        // 限幅
        for (int i = 0; i < 2; i++) {
            if (out.data[i] < lo[i]) out.data[i] = lo[i];
            if (out.data[i] > hi[i]) out.data[i] = hi[i];
        }
        return out;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BodyHeadControlNode>());
    rclcpp::shutdown();
    return 0;
}

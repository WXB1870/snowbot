/**
 * @file force_sensor_node.cpp
 * @brief 夹爪力控节点 — 订阅 servo_node 发布的 EtherCAT 六维力数据
 *
 * 数据流: servo_node(EtherCAT PDO) -> force_sensor wrench topic
 *          -> 本节点力控算法 -> gripper cmd_sensor topic -> servo_node夹爪电机
 *
 * 力控算法: 摩擦锥夹紧力闭环 (FREE -> APPROACH -> FORCE_CTRL)
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <vector>
#include <cstring>
#include <string>
#include <functional>
#include <iostream>

class ForceSensorNode : public rclcpp::Node
{
public:
    ForceSensorNode()
    : Node("encos_force_sensor_node")
    {
        // 夹爪 joint limits from URDF (rad)
        grip_lo_ = declare_parameter("grip_l_lo", -1.484); grip_hi_ = declare_parameter("grip_l_hi", 0.524);
        grip_ro_ = declare_parameter("grip_r_lo", -0.524); grip_rh_ = declare_parameter("grip_r_hi", 1.484);

        declare_parameters();
        load_yaml_config();
        get_parameters();

        // 订阅 servo_node 发布的六维力数据
        left_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_sensor/left/wrench", 10,
            std::bind(&ForceSensorNode::left_wrench_callback, this, std::placeholders::_1));
        right_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_sensor/right/wrench", 10,
            std::bind(&ForceSensorNode::right_wrench_callback, this, std::placeholders::_1));

        // 安全夹爪指令发布（经力控处理后的电机指令）
        left_sensor_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>("/gripper/left_cmd_sensor", 10);
        right_sensor_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gripper/right_cmd_sensor", 10);

        // 订阅 VR 扳机原始指令
        left_vr_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gripper/left_cmd", 10,
            std::bind(&ForceSensorNode::left_vr_callback, this, std::placeholders::_1));
        right_vr_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gripper/right_cmd", 10,
            std::bind(&ForceSensorNode::right_vr_callback, this, std::placeholders::_1));

        // 去皮重服务
        left_tare_srv_  = create_service<std_srvs::srv::Trigger>(
            "/force_sensor/left/tare",
            std::bind(&ForceSensorNode::tare_callback, this,
                      std::placeholders::_1, std::placeholders::_2, 0));
        right_tare_srv_ = create_service<std_srvs::srv::Trigger>(
            "/force_sensor/right/tare",
            std::bind(&ForceSensorNode::tare_callback, this,
                      std::placeholders::_1, std::placeholders::_2, 1));

        // 200Hz 力控定时器
        ctrl_timer_ = create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&ForceSensorNode::control_loop, this));

        // 夹爪 joint_states (RViz仿真, 50Hz, 发布目标位置)
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        js_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ForceSensorNode::publish_joint_states, this));

        RCLCPP_INFO(get_logger(), "Force sensor node started (EtherCAT mode)");
    }

private:
    // ============ VR 指令回调 ============

    void left_vr_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 1) left_vr_pos_ = msg->data[0];
        if (msg->data.size() >= 2) left_vr_spd_  = msg->data[1];
        if (msg->data.size() >= 3) left_vr_tor_  = msg->data[2];
    }
    void right_vr_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 1) right_vr_pos_ = msg->data[0];
        if (msg->data.size() >= 2) right_vr_spd_  = msg->data[1];
        if (msg->data.size() >= 3) right_vr_tor_  = msg->data[2];
    }

    // ============ 传感器回调 — 存最新值 ============

    void left_wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        latest_left_[0] = msg->wrench.force.x;
        latest_left_[1] = msg->wrench.force.y;
        latest_left_[2] = msg->wrench.force.z;
        latest_left_[3] = msg->wrench.torque.x;
        latest_left_[4] = msg->wrench.torque.y;
        latest_left_[5] = msg->wrench.torque.z;
        have_left_ = true;
    }

    void right_wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        latest_right_[0] = msg->wrench.force.x;
        latest_right_[1] = msg->wrench.force.y;
        latest_right_[2] = msg->wrench.force.z;
        latest_right_[3] = msg->wrench.torque.x;
        latest_right_[4] = msg->wrench.torque.y;
        latest_right_[5] = msg->wrench.torque.z;
        have_right_ = true;
    }

    // ============ 200Hz 力控定时器 ============

    void control_loop() {
        if (taring_[0] || taring_[1]) {
            // 去皮重采集中：跳过力控
            for (int h = 0; h < 2; ++h) {
                if (taring_[h]) {
                    tare_buffers_[h].push_back(latest(h));
                    if (tare_buffers_[h].size() >= static_cast<size_t>(tare_samples_)) {
                        compute_tare_offset(h);
                        taring_[h] = false;
                    }
                }
            }
            return;
        }
        if (have_left_)  control_hand(0, latest_left_);
        if (have_right_) control_hand(1, latest_right_);
    }

    void control_hand(int hand_index, const std::array<double,6>& raw) {
        // 减去零点偏移
        std::array<double,6> val;
        for (int i = 0; i < 6; ++i) val[i] = raw[i] - tare_offsets_[hand_index][i];

        double& vr_pos   = (hand_index == 0) ? left_vr_pos_   : right_vr_pos_;
        double& filt_pos = (hand_index == 0) ? left_filt_pos_ : right_filt_pos_;
        bool&   startup  = (hand_index == 0) ? left_startup_   : right_startup_;
        int&    state    = (hand_index == 0) ? left_state_     : right_state_;
        double& slow_fx  = (hand_index == 0) ? left_slow_fx_  : right_slow_fx_;
        double& slow_fy  = (hand_index == 0) ? left_slow_fy_  : right_slow_fy_;
        double& slow_fz  = (hand_index == 0) ? left_slow_fz_  : right_slow_fz_;
        double& Fx_f     = (hand_index == 0) ? left_Fx_f_     : right_Fx_f_;
        double& Fy_f     = (hand_index == 0) ? left_Fy_f_     : right_Fy_f_;
        double& Fz_f     = (hand_index == 0) ? left_Fz_f_     : right_Fz_f_;
        double& lock_pos = (hand_index == 0) ? left_lock_pos_ : right_lock_pos_;
        int&    free_cnt = (hand_index == 0) ? left_free_cnt_ : right_free_cnt_;
        auto&   sensor_pub = (hand_index == 0) ? left_sensor_pub_ : right_sensor_pub_;

        // 低通滤波
        Fx_f += (val[0] - Fx_f) * force_filter_alpha_;
        Fy_f += (val[1] - Fy_f) * force_filter_alpha_;
        Fz_f += (val[2] - Fz_f) * force_filter_alpha_;

        // 方向约定: 右手正值=张开, 左手负值=张开 (L8 负值)
        // 闭合 = 朝 0 位运动: 右手 vr 减小, 左手 vr 增大
        bool closing = (hand_index == 0) ? (vr_pos > filt_pos) : (vr_pos < filt_pos);

        auto publish_cmd = [&](double pos, double tor) {
            // URDF joint limit clamp
            double lo = (hand_index == 0) ? grip_lo_ : grip_ro_;
            double hi = (hand_index == 0) ? grip_hi_ : grip_rh_;
            if (pos < lo) pos = lo;
            if (pos > hi) pos = hi;
            auto msg = std_msgs::msg::Float64MultiArray();
            // data[1] 是目标速度: 必须为 0, 否则驱动器阻抗 KD 项在静止时产生
            // 位置偏差 (KD*spd/KP), 之前发 1.047 导致夹爪偏开约 18 度
            msg.data = {pos, 0.0, tor};
            sensor_pub->publish(msg);
        };

        // 启动阶段：默认保持在零位 (不再张开到 1.047)
        if (startup) {
            filt_pos = 0.0;
            startup = false;
            state = GS_FREE;
            slow_fx += (val[0] - slow_fx) * slow_alpha_;
            slow_fy += (val[1] - slow_fy) * slow_alpha_;
            slow_fz += (val[2] - slow_fz) * slow_alpha_;
            publish_cmd(filt_pos, 0.0);
            return;
        }

        // 状态迁移
        if (state == GS_FORCE_CTRL) {
            // 释放判据: 扳机张开指令 — 右手 > 1.0, 左手 < -1.0
            bool release = (hand_index == 0) ? (vr_pos < -1.0) : (vr_pos > 1.0);
            if (release) { free_cnt++; if (free_cnt > 100) { state = GS_FREE; free_cnt = 0; } }
            else free_cnt = 0;
        } else if (!closing) {
            state = GS_FREE;
        } else if (state == GS_FREE) {
            state = GS_APPROACH;
        }

        if (state == GS_FREE) {
            slow_fx += (Fx_f - slow_fx) * slow_alpha_;
            slow_fy += (Fy_f - slow_fy) * slow_alpha_;
            slow_fz += (Fz_f - slow_fz) * slow_alpha_;

            double raw_step = (vr_pos - filt_pos) * 0.05;
            double max_step = 3.0 * 0.005;
            if (raw_step >  max_step) raw_step =  max_step;
            if (raw_step < -max_step) raw_step = -max_step;
            filt_pos += raw_step;
            if (filt_pos > vr_pos) filt_pos = vr_pos;
            publish_cmd(filt_pos, 0.0);
        }
        else if (state == GS_APPROACH) {
            if (std::abs(Fx_f - slow_fx) > contact_threshold_) {
                state = GS_FORCE_CTRL;
                lock_pos = filt_pos;
            }
            if (state == GS_APPROACH) {
                double raw_step = (vr_pos - filt_pos) * 0.05;
                double max_step = 3.0 * 0.005;
                if (raw_step >  max_step) raw_step =  max_step;
                if (raw_step < -max_step) raw_step = -max_step;
                filt_pos += raw_step;
                if (filt_pos < vr_pos) filt_pos = vr_pos;
                publish_cmd(filt_pos, 0.0);
            }
        }

        if (state == GS_FORCE_CTRL) {
            double Fn = std::abs(Fx_f - slow_fx);
            double dFy = Fy_f - slow_fy, dFz = Fz_f - slow_fz;
            double Ft = std::sqrt(dFy*dFy + dFz*dFz);

            double Fn_target = Ft / mu_;
            if (Fn_target < Fn_min_) Fn_target = Fn_min_;
            if (Fn_target > Fn_max_) Fn_target = Fn_max_;

            double error = Fn_target - Fn;
            if (std::abs(error) > force_deadband_) {
                double step = force_kp_ * error;
                if (step >  force_max_step_) step =  force_max_step_;
                if (step < -force_max_step_) step = -force_max_step_;
                lock_pos -= step;
            }
            filt_pos = lock_pos;
            publish_cmd(filt_pos, 0.0);
        }
    }

    // ============ 去皮重 ============

    void tare_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response,
        uint32_t hand_index)
    {
        tare_buffers_[hand_index].clear();
        taring_[hand_index] = true;
        response->success = true;
        response->message = (hand_index == 0 ? "Left" : "Right") +
            std::string(" tare started, sampling ") + std::to_string(tare_samples_) + " frames";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    void compute_tare_offset(uint32_t hand_index) {
        auto& buf = tare_buffers_[hand_index];
        if (buf.empty()) return;
        std::array<double,6> sum = {0};
        for (const auto& s : buf) for (int i = 0; i < 6; ++i) sum[i] += s[i];
        for (int i = 0; i < 6; ++i) tare_offsets_[hand_index][i] = sum[i] / buf.size();
        buf.clear();
        RCLCPP_INFO(get_logger(), "%s tare done", hand_index == 0 ? "Left" : "Right");
    }

    std::array<double,6> latest(uint32_t hand_index) {
        return hand_index == 0 ? latest_left_ : latest_right_;
    }

    // ============ 参数 ============

    void declare_parameters() {
        declare_parameter("mu", 0.3);
        declare_parameter("Fn_min", 5.0);
        declare_parameter("Fn_max", 10.0);
        declare_parameter("slip_increment", 2.0);
        declare_parameter("contact_threshold", 0.5);
        declare_parameter("slow_alpha", 0.005);
        declare_parameter("force_kp", 0.0005);
        declare_parameter("force_max_step", 0.0002);
        declare_parameter("force_deadband", 0.2);
        declare_parameter("force_filter_alpha", 0.1);
        declare_parameter("tare_samples", 10);
    }

    void load_yaml_config() {
        try {
            std::string yaml_path = ament_index_cpp::get_package_share_directory("encos_force_sensor")
                                  + "/config/force_sensor.yaml";
            YAML::Node config = YAML::LoadFile(yaml_path);
            if (config["encos_force_sensor_node"] && config["encos_force_sensor_node"]["ros__parameters"]) {
                auto p = config["encos_force_sensor_node"]["ros__parameters"];
                if (p["mu"])              set_parameter(rclcpp::Parameter("mu", p["mu"].as<double>()));
                if (p["Fn_min"])          set_parameter(rclcpp::Parameter("Fn_min", p["Fn_min"].as<double>()));
                if (p["Fn_max"])          set_parameter(rclcpp::Parameter("Fn_max", p["Fn_max"].as<double>()));
                if (p["slip_increment"])  set_parameter(rclcpp::Parameter("slip_increment", p["slip_increment"].as<double>()));
                if (p["contact_threshold"]) set_parameter(rclcpp::Parameter("contact_threshold", p["contact_threshold"].as<double>()));
                if (p["slow_alpha"])      set_parameter(rclcpp::Parameter("slow_alpha", p["slow_alpha"].as<double>()));
                if (p["force_kp"])        set_parameter(rclcpp::Parameter("force_kp", p["force_kp"].as<double>()));
                if (p["force_max_step"])  set_parameter(rclcpp::Parameter("force_max_step", p["force_max_step"].as<double>()));
                if (p["force_deadband"])  set_parameter(rclcpp::Parameter("force_deadband", p["force_deadband"].as<double>()));
                if (p["force_filter_alpha"]) set_parameter(rclcpp::Parameter("force_filter_alpha", p["force_filter_alpha"].as<double>()));
                RCLCPP_INFO(get_logger(), "Loaded force control params from YAML");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to load YAML, using defaults: %s", e.what());
        }
    }

    void get_parameters() {
        mu_                 = get_parameter("mu").as_double();
        Fn_min_             = get_parameter("Fn_min").as_double();
        Fn_max_             = get_parameter("Fn_max").as_double();
        slip_increment_     = get_parameter("slip_increment").as_double();
        contact_threshold_  = get_parameter("contact_threshold").as_double();
        slow_alpha_         = get_parameter("slow_alpha").as_double();
        force_kp_           = get_parameter("force_kp").as_double();
        force_max_step_     = get_parameter("force_max_step").as_double();
        force_deadband_     = get_parameter("force_deadband").as_double();
        force_filter_alpha_ = get_parameter("force_filter_alpha").as_double();
        tare_samples_       = get_parameter("tare_samples").as_int();
    }

    // ============ joint_states (RViz) ============

    void publish_joint_states() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"arm_l_joint_8", "arm_r_joint_8"};
        msg.position = {left_filt_pos_, right_filt_pos_};
        joint_state_pub_->publish(msg);
    }

    // ============ 成员变量 ============

    // 传感器数据接收
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_sub_;
    std::array<double,6> latest_left_  = {0};
    std::array<double,6> latest_right_ = {0};
    bool have_left_  = false;
    bool have_right_ = false;

    // 力控定时器
    rclcpp::TimerBase::SharedPtr ctrl_timer_;

    // joint_states
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr js_timer_;

    // 夹爪指令发布
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_sensor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_sensor_pub_;

    // VR 指令订阅
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_vr_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_vr_sub_;
    double left_vr_pos_ = 0.0, left_vr_spd_ = 0.0, left_vr_tor_ = 0.0;
    double right_vr_pos_ = 0.0, right_vr_spd_ = 0.0, right_vr_tor_ = 0.0;

    // URDF 夹爪限位
    double grip_lo_, grip_hi_, grip_ro_, grip_rh_;

    // 力控参数
    double mu_ = 0.3, Fn_min_ = 5.0, Fn_max_ = 10.0;
    double slip_increment_ = 2.0, contact_threshold_ = 0.5;
    double slow_alpha_ = 0.005, force_kp_ = 0.0005;
    double force_max_step_ = 0.0002, force_deadband_ = 0.2;
    double force_filter_alpha_ = 0.1;
    int tare_samples_ = 10;

    // 力控状态
    static constexpr int GS_FREE = 0, GS_APPROACH = 1, GS_FORCE_CTRL = 2;
    double left_filt_pos_ = 0.0, right_filt_pos_ = 0.0;
    bool left_startup_ = true, right_startup_ = true;
    int left_state_ = GS_FREE, right_state_ = GS_FREE;
    double left_slow_fx_ = 0.0, right_slow_fx_ = 0.0;
    double left_slow_fy_ = 0.0, right_slow_fy_ = 0.0;
    double left_slow_fz_ = 0.0, right_slow_fz_ = 0.0;
    double left_Fx_f_ = 0.0, right_Fx_f_ = 0.0;
    double left_Fy_f_ = 0.0, right_Fy_f_ = 0.0;
    double left_Fz_f_ = 0.0, right_Fz_f_ = 0.0;
    double left_lock_pos_ = 0.0, right_lock_pos_ = 0.0;
    int left_free_cnt_ = 0, right_free_cnt_ = 0;

    // 去皮重
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr left_tare_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr right_tare_srv_;
    bool taring_[2] = {false, false};
    std::array<double,6> tare_offsets_[2] = {{{0}}};
    std::vector<std::array<double,6>> tare_buffers_[2];
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceSensorNode>());
    rclcpp::shutdown();
    return 0;
}

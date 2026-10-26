#include "controller/chassis_control/chassis_control.h"
#include "can/chassis_service.h"
#include "can/socket_can.h"
#include <thread>

chassis_controller::chassis_controller(rclcpp::Node::SharedPtr node)
: node_(node)
{
    // 仅保留构造职责，init 中做 ROS 相关创建
}

void chassis_controller::init()
{
    if (!node_) {
        throw std::runtime_error("node is null in chassis_controller::init");
    }

    // 参数声明和获取
    node_->declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
    node_->declare_parameter<int>("baud_rate", 9600);
    std::string device;
    int baud_rate;
    node_->get_parameter("serial_device", device);
    node_->get_parameter("baud_rate", baud_rate);

    RCLCPP_INFO(node_->get_logger(), "Connecting to serial port %s at baud %d", device.c_str(), baud_rate);

    // 初始化串口
    serial_ = std::make_unique<SerialPort>(device);
    if (!serial_->openPort(baud_rate == 9600 ? B9600 : B115200)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open serial port!");
        throw std::runtime_error("Serial port opening failed");
    }

    // 初始化升降台控制
    elevator_ = std::make_unique<ElevatorController>(*serial_);

    // 订阅
    lifter_cmd_sub_ = node_->create_subscription<robot_control::msg::LifterCmd>(
        "/lifter/cmd", 10,
        std::bind(&chassis_controller::cmd_callback, this, std::placeholders::_1)
    );

    // 创建 SavePreset 服务
    save_preset_srv_ = node_->create_service<robot_control::srv::SavePreset>(
        "lifter/save_preset",
        [this](const std::shared_ptr<robot_control::srv::SavePreset::Request>,
               std::shared_ptr<robot_control::srv::SavePreset::Response> resp)
        {
            bool ok = elevator_->saveCurrentAsPreset();
            resp->success = ok;
            resp->message = ok ? "保存成功" : "保存失败，查看节点日志";
        }
    );

    // Action server: move_to_preset
    action_server_ = rclcpp_action::create_server<MoveToPreset>(
        node_,
        "lifter/move_to_preset",
        std::bind(&chassis_controller::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&chassis_controller::handle_cancel, this, std::placeholders::_1),
        std::bind(&chassis_controller::handle_accepted, this, std::placeholders::_1)
    );

    // 将原 BringUp 的逻辑直接放在这里：创建并启动底盘服务 + SocketCan
    m_chassis_service = std::make_shared<agx::nav::ChassisService>(node_.get());
    m_chassis_service->launch();

    auto socket_can = agx::nav::SocketCan::get_instance(node_.get());
    if (!socket_can || !socket_can->is_initialized()) {
        RCLCPP_ERROR(node_->get_logger(), "SocketCan 初始化失败，无法启动底盘服务");
    } else {
        m_socket_can = socket_can;
        m_socket_can->launch();
    }

    RCLCPP_INFO(node_->get_logger(), "Lifter control node initialized. Ready for /lifter/cmd, service /lifter/save_preset, action /lifter/move_to_preset");
}

void chassis_controller::cmd_callback(const robot_control::msg::LifterCmd::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(), "收到升降指令: move_cmd=%d", msg->move_cmd.data);

    // 如果当前正在执行 action，则只允许停止（move_cmd==0）以避免冲突
    if (busy_.load() && msg->move_cmd.data != 0) {
        RCLCPP_WARN(node_->get_logger(), "正在执行动作，忽略非停止的手动运动指令");
        return;
    }

    // 仅处理运动控制指令，预设保存/移动通过 service/action 触发
    if (msg->move_cmd.data == 1) {
        move_up();
    } else if (msg->move_cmd.data == -1) {
        move_down();
    } else if (msg->move_cmd.data == 0) {
        stop();
    } else {
        RCLCPP_WARN(node_->get_logger(), "未知 move_cmd: %d，已忽略。", msg->move_cmd.data);
    }
}

// Action callbacks
rclcpp_action::GoalResponse chassis_controller::handle_goal(const rclcpp_action::GoalUUID & /*uuid*/,
                                                            std::shared_ptr<const MoveToPreset::Goal> /*goal*/)
{
    if (busy_.load()) {
        RCLCPP_WARN(node_->get_logger(), "Rejecting goal: already busy");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse chassis_controller::handle_cancel(const std::shared_ptr<GoalHandleMove> /*goal_handle*/)
{
    RCLCPP_INFO(node_->get_logger(), "收到取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void chassis_controller::handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
{
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
        RCLCPP_WARN(node_->get_logger(), "接受目标时节点已忙");
        auto res = std::make_shared<MoveToPreset::Result>();
        res->success = false; res->message = "busy";
        goal_handle->abort(res);
        return;
    }
    std::thread{std::bind(&chassis_controller::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void chassis_controller::execute(const std::shared_ptr<GoalHandleMove> goal_handle)
{
    // RAII 清理器：退出 execute 时自动清 busy_
    struct BusyClearer {
        std::atomic<bool> &b;
        BusyClearer(std::atomic<bool> &bb): b(bb) {}
        ~BusyClearer() { b.store(false); }
    } busy_clear{busy_};

    RCLCPP_INFO(node_->get_logger(), "开始执行动作");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPreset::Result>();

    int target = goal->target_height;
    if (target == 0) {
        target = elevator_->getPresetHeight();
        if (target <= 0) {
            result->success = false; result->message = "无有效记忆点";
            goal_handle->abort(result);
            return;
        }
    }

    if (!elevator_->queryHeight(false)) {
        result->success = false; result->message = "无法读取当前高度";
        goal_handle->abort(result);
        return;
    }
    int start = elevator_->getCurrentHeight();
    if (std::abs(start - target) <= ERROR_MARGIN) {
        // 如果目标高度与当前高度的差值在容差范围内，则认为已到达目标位置
        result->success = true; result->message = "已在目标位置";
        goal_handle->succeed(result);
        return;
    }

    enum class Cmd { NONE, UP, DOWN, STOP };
    Cmd last_cmd = Cmd::NONE;

    auto feedback = std::make_shared<MoveToPreset::Feedback>();
    rclcpp::Rate rate(30); // 30 Hz

    // 用于节流日志与 progress 变化检测
    auto last_log = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    float last_prog = -1.0f;

    while (rclcpp::ok()) {
        // 立即处理取消：优先停机并返回
        if (goal_handle->is_canceling()) {
            if (last_cmd != Cmd::STOP) {
                stop();
                last_cmd = Cmd::STOP;
            }
            RCLCPP_INFO(node_->get_logger(), "动作已被取消");
            result->success = false; result->message = "已取消";
            goal_handle->canceled(result);
            return;
        }

        // 读取高度
        if (!elevator_->queryHeight(false)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "动作执行期间读取高度失败，正在重试...");
            rate.sleep();
            continue;
        }

        int current = elevator_->getCurrentHeight();
        feedback->current_height = current;

        // 计算 progress
        float prog = 0.0f;
        if (start != target) {
            float total = std::abs((float)target - (float)start);
            float done = std::abs((float)current - (float)start);
            prog = std::min(1.0f, done / total);
        }
        feedback->progress = prog;
        goal_handle->publish_feedback(feedback);

        // 节流打印：每1秒或 progress 改变超过 5% 时打印
        auto now = std::chrono::steady_clock::now();
        auto since = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log).count();
        if (since > 1000 || std::fabs(prog - last_prog) > 0.05f) {
            RCLCPP_INFO(node_->get_logger(), "动作进度: 当前=%d mm, 目标=%d mm, progress=%.2f",
                        current, target, prog);
            last_log = now;
            last_prog = prog;
        }

        // 判断到位（使用全局容差 STOP_OFFSET）
        if (std::abs(current - target) <= STOP_OFFSET) {
            if (last_cmd != Cmd::STOP) {
                stop();
                last_cmd = Cmd::STOP;
            }
            RCLCPP_INFO(node_->get_logger(), "已到达目标: %d mm", current);
            feedback->progress = 1.0f;
            goal_handle->publish_feedback(feedback);
            result->success = true; result->message = "到达目标";
            goal_handle->succeed(result);
            return;
        }

        // 决定移动方向，仅在方向变化时发送命令
        Cmd desired = (current < target) ? Cmd::UP : Cmd::DOWN;
        if (desired != last_cmd) {
            if (desired == Cmd::UP) {
                move_up();
                RCLCPP_DEBUG(node_->get_logger(), "发送 moveUp 命令");
            } else {
                move_down();
                RCLCPP_DEBUG(node_->get_logger(), "发送 moveDown 命令");
            }
            last_cmd = desired;
        }

        rate.sleep();
    }

    // 若循环意外退出
    if (last_cmd != Cmd::STOP) stop();
    result->success = false; result->message = "执行中断";
    goal_handle->abort(result);
}

void chassis_controller::move_up() {
    if (elevator_) elevator_->moveUp();
}
void chassis_controller::move_down() {
    if (elevator_) elevator_->moveDown();
}
void chassis_controller::stop() {
    if (elevator_) elevator_->stop();
}
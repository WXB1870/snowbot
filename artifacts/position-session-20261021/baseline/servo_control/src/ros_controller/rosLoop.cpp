#include "ros_controller/rosLoop.h"
#include "servo_control/ethercat_driver.h"
#include "define.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <cstddef>
#include <cstdlib>
#include <cmath>    // std::abs
#include <fstream>  // std::ifstream 用于 yaml 路径存在性检查
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {
// 启动回零时零位姿态的重力前馈初值（Nm）。
// 这是力矩偏置，不改变关节位置、速度或反馈符号定义。
constexpr float kLeftArmRetzeroGravityTorque[LARMDOF] = {
    -50.0f, 0.0f, 0.0f, -35.0f, 0.0f, 0.0f, 0.0f
};
constexpr float kRightArmRetzeroGravityTorque[RARMDOF] = {
     50.0f, 0.0f, 0.0f,  35.0f, 0.0f, 0.0f, 0.0f
};
constexpr float kRetzeroTrimLimit = 10.0f;
constexpr float kRetzeroTimeoutAcceptError = 0.01f; // timeout放行的最大残差(rad)
constexpr int kRetzeroTimeoutCycles = 10000; // 10 s @ 1 kHz
constexpr auto kShutdownTrajectoryDuration = std::chrono::seconds(3);
constexpr auto kShutdownSettleDelay = std::chrono::milliseconds(3200);
constexpr auto kShutdownSettleDuration = std::chrono::milliseconds(500);
constexpr auto kShutdownTimeout = std::chrono::seconds(8);

// systemd Type=notify uses this datagram socket protocol. Outside systemd the
// NOTIFY_SOCKET variable is absent, so manual ros2 runs remain unaffected.
void notify_systemd(const char* state) {
    const char* socket_path = std::getenv("NOTIFY_SOCKET");
    if (socket_path == nullptr || socket_path[0] == '\0') {
        return;
    }

    sockaddr_un address{};
    const size_t path_len = std::strlen(socket_path);
    if (path_len == 0 || path_len >= sizeof(address.sun_path)) {
        std::cerr << "[systemd] Invalid NOTIFY_SOCKET path" << std::endl;
        return;
    }

    address.sun_family = AF_UNIX;
    const bool abstract_socket = socket_path[0] == '@';
    if (abstract_socket) {
        address.sun_path[0] = '\0';
        std::memcpy(address.sun_path + 1, socket_path + 1, path_len - 1);
    } else {
        std::memcpy(address.sun_path, socket_path, path_len + 1);
    }

    const int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        return;
    }

    const socklen_t address_len = static_cast<socklen_t>(
        offsetof(sockaddr_un, sun_path) + path_len + (abstract_socket ? 0 : 1));
    (void)sendto(fd, state, std::strlen(state), MSG_NOSIGNAL,
                 reinterpret_cast<const sockaddr*>(&address), address_len);
    close(fd);
}
}

//全局变量 -- 数据传输
std::vector<double> joint_pos(SERVODOF, 0.0);
std::vector<double> joint_kp(SERVODOF, 0.0);
std::vector<double> joint_kd(SERVODOF, 0.0);
std::vector<double> joint_spd(SERVODOF, 0.0);
std::vector<double> joint_tor(SERVODOF, 0.0);

ServoNode::ServoNode() : Node ("servo_node"),
    has_received_command_(false),
    ifFirstTemp(true)
{
    //反馈值发送器初始化
    RetData_pub = this->create_publisher<snowbot_ctl_msgs::msg::RetInformation>(
        "dualArmCtl_fdback",
        200
    );

    //上位机接收器初始化
    CtlData = this->create_subscription<snowbot_ctl_msgs::msg::CtlInformation>(
        "ctl_data_pub",
        200,
        std::bind(&ServoNode::message_callback, this, std::placeholders::_1)
    );

    //六维力传感器发布 (从 EtherCAT 读取)
    left_wrench_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/force_sensor/left/wrench", 10);
    right_wrench_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/force_sensor/right/wrench", 10);

    //----------debug消息类型初始化-----------
    debugDataPub_comparison = this->create_publisher<snowbot_ctl_msgs::msg::DbgInformation>(
        "/debug_msgs/debug_of_comparison",
        10
    );

    //----------夹爪控制订阅----------
    // 主通道：经力传感器力控处理后的指令
    left_gripper_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/gripper/left_cmd_sensor",
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            last_left_sensor_time_ = this->now();
            gripper_cmd_apply(msg, true);
        }
    );
    right_gripper_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/gripper/right_cmd_sensor",
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            last_right_sensor_time_ = this->now();
            gripper_cmd_apply(msg, false);
        }
    );
    // 降级通道：sensor 超时 200ms 才生效
    left_gripper_direct_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/gripper/left_cmd",
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            if((this->now() - last_left_sensor_time_).seconds() > 0.2)
                gripper_cmd_apply(msg, true);
        }
    );
    right_gripper_direct_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/gripper/right_cmd",
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            if((this->now() - last_right_sensor_time_).seconds() > 0.2)
                gripper_cmd_apply(msg, false);
        }
    );

    // 躯干/头部电机控制订阅 (Float64MultiArray: [pos1,spd1,tor1,kp1,kd1, pos2,spd2,tor2,kp2,kd2])
    torso_cmd_sub = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/body/motor_cmd", 10,
        std::bind(&ServoNode::torso_cmd_callback, this, std::placeholders::_1));
    head_cmd_sub = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/head/motor_cmd", 10,
        std::bind(&ServoNode::head_cmd_callback, this, std::placeholders::_1));

    //----------初始化零位全局变量----------
    yaml_path_zeroPos = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") + "/config/zeropos.yaml";
    yaml_path_shutdownPos = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") + "/config/shutdownpos.yaml";
    yaml_path_initialPD = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") + "/config/initialPD.yaml";
    loadZeroPosFromYaml(yaml_path_zeroPos);
    loadShutdownPosFromYaml(yaml_path_shutdownPos);
    loadInitialPDFromYaml(yaml_path_initialPD, "high");     // 回零用高刚度抵抗重力

    // 关机位置配置缺失时退回零位
    if(lArm_shutdownPos.size() < LARMDOF){
        lArm_shutdownPos = lArm_zeroPos;
    }
    if(rArm_shutdownPos.size() < RARMDOF){
        rArm_shutdownPos = rArm_zeroPos;
    }

    //----------控制调试信息发送初始化----------
    performance_test = new performanceTestSeal(this);  // 传入当前节点指针

    //----------阻抗控制器初始化----------
    initImpedanceControllers();
}

//==================== 阻抗控制实现 ====================

// URDF 中关节顺序:
//   [0] body_joint_1  [1] body_joint_2  [2] head_joint_1  [3] head_joint_2
//   [4..10] arm_r_joint_1..7   [11] arm_r_joint_8 (夹爪)
//   [12..18] arm_l_joint_1..7   [19] arm_l_joint_8 (夹爪)
// EtherCAT 顺序: 0..6=左臂 7..13=右臂 14..15=夹爪 16..17=头 18..19=腰
// servo_retMsg_unpacked 只含 14 个臂关节反馈, 头/腰通过 getMotorFb_head/torso.

void ServoNode::initImpedanceControllers() {
    const std::string urdf_path =
        ament_index_cpp::get_package_share_directory("dual_arm") + "/urdf/snowbot.urdf";

    // 阻抗参数 yaml 配置文件 (与 PD/零位配置同位于 snowbot_ctl_msgs/config/).
    // 文件缺失或字段非法时, ImpedanceConfig::fromYaml 内部自动回退到默认值.
    //
    // 加载顺序: 1) 环境变量 IMPEDANCE_PARAMS_YAML 指定的绝对路径 (优先级最高,
    //              方便调试时直接指向 src 源文件, 修改 yaml 无需 colcon build)
    //           2) src 源文件路径 (开发期常用, 若存在则优先于 install)
    //           3) install 路径 (生产环境标准方式)
    //           4) 以上均不存在时 fromYaml 内部回退到代码默认值
    std::string yaml_path_impedance;
    // 常见 src 源文件路径 (相对于 workspace root)
    const std::vector<std::string> candidate_paths = {
        // 1) 环境变量覆盖 (绝对路径)
        std::getenv("IMPEDANCE_PARAMS_YAML") ? std::getenv("IMPEDANCE_PARAMS_YAML") : "",
        // 2) src 源文件路径 (开发期: 修改 yaml 不需 colcon build 即时生效)
        "/home/cc/snowbot_ws/src/ctl_msgs/config/impedance_params.yaml",
        // 3) install 路径 (生产环境)
        ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") +
            "/config/impedance_params.yaml",
    };
    for (const auto& path : candidate_paths) {
        if (path.empty()) continue;
        std::ifstream f(path);
        if (f.good()) {
            yaml_path_impedance = path;
            break;
        }
    }
    if (yaml_path_impedance.empty()) {
        yaml_path_impedance = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") +
                              "/config/impedance_params.yaml";
    }
    RCLCPP_INFO(this->get_logger(),
                "Loading impedance config: %s", yaml_path_impedance.c_str());

    auto make_config = [&](const std::string& side) {
        return snowbot_impedance::ImpedanceConfig::fromYaml(
            yaml_path_impedance, side, urdf_path);
    };

    impedance_left_  = std::make_shared<snowbot_impedance::CartesianImpedanceController>();
    impedance_right_ = std::make_shared<snowbot_impedance::CartesianImpedanceController>();

    auto node_shared = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    bool ok_l = impedance_left_->init(node_shared, make_config("left"));
    bool ok_r = impedance_right_->init(node_shared, make_config("right"));

    if (ok_l && ok_r) {
        impedance_inited_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Impedance controllers initialized (left+right), URDF=%s",
                    urdf_path.c_str());
    } else {
        impedance_inited_ = false;
        RCLCPP_ERROR(this->get_logger(),
                     "Impedance controller init failed (left=%d right=%d)",
                     ok_l, ok_r);
    }

    // 双臂协调层初始化 (可选, 默认 OFF).
    //   YAML 配置段完全可选, 缺失时 dual_arm_mode_ 保持 OFF, 零开销.
    {
        snowbot_impedance::DualArmConfig dcfg;
        try {
            YAML::Node root = YAML::LoadFile(yaml_path_impedance);
            if (root["dual_arm"]) {
                YAML::Node da = root["dual_arm"];
                // mode 仅通过 ROS 话题 ~/dual_arm/mode 运行时切换, 不从 YAML 读取
                if (da["leader_side"]) {
                    dcfg.leader_side = da["leader_side"].as<std::string>();
                }
                if (da["tool_constraint"]) {
                    YAML::Node tc = da["tool_constraint"];
                    Eigen::Affine3d T = Eigen::Affine3d::Identity();
                    if (tc["position"]) {
                        auto p = tc["position"].as<std::vector<double>>();
                        if (p.size() == 3) {
                            T.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
                        }
                    }
                    if (tc["orientation"]) {
                        auto o = tc["orientation"].as<std::vector<double>>();
                        if (o.size() == 3) {
                            T.linear() =
                                (Eigen::AngleAxisd(o[2], Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(o[1], Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(o[0], Eigen::Vector3d::UnitX()))
                                    .toRotationMatrix();
                        }
                    }
                    dcfg.T_tool_constraint = T;
                }

                if (da["max_grasp_deviation"]) {
                    dcfg.max_grasp_deviation =
                        da["max_grasp_deviation"].as<double>();
                }

                // === Step 5: Full 3D PD 内力控制 (可选, 默认零 → 关闭) ===
                if (da["internal_force"]) {
                    YAML::Node ifc = da["internal_force"];

                    auto parse_vec3 = [](YAML::Node n, Eigen::Vector3d& out) {
                        if (n) {
                            if (n.IsSequence() && n.size() == 3) {
                                out = Eigen::Vector3d(
                                    n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
                            } else if (n.IsScalar()) {
                                double v = n.as<double>();
                                out = Eigen::Vector3d::Constant(v);
                            }
                        }
                    };

                    parse_vec3(ifc["kp"], dcfg.internal_force_kp);
                    parse_vec3(ifc["kd"], dcfg.internal_force_kd);
                    if (ifc["max_wrench"]) dcfg.internal_force_max_wrench =
                        ifc["max_wrench"].as<double>();

                    const bool enabled =
                        !(dcfg.internal_force_kp.isZero(1e-9) && dcfg.internal_force_kd.isZero(1e-9));
                    RCLCPP_INFO(this->get_logger(),
                                "Internal force PD3D: %s kp=[%.2f,%.2f,%.2f] "
                                "kd=[%.2f,%.2f,%.2f] max_wrench=%.1f",
                                enabled ? "ENABLED" : "OFF (zero gains)",
                                dcfg.internal_force_kp.x(),
                                dcfg.internal_force_kp.y(),
                                dcfg.internal_force_kp.z(),
                                dcfg.internal_force_kd.x(),
                                dcfg.internal_force_kd.y(),
                                dcfg.internal_force_kd.z(),
                                dcfg.internal_force_max_wrench);
                }

                // === 阶段 2: 物体级阻抗配置 ===
                if (da["object_impedance"]) {
                    YAML::Node oi = da["object_impedance"];
                    if (oi["trp_geometry"]) {
                        YAML::Node tg = oi["trp_geometry"];
                        auto parse_tcp_to_trp = [&](YAML::Node n) -> Eigen::Affine3d {
                            Eigen::Affine3d T = Eigen::Affine3d::Identity();
                            if (n["position"]) {
                                auto p = n["position"].as<std::vector<double>>();
                                if (p.size() == 3)
                                    T.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
                            }
                            if (n["orientation"]) {
                                auto o = n["orientation"].as<std::vector<double>>();
                                if (o.size() == 3)
                                    T.linear() =
                                        (Eigen::AngleAxisd(o[2], Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(o[1], Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(o[0], Eigen::Vector3d::UnitX()))
                                            .toRotationMatrix();
                            }
                            return T;
                        };
                        if (tg["left_tcp_to_trp"])
                            dcfg.T_left_tcp_to_trp = parse_tcp_to_trp(tg["left_tcp_to_trp"]);
                        if (tg["right_tcp_to_trp"])
                            dcfg.T_right_tcp_to_trp = parse_tcp_to_trp(tg["right_tcp_to_trp"]);
                    }
                    if (oi["stiffness"]) {
                        auto v = oi["stiffness"].as<std::vector<double>>();
                        if (v.size() >= 6)
                            for (int i = 0; i < 6; ++i) dcfg.object_stiffness(i) = v[i];
                    }
                    if (oi["damping"]) {
                        auto v = oi["damping"].as<std::vector<double>>();
                        if (v.size() >= 6)
                            for (int i = 0; i < 6; ++i) dcfg.object_damping(i) = v[i];
                    }
                    RCLCPP_INFO(this->get_logger(),
                                "Object impedance: K=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f] "
                                "D=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
                                dcfg.object_stiffness(0), dcfg.object_stiffness(1),
                                dcfg.object_stiffness(2), dcfg.object_stiffness(3),
                                dcfg.object_stiffness(4), dcfg.object_stiffness(5),
                                dcfg.object_damping(0), dcfg.object_damping(1),
                                dcfg.object_damping(2), dcfg.object_damping(3),
                                dcfg.object_damping(4), dcfg.object_damping(5));
                }
                RCLCPP_INFO(this->get_logger(),
                            "Dual-arm coordinator: mode=%s leader=%s",
                            snowbot_impedance::dualArmModeToString(dcfg.mode).c_str(),
                            dcfg.leader_side.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Dual-arm coordinator: no [dual_arm] config, default OFF (independent)");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
                        "Dual-arm config parse failed, default OFF: %s", e.what());
        }
        dual_arm_coordinator_.configure(dcfg);
    }

    // 运行时模式切换订阅 (可选, 不配置时默认 OFF).
    dual_arm_mode_sub_ = create_subscription<std_msgs::msg::String>(
        "~/dual_arm/mode", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            auto prev = dual_arm_coordinator_.mode();
            auto m = snowbot_impedance::stringToDualArmMode(msg->data);
            dual_arm_coordinator_.setMode(m);

            // 离开 OBJECT_IMPEDANCE 时必须清除外部 wrench, 否则两臂会冻结在上一次
            // 注入的力上 (外部 wrench 在 update() 内部持续生效直到被清除).
            if (prev == snowbot_impedance::DualArmMode::OBJECT_IMPEDANCE &&
                m != snowbot_impedance::DualArmMode::OBJECT_IMPEDANCE) {
                if (impedance_left_)  impedance_left_->clearExternalWrench();
                if (impedance_right_) impedance_right_->clearExternalWrench();
                RCLCPP_INFO(this->get_logger(),
                            "Cleared external wrenches on OBJECT_IMPEDANCE exit");
            }

            RCLCPP_INFO(this->get_logger(), "Dual-arm mode -> %s",
                        snowbot_impedance::dualArmModeToString(m).c_str());
        });

    // 物体参考点 (TRP) 目标位姿订阅 (OBJECT_IMPEDANCE 模式使用).
    //   话题: ~/dual_arm/trp_reference, 类型 geometry_msgs/msg/PoseStamped.
    //   若未订阅 (或话题不存在), 进入 OBJECT_IMPEDANCE 时协调层会自动把当前
    //   TRP 位姿作为目标 (零跟踪误差, 两臂零输出 wrench).
    trp_reference_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/dual_arm/trp_reference", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.translation() = Eigen::Vector3d(
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            Eigen::Quaterniond q(
                msg->pose.orientation.w, msg->pose.orientation.x,
                msg->pose.orientation.y, msg->pose.orientation.z);
            q.normalize();
            T.linear() = q.toRotationMatrix();
            snowbot_impedance::Vector6d v = snowbot_impedance::Vector6d::Zero();
            dual_arm_coordinator_.setTrpReference(T, v);
        });

    // 默认不上电启用, 由上位机通过 ~/impedance_enable 切换.
    // 左/右参考位姿由控制器内部直接订阅:
    //   ~/impedance/arm_l_link_7/reference  (左臂)
    //   ~/impedance/arm_r_link_7/reference  (右臂)
    impedance_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "~/impedance_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (!impedance_inited_ && msg->data) {
                RCLCPP_WARN(this->get_logger(),
                            "Impedance not initialized, ignore enable");
                return;
            }
            const bool was_enabled = impedance_enable_;
            impedance_enable_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "Impedance enable -> %s",
                        impedance_enable_ ? "ON" : "OFF");
            if (impedance_enable_ && !was_enabled) {
                // 上升沿: 标记 just_enabled, 由 applyImpedanceControl 在 RT
                // 线程上下文捕获当前末端位姿/sensor bias (避免与 EtherCAT
                // 线程并发读 motor feedback). 同步使能双臂+双夹爪电机,
                // 让 CiA402 在下一个 PDO 周期内走完 enable 序列.
                impedance_just_enabled_.store(true, std::memory_order_release);
                bool arm_mask[TOTAL_SLAVES] = {};
                for (int i = 0; i < LARMDOF; i++) arm_mask[i] = true;        // 左臂 J1~J7
                arm_mask[7] = true;                                            // 左夹爪
                // arm_mask[5] = true;
                for (int i = 0; i < RARMDOF; i++) arm_mask[8 + i] = true;    // 右臂 J1~J7
                arm_mask[15] = true;                                           // 右夹爪
                // arm_mask[14] = true;
                // arm_mask[13] = true;
                // arm_mask[12] = true;
                // arm_mask[10] = true;
                EtherCAT_setMotorEnable(true);
                EtherCAT_setPerMotorEnable(arm_mask);
                RCLCPP_INFO(this->get_logger(),
                            "Impedance enable: motors enabled (arms+grippers)");
            } else if (!impedance_enable_ && was_enabled) {
                // 下降沿: 立即清 bias 和参考, 并失能双臂+双夹爪电机,
                // 让机械臂回到回零后的锁死状态 (与原有失能逻辑一致)
                if (impedance_left_)  impedance_left_->disable();
                if (impedance_right_) impedance_right_->disable();
                bool all_off_mask[TOTAL_SLAVES] = {};
                EtherCAT_setPerMotorEnable(all_off_mask);
                EtherCAT_setMotorEnable(false);
                RCLCPP_INFO(this->get_logger(),
                            "Impedance disable: motors disabled (arms+grippers)");
            }
        });

    // 前馈力矩测试订阅: 通过话题直接给定关节力矩, 绕过阻抗计算
    //   用途: 标定驱动器力矩响应, 测试给定 tau_ff 时关节是否真实输出对应力矩
    //   数据格式: Float64MultiArray, 7 维 [J1..J7] 单位 [Nm]
    //   全零 = 恢复阻抗控制; 非零 = 覆盖阻抗输出
    tau_ff_left_  = std::vector<double>(LARMDOF, 0.0);
    tau_ff_right_ = std::vector<double>(RARMDOF, 0.0);
    tau_ff_left_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/tau_ff_left", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            tau_ff_left_callback(msg);
        });
    tau_ff_right_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/tau_ff_right", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            tau_ff_right_callback(msg);
        });
    RCLCPP_INFO(this->get_logger(),
                "Tau-FF test topics ready: ~/tau_ff_left, ~/tau_ff_right (7-DoF [Nm])");
}

void ServoNode::tau_ff_left_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < static_cast<size_t>(LARMDOF)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tau_ff_left: expected %d elements, got %zu",
                             LARMDOF, msg->data.size());
        return;
    }
    std::lock_guard<std::mutex> lk(tau_ff_mutex_);
    for (int i = 0; i < LARMDOF; ++i) tau_ff_left_[i] = msg->data[i];
    // 左臂任一关节非零则激活左臂前馈模式
    bool any_nonzero = false;
    for (int i = 0; i < LARMDOF; ++i) {
        if (std::abs(tau_ff_left_[i]) > 1e-6) { any_nonzero = true; break; }
    }
    tau_ff_left_active_ = any_nonzero;
}

void ServoNode::tau_ff_right_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < static_cast<size_t>(RARMDOF)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tau_ff_right: expected %d elements, got %zu",
                             RARMDOF, msg->data.size());
        return;
    }
    RCLCPP_INFO(this->get_logger(),
                "tau_ff_right received: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                msg->data[0], msg->data[1], msg->data[2],
                msg->data[3], msg->data[4], msg->data[5], msg->data[6]);
    std::lock_guard<std::mutex> lk(tau_ff_mutex_);
    for (int i = 0; i < RARMDOF; ++i) tau_ff_right_[i] = msg->data[i];
    // 右臂任一关节非零则激活右臂前馈模式
    bool any_nonzero = false;
    for (int i = 0; i < RARMDOF; ++i) {
        if (std::abs(tau_ff_right_[i]) > 1e-6) { any_nonzero = true; break; }
    }
    tau_ff_right_active_ = any_nonzero;
}

void ServoNode::applyImpedanceControl() {
    // 0. 提前获取六维力传感器数据并滤波
    //    无论阻抗是否使能都持续更新, 使能时无阶跃
    float fx, fy, fz, mx, my, mz;
    const auto& offset_l = impedance_left_->getSensorOffset();
    const auto& offset_r = impedance_right_->getSensorOffset();
    getSensorData(0, fx, fy, fz, mx, my, mz);   // 左
    Eigen::Matrix<double, 6, 1> wrench_l_raw;
    wrench_l_raw << fx - offset_l(0), fy - offset_l(1), fz - offset_l(2),
                    mx - offset_l(3), my - offset_l(4), mz - offset_l(5);
    getSensorData(1, fx, fy, fz, mx, my, mz);   // 右
    Eigen::Matrix<double, 6, 1> wrench_r_raw;
    wrench_r_raw << fx - offset_r(0), fy - offset_r(1), fz - offset_r(2),
                    mx - offset_r(3), my - offset_r(4), mz - offset_r(5);
    // 一阶低通滤波
    if (!sensor_filter_inited_) {
        wrench_l_filtered_ = wrench_l_raw;
        wrench_r_filtered_ = wrench_r_raw;
        sensor_filter_inited_ = true;
    } else {
        wrench_l_filtered_ = sensor_lpf_alpha_ * wrench_l_raw
                           + (1.0 - sensor_lpf_alpha_) * wrench_l_filtered_;
        wrench_r_filtered_ = sensor_lpf_alpha_ * wrench_r_raw
                           + (1.0 - sensor_lpf_alpha_) * wrench_r_filtered_;
    }
    Eigen::Matrix<double, 6, 1> wrench_l = wrench_l_filtered_;
    Eigen::Matrix<double, 6, 1> wrench_r = wrench_r_filtered_;

    if (!impedance_inited_ || shutdown_mode_) return;
    if (sigInterrupt.load()) return;  // Ctrl+C 期间禁用阻抗

    // 1. 构造 q_full / dq_full 和 efforts (置 0, 因 has_effort_states=false)
    //    维度 = pinocchio model.nq (20: 躯干2 + 头2 + 右臂8(含夹爪) + 左臂8(含夹爪))
    //    关键: 必须用 pinocchio 的实际 idx_qs 填充, 而不是硬编码 URDF 文档顺序.
    //    pinocchio buildModel 走运动学树遍历 (具体顺序取决于 URDF 解析实现,
    //    可能与 URDF 文档顺序不同), 因此通过 getControlledQIdx/getJointQIdx 动态查询.
    //    注: 此构造在 impedance_enable_ 检查之前执行, 用于:
    //      (a) EtherCAT 使能序列期间实时重力前馈 (未使能阻抗时);
    //      (b) 阻抗使能上升沿采样重力;
    //      (c) 阻抗 update().
    const size_t nq = impedance_left_->getNqFull();
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(nq);
    Eigen::VectorXd dq_full = Eigen::VectorXd::Zero(nq);
    Eigen::VectorXd efforts_full = Eigen::VectorXd::Zero(nq);

    // 躯干/头部: 通过关节名查询 pinocchio 索引 (左右臂控制器共享同一 model,
    // 用任一查询均可, 这里用 left)
    float p, sp, c; uint16_t st;
    auto fill_aux_joint = [&](const std::string& jname, float val_p, float val_sp) {
        int qi = impedance_left_->getJointQIdx(jname);
        int vi = impedance_left_->getJointVIdx(jname);
        if (qi >= 0 && static_cast<size_t>(qi) < nq) { q_full(qi) = val_p;  dq_full(qi) = val_sp; }
        (void)vi;
    };
    getMotorFb_torso(0, p, sp, c, st); fill_aux_joint("body_joint_1", p, sp);
    getMotorFb_torso(1, p, sp, c, st); fill_aux_joint("body_joint_2", p, sp);
    getMotorFb_head(0, p, sp, c, st);   fill_aux_joint("head_joint_1", p, sp);
    getMotorFb_head(1, p, sp, c, st);   fill_aux_joint("head_joint_2", p, sp);

    // 右臂 (EtherCAT servo_retMsg_unpacked[7..13]) -> pinocchio 索引由
    // impedance_right_->getControlledQIdx() 给出
    {
        const auto& qidx_r = impedance_right_->getControlledQIdx();
        const auto& vidx_r = impedance_right_->getControlledVIdx();
        const int n = static_cast<int>(qidx_r.size());
        for (int i = 0; i < n && i < RARMDOF; ++i) {
            if (qidx_r[i] >= 0 && static_cast<size_t>(qidx_r[i]) < nq) {
                q_full(qidx_r[i])  = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
                dq_full(vidx_r[i]) = servo_retMsg_unpacked[i + LARMDOF].joint_spdRet;
            }
        }
        // 右夹爪虚拟关节 (arm_r_joint_8) -> 0
        int qg = impedance_left_->getJointQIdx("arm_r_joint_8");
        if (qg >= 0 && static_cast<size_t>(qg) < nq) { q_full(qg) = 0.0; dq_full(qg) = 0.0; }
    }
    // 左臂 (EtherCAT servo_retMsg_unpacked[0..6]) -> pinocchio 索引由
    // impedance_left_->getControlledQIdx() 给出
    {
        const auto& qidx_l = impedance_left_->getControlledQIdx();
        const auto& vidx_l = impedance_left_->getControlledVIdx();
        const int n = static_cast<int>(qidx_l.size());
        for (int i = 0; i < n && i < LARMDOF; ++i) {
            if (qidx_l[i] >= 0 && static_cast<size_t>(qidx_l[i]) < nq) {
                q_full(qidx_l[i])  = servo_retMsg_unpacked[i].joint_posRet;
                dq_full(vidx_l[i]) = servo_retMsg_unpacked[i].joint_spdRet;
            }
        }
        // 左夹爪虚拟关节 (arm_l_joint_8) -> 0
        int qg = impedance_left_->getJointQIdx("arm_l_joint_8");
        if (qg >= 0 && static_cast<size_t>(qg) < nq) { q_full(qg) = 0.0; dq_full(qg) = 0.0; }
    }

    // 1.5. 实时重力补偿计算 (阻抗使能与否均执行)
    //      用于 EtherCAT CiA402 使能序列期间预注入重力前馈, 防止抱闸打开后下掉.
    //      原方案 E 仅在使能上升沿采样一次, 若使能序列耗时较长, 期间重力_ff 为零.
    //      现改为每帧实时更新, 使能序列任何阶段都能获得重力补偿.
    //      注: 阻抗未使能时也需要此值, 因此在 impedance_enable_ 检查之前执行.
    {
        const auto& g_l = impedance_left_->computeGravityLive(q_full, dq_full);
        const auto& g_r = impedance_right_->computeGravityLive(q_full, dq_full);
        for (int i = 0; i < LARMDOF; ++i) {
            EtherCAT_setGravityFF(i, static_cast<float>(g_l(i)));
            EtherCAT_setGravityFF(8 + i, static_cast<float>(g_r(i)));
        }
    }

    // 2. (已移至函数最前面) 六维力传感器数据获取+滤波
    //    在 impedance_enable_ 检查之前处理, 使能时无阶跃

    // 阻抗未使能则到此为止 (重力补偿仍持续注入 EtherCAT)
    if (!impedance_enable_) return;

    // 3. 上升沿处理: 使能后第一帧, 在 RT 线程上下文捕获当前末端位姿和
    //    sensor bias (避免与 EtherCAT 线程并发读 motor feedback 撕裂数据)
    if (impedance_just_enabled_.exchange(false, std::memory_order_acq_rel)) {
        impedance_left_->enable(q_full, dq_full, wrench_l);
        impedance_right_->enable(q_full, dq_full, wrench_r);
        // 方案 E 已废弃: 不再在上升沿一次性采样重力, 改为上面每帧实时更新.
    }

    // 4. 双臂协调 PRE-update (阶段 2: OBJECT_IMPEDANCE 在 update 之前注入外部 wrench)
    dual_arm_coordinator_.applyPreUpdate(impedance_left_.get(), impedance_right_.get(), get_logger());

    // 4.5 阻抗计算
    const double dt = TEMPTIME;
    impedance_left_->update(q_full, dq_full, efforts_full, wrench_l, dt);
    impedance_right_->update(q_full, dq_full, efforts_full, wrench_r, dt);

    // 4.6 双臂协调 POST-update (阶段 1 FOLLOWER + 阶段 2 reference 维持)
    dual_arm_coordinator_.applyPostUpdate(impedance_left_.get(), impedance_right_.get(), get_logger());

    // 5. 力矩注入到 servo_desMsg_orig
    //    力位混合模式: 位置环跟踪零空间期望构型 (q_d_nullspace_filtered_),
    //    让位置环维持期望构型, 而不是把当前反馈当作目标 (避免位置漂移累积).
    //    力矩为阻抗输出, Kp/Kd 设为低值, 让力矩主导同时保留少量位置阻尼.
    //    注: PT 模式下 KP=0 会导致驱动器不输出任何力矩 (含 target_torque),
    //        因此设为小 Kp 让位置环吸收残余误差 + target_torque 主导.
    constexpr float kImpedanceKp = 1.0f;
    constexpr float kImpedanceKd = 0.2f;
    const auto& tau_l = impedance_left_->getEffortCommands();
    const auto& tau_r = impedance_right_->getEffortCommands();
    const auto& qd_l = impedance_left_->getQDNullspaceFiltered();
    const auto& qd_r = impedance_right_->getQDNullspaceFiltered();

    // 前馈力矩测试模式: 话题下发的 tau_ff 直接覆盖阻抗输出
    //   左右臂独立使能: 哪个臂收到非零前馈, 哪个臂绕过阻抗
    bool ff_l_active = false, ff_r_active = false;
    std::vector<double> ff_l(LARMDOF, 0.0), ff_r(RARMDOF, 0.0);
    {
        std::lock_guard<std::mutex> lk(tau_ff_mutex_);
        ff_l_active = tau_ff_left_active_;
        ff_r_active = tau_ff_right_active_;
        ff_l = tau_ff_left_;
        ff_r = tau_ff_right_;
    }
    // 诊断: 每秒打印一次前馈力矩状态
    // {
    //     static int64_t last_ff_diag_ns = 0;
    //     const int64_t now_ns = this->now().nanoseconds();
    //     if (last_ff_diag_ns == 0 || (now_ns - last_ff_diag_ns) > 1'000'000'000) {
    //         RCLCPP_INFO(this->get_logger(),
    //                     "tau_ff: L_active=%d R_active=%d ff_r=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] tau_r=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
    //                     ff_l_active ? 1 : 0, ff_r_active ? 1 : 0,
    //                     ff_r[0], ff_r[1], ff_r[2], ff_r[3], ff_r[4], ff_r[5], ff_r[6],
    //                     tau_r(0), tau_r(1), tau_r(2), tau_r(3), tau_r(4), tau_r(5), tau_r(6));
    //         last_ff_diag_ns = now_ns;
    //     }
    // }

    // 构建实际下发力矩向量, 传给阻抗控制器用于 effort_msg_ 发布
    Eigen::VectorXd tau_applied_l(LARMDOF), tau_applied_r(RARMDOF);

    for (int i = 0; i < LARMDOF; ++i) {
        const float tor = ff_l_active ? static_cast<float>(ff_l[i])
                                      : static_cast<float>(tau_l(i));
        servo_desMsg_orig[i].transMod  = 1;
        servo_desMsg_orig[i].joint_pos = static_cast<float>(qd_l(i));
        servo_desMsg_orig[i].joint_spd = 0.0f;
        servo_desMsg_orig[i].joint_tor = tor;
        servo_desMsg_orig[i].joint_Kp  = kImpedanceKp;
        servo_desMsg_orig[i].joint_Kd  = kImpedanceKd;
        tau_applied_l(i) = tor;
    }
    for (int i = 0; i < RARMDOF; ++i) {
        const float tor = ff_r_active ? static_cast<float>(ff_r[i])
                                      : static_cast<float>(tau_r(i));
        servo_desMsg_orig[i + LARMDOF].transMod  = 1;
        servo_desMsg_orig[i + LARMDOF].joint_pos = static_cast<float>(qd_r(i));
        servo_desMsg_orig[i + LARMDOF].joint_spd = 0.0f;
        servo_desMsg_orig[i + LARMDOF].joint_tor = tor;
        servo_desMsg_orig[i + LARMDOF].joint_Kp  = kImpedanceKp;
        servo_desMsg_orig[i + LARMDOF].joint_Kd  = kImpedanceKd;
        tau_applied_r(i) = tor;
    }

    // 将实际下发力矩传给阻抗控制器, 通过 ~/effort_cmds 发布
    impedance_left_->setTorqueFF(tau_applied_l);
    impedance_right_->setTorqueFF(tau_applied_r);
}

void ServoNode::rosLoopVar_Init(){

    //-----传输变量初始化-----

    servo_jnt_topology = get_jnt_topology();
    jnt_kt_unpacked = joint_Kt_get();
    servo_desMsg.resize(servo_jnt_topology.board_count);
    servo_retMsg.resize(servo_jnt_topology.board_count);
    for(int i = 0; i < servo_jnt_topology.board_count; i++){
        joint_desMsg temp_desMsg;
        joint_retMsg temp_retMsg;

        temp_desMsg.transMod = 1;
        temp_desMsg.joint_pos = 0.0;
        temp_desMsg.joint_spd = 0.0;
        temp_desMsg.joint_tor = 0.0;
        temp_desMsg.joint_Kp = 0.0;
        temp_desMsg.joint_Kd = 0.0;

        temp_retMsg.joint_posRet = 0.0;
        temp_retMsg.joint_spdRet = 0.0;
        temp_retMsg.joint_currentRet = 0.0;
        temp_retMsg.joint_temperature = 0.0;

        servo_desMsg[i].assign(servo_jnt_topology.joints_per_board[i], temp_desMsg);
        servo_retMsg[i].assign(servo_jnt_topology.joints_per_board[i], temp_retMsg);
    }

    int m_vector_len = 0;
    for(int i = 0; i < servo_jnt_topology.board_count; i++){
        m_vector_len += servo_jnt_topology.joints_per_board[i];
    }
    this->vector_len = m_vector_len;
    servo_desMsg_orig.resize(this->vector_len);
    servo_retMsg_unpacked.resize(this->vector_len);
    for(int i = 0; i < this->vector_len; i++){
        servo_desMsg_orig[i].joint_pos = 0.0;
        servo_desMsg_orig[i].joint_spd = 0.0;
        servo_desMsg_orig[i].joint_tor = 0.0;
        servo_desMsg_orig[i].joint_Kp = 0.0;
        servo_desMsg_orig[i].joint_Kd = 0.0;

        servo_retMsg_unpacked[i].joint_posRet = 0.0;
        servo_retMsg_unpacked[i].joint_spdRet = 0.0;
        servo_retMsg_unpacked[i].joint_currentRet = 0.0;
        servo_retMsg_unpacked[i].joint_temperature = 0.0;
    }

    ftrInitOutpTimes = 4000;
    ftrInitOutpCount = 0;
    ifFirstFtrInitOutp = true;

    //-----过程控制变量初始化-----

    servo_desVel_last.assign(this->vector_len, 0.0);

    lArm_transMod.assign(LARMDOF, 1);
    
    lArm_pos_receive = lArm_zeroPos;
    lArm_vel_receive.assign(LARMDOF, 0.0);

    lArm_torComp_feedforward_receive.assign(LARMDOF, 0.0);
    lArm_torComp_grav_receive.assign(LARMDOF, 0.0);
    lArm_torComp_interia_receive.assign(LARMDOF, 0.0);

    lArm_Kp_receive = lArm_defaultKp;
    lArm_Kd_receive = lArm_defaultKd;

    rArm_transMod.assign(RARMDOF, 1);

    rArm_pos_receive = rArm_zeroPos;
    rArm_vel_receive.assign(RARMDOF, 0.0);

    rArm_torComp_feedforward_receive.assign(RARMDOF, 0.0);
    rArm_torComp_grav_receive.assign(RARMDOF, 0.0);
    rArm_torComp_interia_receive.assign(RARMDOF, 0.0);

    rArm_Kp_receive = rArm_defaultKp;
    rArm_Kd_receive = rArm_defaultKd;

    //插补过程辅助变量
    ifFirstRunin = true;

    temp_len = 0;

    time_res = this->now().seconds() + this->now().nanoseconds() * 1e-9;
    time_last = this->now().seconds() + this->now().nanoseconds() * 1e-9;

    //-----插补器变量初始化-----
    l_posReceive_atRuninSt.assign(LARMDOF, 0.0);
    r_posReceive_atRuninSt.assign(RARMDOF, 0.0);

    ifRunin = true;

    //-----模式控制变量初始化-----
    tempTime_upperMonitor = 0.02;
    runin_mod = 2;
    if_topic_point_position = false;
    if_runin_renew = 0;
    msg_get_ifUseDebugMod = 0;
    if_useret_to_start_runin = 1;
}

//ros消息参量发送模块 -- 反馈值发送函数
void ServoNode::msg_publish(){
    auto tempRetData_pub = snowbot_ctl_msgs::msg::RetInformation();
    tempRetData_pub.header.stamp = this->now();  // 添加当前时间戳

    //获取并发送反馈值
    for(int i = 0; i < LARMDOF; i++){
        tempRetData_pub.l_position[i] = servo_retMsg_unpacked[i].joint_posRet;
        tempRetData_pub.l_speed[i] = servo_retMsg_unpacked[i].joint_spdRet;
        tempRetData_pub.l_iq[i] = servo_retMsg_unpacked[i].joint_currentRet;

        tempRetData_pub.r_position[i] = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
        tempRetData_pub.r_speed[i] = servo_retMsg_unpacked[i + LARMDOF].joint_spdRet;
        tempRetData_pub.r_iq[i] = servo_retMsg_unpacked[i + LARMDOF].joint_currentRet;
    }

    float left_gripper_pos = 0.0f, left_gripper_spd = 0.0f, left_gripper_cur = 0.0f;
    float right_gripper_pos = 0.0f, right_gripper_spd = 0.0f, right_gripper_cur = 0.0f;
    uint16_t left_gripper_status = 0, right_gripper_status = 0;
    getGripperFb(0, left_gripper_pos, left_gripper_spd, left_gripper_cur, left_gripper_status);
    getGripperFb(1, right_gripper_pos, right_gripper_spd, right_gripper_cur, right_gripper_status);
    tempRetData_pub.l_position[7] = left_gripper_pos;
    tempRetData_pub.l_speed[7] = left_gripper_spd;
    tempRetData_pub.l_iq[7] = left_gripper_cur;
    tempRetData_pub.r_position[7] = right_gripper_pos;
    tempRetData_pub.r_speed[7] = right_gripper_spd;
    tempRetData_pub.r_iq[7] = right_gripper_cur;

    if(ifFirstTemp){
        tempRetData_pub.if_first_time = 1;
        ifFirstTemp = false;
    }else{
        tempRetData_pub.if_first_time = 0;
    }

    RetData_pub->publish(tempRetData_pub);
}

//ros消息参量发送模块 -- 反馈值发送函数
void ServoNode::ctlmsg_get(){
    rclcpp::spin_some(this->get_node_base_interface());  // 非阻塞检查回调
}

//消息传输辅助函数 -- 仅仅进行一次消息的接收(与底层关节的通讯)
void ServoNode::EtherCATmsg_Get(){
    servo_retMsg = runMsgGet();
    retMsg_unpack_func();
}

//消息传输辅助函数 -- 进行一次消息的收发(与底层关节的通讯)
void ServoNode::EtherCATrun(){
    //进行一轮数据转换与收发
    desMsg_pack_func();
    servo_retMsg = runImpl(servo_desMsg);
    retMsg_unpack_func();
}

//主要循环函数
void ServoNode::run(){
    std::cout << "SOEM主站代码 -- snowbot关节控制与测试代码" << std::endl;

    //初始化ros迭代频率
    double loopRate_unrounded = 1 / TEMPTIME;
    int loopRate = static_cast<int>(loopRate_unrounded);
    rclcpp::Rate loop_rate(loopRate);
    std::cout << "执行频率：" << loopRate << " Hz。" << std::endl;

    //初始化EtherCAT通讯
    EtherCAT_Init(netCard_name.data());        //输入量 -- 电脑网卡名称
    std::cout << "slave count = " << ec_slavecount << std::endl;
    if(ec_slavecount <= 0){
        std::cout << "未找到从站，进入调试模式" << std::endl;
    }

    //开启transmit过程
    startRun();

    //初始化传输相关变量
    rosLoopVar_Init();


    //-----------------------------------------------
    // 初始使能: 跑 200 个空指令周期让 CiA402 使能序列完成 (200ms)
    {
        auto zero_cmd = JntDesMsg_inContainer_Outp_v1();
        for(int i = 0; i < 200; i++){
            servo_retMsg = runImpl(zero_cmd);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
        retMsg_unpack_func();
    }

    //-----------------------------------------------
    //检查是否有电机反馈
    bool hasFeedback = false;
    for(int i = 0; i < RARMDOF; i++){
        std::cout << "  电机 " << i
                  << " pos=" << std::fixed << servo_retMsg_unpacked[i + LARMDOF].joint_posRet
                  << " spd=" << std::fixed << servo_retMsg_unpacked[i + LARMDOF].joint_spdRet
                  << " cur=" << std::fixed << servo_retMsg_unpacked[i + LARMDOF].joint_currentRet << std::endl;
        if(std::abs(servo_retMsg_unpacked[i + LARMDOF].joint_posRet) > 1e-6){
            hasFeedback = true;
        }
    }
    if(!hasFeedback){
        std::cout << "警告：所有电机反馈为0，跳过回零控制。" << std::endl;
    }

    //回零控制
    // Position feedback can legitimately be all zero when the robot is already
    // at its mechanical zero. EtherCAT OP state is the hardware-ready check.
    bool arm_startup_ready = EtherCAT_isOperational();
    bool needRetZero = false;
    if(hasFeedback){
        for(int i = 0; i < LARMDOF; i++){
            if(std::abs(servo_retMsg_unpacked[i].joint_posRet - lArm_zeroPos[i]) > ZERO_POS_THRESHOLD){
                needRetZero = true;
                break;
            }
        }
        for(int i = 0; i < RARMDOF; i++){
            if(std::abs(servo_retMsg_unpacked[i + LARMDOF].joint_posRet - rArm_zeroPos[i]) > ZERO_POS_THRESHOLD){
                needRetZero = true;
                break;
            }
        }
    }

    //若达成回零控制条件，则开启回零控制
    bool arm_shutdown_handoff_pending = false;
    if(needRetZero){
        std::cout << "检测到关节偏离零点，开始回零控制..." << std::endl;

        //提取当前双臂位置
        std::vector<double> lArm_stPos(LARMDOF), rArm_stPos(RARMDOF);
        for(int i = 0; i < LARMDOF; i++){
            lArm_stPos[i] = servo_retMsg_unpacked[i].joint_posRet;
        }
        for(int i = 0; i < RARMDOF; i++){
            rArm_stPos[i] = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
        }

        // 回零前只使能偏离零点的关节
        bool retzero_mask[TOTAL_SLAVES] = {};
        for (int i = 0; i < LARMDOF; i++) {
            if (std::abs(servo_retMsg_unpacked[i].joint_posRet - lArm_zeroPos[i]) > ZERO_POS_THRESHOLD)
                retzero_mask[i] = true;
        }
        for (int i = 0; i < RARMDOF; i++) {
            if (std::abs(servo_retMsg_unpacked[i + LARMDOF].joint_posRet - rArm_zeroPos[i]) > ZERO_POS_THRESHOLD)
                retzero_mask[8 + i] = true;
        }
        EtherCAT_setMotorEnable(true);
        EtherCAT_setPerMotorEnable(retzero_mask);
        std::cout << "回零使能关节: ";
        for (int i = 0; i < LARMDOF; i++) if (retzero_mask[i]) std::cout << "L" << i+1 << " ";
        for (int i = 0; i < RARMDOF; i++) if (retzero_mask[8+i]) std::cout << "R" << i+1 << " ";
        std::cout << std::endl;

        //初始化回零插补轨迹（S型加减速，retZeroTime=2.0s）
        retZero_Init.retZero_runin(lArm_stPos, rArm_stPos);
        std::cout << "回零轨迹生成完毕，开始执行回零..." << std::endl;

        //执行回零控制循环 (重力前馈 + 小范围积分修正, 最长10s超时)
        bool retZero_done = false;
        float retzero_integral[14] = {0};
        bool traj_ended = false;
        int retzero_timeout = 0;
        while(!retZero_done && rclcpp::ok() && !sigInterrupt.load() &&
              retzero_timeout < kRetzeroTimeoutCycles){
            float l_des[LARMDOF], r_des[RARMDOF];
            float l_Kp[LARMDOF], r_Kp[RARMDOF], l_Kd[LARMDOF], r_Kd[RARMDOF];

            if (!traj_ended) {
                TrajCtlOutp retZero_outp = retZero_Init.retZero_trajOutp();
                for (int i = 0; i < LARMDOF; i++) {
                    l_des[i] = retZero_outp.lArm_desPos[i];
                    l_Kp[i]  = retZero_outp.lArm_Kp[i]; l_Kd[i] = retZero_outp.lArm_Kd[i];
                }
                for (int i = 0; i < RARMDOF; i++) {
                    r_des[i] = retZero_outp.rArm_desPos[i];
                    r_Kp[i]  = retZero_outp.rArm_Kp[i]; r_Kd[i] = retZero_outp.rArm_Kd[i];
                }
                if (retZero_outp.ifOutpReachEnd) {
                    traj_ended = true;
                    for (int i = 0; i < 14; ++i) retzero_integral[i] = 0.0f;
                }
            } else {
                for (int i = 0; i < LARMDOF; i++) {
                    l_des[i] = (float)lArm_zeroPos[i];
                    l_Kp[i]  = lArm_defaultKp[i]; l_Kd[i] = lArm_defaultKd[i];
                }
                for (int i = 0; i < RARMDOF; i++) {
                    r_des[i] = (float)rArm_zeroPos[i];
                    r_Kp[i]  = rArm_defaultKp[i]; r_Kd[i] = rArm_defaultKd[i];
                }
            }

            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].transMod = 1;
                servo_desMsg_orig[i].joint_pos = l_des[i];
                servo_desMsg_orig[i].joint_spd = 0.0;
                servo_desMsg_orig[i].joint_tor =
                    kLeftArmRetzeroGravityTorque[i] + retzero_integral[i];
                servo_desMsg_orig[i].joint_Kp  = l_Kp[i];
                servo_desMsg_orig[i].joint_Kd  = l_Kd[i];
            }
            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].transMod = 1;
                servo_desMsg_orig[i + LARMDOF].joint_pos = r_des[i];
                servo_desMsg_orig[i + LARMDOF].joint_spd = 0.0;
                servo_desMsg_orig[i + LARMDOF].joint_tor =
                    kRightArmRetzeroGravityTorque[i] + retzero_integral[7 + i];
                servo_desMsg_orig[i + LARMDOF].joint_Kp  = r_Kp[i];
                servo_desMsg_orig[i + LARMDOF].joint_Kd  = r_Kd[i];
            }

            EtherCATrun();
            rclcpp::sleep_for(std::chrono::milliseconds(1));

            // 积分修正 (Ki=500, 左右臂统一限幅±10Nm)
            for (int i = 0; i < LARMDOF; i++) {
                float err = l_des[i] - servo_retMsg_unpacked[i].joint_posRet;
                retzero_integral[i] += err * 0.001f * 500.0f;
                if (retzero_integral[i] > kRetzeroTrimLimit) retzero_integral[i] = kRetzeroTrimLimit;
                if (retzero_integral[i] < -kRetzeroTrimLimit) retzero_integral[i] = -kRetzeroTrimLimit;
            }
            for (int i = 0; i < RARMDOF; i++) {
                float err = r_des[i] - servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
                retzero_integral[7 + i] += err * 0.001f * 500.0f;
                if (retzero_integral[7 + i] > kRetzeroTrimLimit) retzero_integral[7 + i] = kRetzeroTrimLimit;
                if (retzero_integral[7 + i] < -kRetzeroTrimLimit) retzero_integral[7 + i] = -kRetzeroTrimLimit;
            }

            retzero_timeout++;

            msg_publish();

            if (traj_ended) {
                bool all_reached = true;
                for (int i = 0; i < LARMDOF; i++)
                    if (retzero_mask[i] && std::abs(servo_retMsg_unpacked[i].joint_posRet - lArm_zeroPos[i]) > ZERO_POS_EXIT_THRESH)
                        all_reached = false;
                for (int i = 0; i < RARMDOF; i++)
                    if (retzero_mask[8+i] && std::abs(servo_retMsg_unpacked[i + LARMDOF].joint_posRet - rArm_zeroPos[i]) > ZERO_POS_EXIT_THRESH)
                        all_reached = false;
                static int settle_cnt = 0;
                if (all_reached) settle_cnt++; else settle_cnt = 0;
                if (settle_cnt > 500) retZero_done = true;
            }
        }

        // 无论严格完成还是超时，都先失能并等待实时 PDO 线程下发失能控制字。
        bool all_off_mask[TOTAL_SLAVES] = {};
        EtherCAT_setPerMotorEnable(all_off_mask);
        EtherCAT_setMotorEnable(false);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
        EtherCATmsg_Get();

        bool residual_within_startup_tolerance = EtherCAT_communicationHealthy();
        for (int i = 0; i < LARMDOF; ++i) {
            if (retzero_mask[i] &&
                std::abs(servo_retMsg_unpacked[i].joint_posRet - lArm_zeroPos[i]) >
                    kRetzeroTimeoutAcceptError) {
                residual_within_startup_tolerance = false;
            }
        }
        for (int i = 0; i < RARMDOF; ++i) {
            if (retzero_mask[8 + i] &&
                std::abs(servo_retMsg_unpacked[i + LARMDOF].joint_posRet - rArm_zeroPos[i]) >
                    kRetzeroTimeoutAcceptError) {
                residual_within_startup_tolerance = false;
            }
        }

        if (retZero_done) {
            std::cout << "回零控制完成，关节已到达严格零位，手臂电机已失能锁死。" << std::endl;
        } else {
            std::cout << "回零超时，手臂电机已失能锁死。" << std::endl;
        }

        arm_startup_ready = residual_within_startup_tolerance;
        if (!arm_startup_ready) {
            std::cerr << "回零后残差超过 " << kRetzeroTimeoutAcceptError
                      << " rad 或 EtherCAT WKC 异常，后续控制节点不会启动。" << std::endl;
        }
    }else{
        std::cout << "关节已在零点附近，跳过回零控制。" << std::endl;
    }

    // Ctrl+C 在双臂开机回零期间到来时，锁住双臂输出，直到关机目标
    // 已经写入命令缓存后再交给关机回零接管。
    if (sigInterrupt.load()) {
        EtherCAT_BeginArmShutdownHandoff();
        EtherCAT_setMotorEnable(false);
        arm_shutdown_handoff_pending = true;
        std::cout << "双臂启动回零已停止，等待关机回零目标接管。" << std::endl;
    }

    //-----------------------------------------------
    // 头部与腰部 CSP 模式启动回零 (Alias 17~18: 头部, 19~20: 躯干)
    bool head_torso_startup_ready = true;
    if (!sigInterrupt.load()) {
        bool head_torso_need_homing = false;
        float h_fb[HEAD_MOTORS] = {0}, t_fb[TORSO_MOTORS] = {0};
        float cur_p, cur_s, cur_c; uint16_t cur_st;

        for (int i = 0; i < HEAD_MOTORS; i++) {
            getMotorFb_head(i, cur_p, cur_s, cur_c, cur_st);
            h_fb[i] = cur_p;
            if (std::abs(cur_p) > 0.01f) head_torso_need_homing = true;
        }
        for (int i = 0; i < TORSO_MOTORS; i++) {
            getMotorFb_torso(i, cur_p, cur_s, cur_c, cur_st);
            t_fb[i] = cur_p;
            if (std::abs(cur_p) > 0.01f) head_torso_need_homing = true;
        }

        if (head_torso_need_homing) {
            std::cout << "检测到头/腰电机偏离零位，开始 CSP 并行回零..." << std::endl;
            // 使能头腰电机 (Slot 16~19)
            bool ht_mask[TOTAL_SLAVES] = {};
            for (int i = 16; i <= 19; i++) ht_mask[i] = true;
            EtherCAT_setMotorEnable(true);
            EtherCAT_setPerMotorEnable(ht_mask);

            // 斜坡步进回零 (1kHz 周期，每周期最大变化 0.001 rad，相当于 1.0 rad/s)
            const float MAX_STEP = 0.001f;
            float h_tgt[HEAD_MOTORS], t_tgt[TORSO_MOTORS];
            for (int i = 0; i < HEAD_MOTORS; i++) h_tgt[i] = h_fb[i];
            for (int i = 0; i < TORSO_MOTORS; i++) t_tgt[i] = t_fb[i];

            int ht_timeout = 0;
            bool ht_done = false;
            while (!ht_done && rclcpp::ok() && !sigInterrupt.load() && ht_timeout < 10000) {
                bool all_zero = true;
                for (int i = 0; i < HEAD_MOTORS; i++) {
                    if (h_tgt[i] > MAX_STEP) { h_tgt[i] -= MAX_STEP; all_zero = false; }
                    else if (h_tgt[i] < -MAX_STEP) { h_tgt[i] += MAX_STEP; all_zero = false; }
                    else { h_tgt[i] = 0.0f; }
                    setHeadMotorCmd(i, h_tgt[i], 0.0, 0.0, 0.0, 0.0);
                }
                for (int i = 0; i < TORSO_MOTORS; i++) {
                    if (t_tgt[i] > MAX_STEP) { t_tgt[i] -= MAX_STEP; all_zero = false; }
                    else if (t_tgt[i] < -MAX_STEP) { t_tgt[i] += MAX_STEP; all_zero = false; }
                    else { t_tgt[i] = 0.0f; }
                    setTorsoMotorCmd(i, t_tgt[i], 0.0, 0.0, 0.0, 0.0);
                }

                EtherCATrun();
                rclcpp::sleep_for(std::chrono::milliseconds(1));
                ht_timeout++;

                if (all_zero) {
                    // 确认当前反馈是否稳定在零点附近
                    bool stable = true;
                    for (int i = 0; i < HEAD_MOTORS; i++) {
                        getMotorFb_head(i, cur_p, cur_s, cur_c, cur_st);
                        if (std::abs(cur_p) > 0.01f) stable = false;
                    }
                    for (int i = 0; i < TORSO_MOTORS; i++) {
                        getMotorFb_torso(i, cur_p, cur_s, cur_c, cur_st);
                        if (std::abs(cur_p) > 0.01f) stable = false;
                    }
                    static int ht_settle = 0;
                    if (stable) ht_settle++; else ht_settle = 0;
                    if (ht_settle > 200) ht_done = true;
                }
            }
            if (ht_done) {
                std::cout << "头/腰电机 CSP 回零完成 (耗时 " << ht_timeout << " ms)。" << std::endl;
            } else {
                head_torso_startup_ready = false;
                std::cout << "头/腰电机 CSP 回零超时，后续控制节点不会启动。" << std::endl;
            }
        } else {
            std::cout << "头/腰电机已在零位附近，跳过 CSP 回零。" << std::endl;
        }
    } else {
        head_torso_startup_ready = false;
        std::cout << "检测到退出请求，跳过头/腰启动回零，直接进入关机回零。" << std::endl;
    }

    if (arm_startup_ready && head_torso_startup_ready && !sigInterrupt.load()) {
        notify_systemd("READY=1\nSTATUS=Startup homing complete; arm brakes engaged");
        std::cout << "[systemd] Servo startup ready; downstream services may start." << std::endl;
    } else {
        notify_systemd("STATUS=Startup homing failed; downstream services blocked");
        std::cerr << "[systemd] Servo startup is not ready; downstream services are blocked." << std::endl;
    }

    //-----------------------------------------------
    // 关机回零状态变量 (Ctrl+C 后按 shutdownpos.yaml 回零)
    bool shutdown_need[TOTAL_SLAVES] = {};
    std::vector<double> lArm_shutdown_start(LARMDOF, 0.0), rArm_shutdown_start(RARMDOF, 0.0);
    std::chrono::steady_clock::time_point shutdown_start_time;
    std::chrono::steady_clock::time_point shutdown_settle_start_time;

    //开启ros2循环
    while (rclcpp::ok()){
        //----- Ctrl+C 关机回零: 像上位机下发一样, 检测需要回零的关节并单独使能 -----
        if (sigInterrupt.load() && !shutdown_mode_) {
            shutdown_mode_ = true;
            shutdown_start_time = std::chrono::steady_clock::now();
            shutdown_settle_start_time = {};
            std::cout << "收到中断信号，进入关机回零模式..." << std::endl;

            const double SHUTDOWN_DELTA = 0.05;
            // 双臂: 偏离关机位置的关节才使能
            for (int i = 0; i < LARMDOF; i++) {
                shutdown_need[i] = std::abs(servo_retMsg_unpacked[i].joint_posRet - lArm_shutdownPos[i]) > SHUTDOWN_DELTA;
                EtherCAT_setSingleMotorEnable(i, shutdown_need[i]);
            }
            for (int i = 0; i < RARMDOF; i++) {
                shutdown_need[8 + i] = std::abs(servo_retMsg_unpacked[i + LARMDOF].joint_posRet - rArm_shutdownPos[i]) > SHUTDOWN_DELTA;
                EtherCAT_setSingleMotorEnable(8 + i, shutdown_need[8 + i]);
            }
            // 夹爪 (槽位 7/15)
            shutdown_need[7]  = std::abs(getGripperFbPos(0) - lGripper_shutdownPos) > SHUTDOWN_DELTA;
            shutdown_need[15] = std::abs(getGripperFbPos(1) - rGripper_shutdownPos) > SHUTDOWN_DELTA;
            EtherCAT_setSingleMotorEnable(7, shutdown_need[7]);
            EtherCAT_setSingleMotorEnable(15, shutdown_need[15]);
            // 头腰 (槽位 16~19, CSP 回零到 0)
            {
                float p = 0.0f, sp = 0.0f, c = 0.0f; uint16_t stt = 0;
                for (int i = 0; i < HEAD_MOTORS; i++) {
                    getMotorFb_head(i, p, sp, c, stt);
                    shutdown_need[16 + i] = std::abs(p) > 0.01f;
                    EtherCAT_setSingleMotorEnable(16 + i, shutdown_need[16 + i]);
                }
                for (int i = 0; i < TORSO_MOTORS; i++) {
                    getMotorFb_torso(i, p, sp, c, stt);
                    shutdown_need[18 + i] = std::abs(p) > 0.01f;
                    EtherCAT_setSingleMotorEnable(18 + i, shutdown_need[18 + i]);
                }
            }
            EtherCAT_setMotorEnable(true);
            std::cout << "关机回零使能关节: ";
            for (int i = 0; i < LARMDOF; i++) if (shutdown_need[i]) std::cout << "L" << i+1 << " ";
            for (int i = 0; i < RARMDOF; i++) if (shutdown_need[8+i]) std::cout << "R" << i+1 << " ";
            if (shutdown_need[7])  std::cout << "GL ";
            if (shutdown_need[15]) std::cout << "GR ";
            if (shutdown_need[16] || shutdown_need[17] || shutdown_need[18] || shutdown_need[19]) std::cout << "HT ";
            std::cout << std::endl;

            // 手臂插补起点: 当前反馈
            for (int i = 0; i < LARMDOF; i++) lArm_shutdown_start[i] = servo_retMsg_unpacked[i].joint_posRet;
            for (int i = 0; i < RARMDOF; i++) rArm_shutdown_start[i] = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;

            // 夹爪/头腰目标: 复用主循环现有的平滑插补器
            g_left_pos_curr  = getGripperFbPos(0);
            g_left_pos_start = g_left_pos_curr;
            g_left_pos_target = lGripper_shutdownPos;
            g_left_winlen = 3000; g_left_step = 0; g_left_ifRunin = true;
            g_left_spd = 0.0; g_left_tor = 0.0;   // 清零残留速度, 否则 KD*spd 产生零位偏差
            g_right_pos_curr  = getGripperFbPos(1);
            g_right_pos_start = g_right_pos_curr;
            g_right_pos_target = rGripper_shutdownPos;
            g_right_winlen = 3000; g_right_step = 0; g_right_ifRunin = true;
            g_right_spd = 0.0; g_right_tor = 0.0;
            {
                float p = 0.0f, sp = 0.0f, c = 0.0f; uint16_t stt = 0;
                for (int i = 0; i < TORSO_MOTORS; i++) {
                    getMotorFb_torso(i, p, sp, c, stt);
                    t_pos_curr[i] = p; t_pos_start[i] = p; t_pos_target[i] = 0.0;
                }
                t_winlen = 3000; t_step = 0; t_ifRunin = true;
                for (int i = 0; i < HEAD_MOTORS; i++) {
                    getMotorFb_head(i, p, sp, c, stt);
                    h_pos_curr[i] = p; h_pos_start[i] = p; h_pos_target[i] = 0.0;
                }
                h_winlen = 3000; h_step = 0; h_ifRunin = true;
            }

            // 刷新手臂插补器: 以当前反馈为起点
            if_useret_to_start_runin = 1;
            if_runin_renew = 1;
            // 跳过滤波器初始化段, 避免覆盖关机目标
            ftrInitOutpCount = ftrInitOutpTimes;
        }

        //第一轮控制，先直接获取一下反馈值
        if(ifShallInit){
            for(int i = 0; i < 10; i++){
                EtherCATmsg_Get();
            }

            //初始化目标位置
            for(int i = 0; i < SINGLEARMJNTNUM; i++){
                servo_desMsg_orig[i].joint_pos = servo_retMsg_unpacked[i].joint_posRet;
                servo_desMsg_orig[i + LARMDOF].joint_pos = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
            }

            ifShallInit = false;
        }

        //发送一轮反馈值
        msg_publish();

        //获取一轮控制量
        ctlmsg_get();

        //----- 关机回零: 覆盖上位机目标, 走同一套插补链路 (像上位机下发一样) -----
        if (shutdown_mode_) {
            const auto shutdown_now = std::chrono::steady_clock::now();
            const auto shutdown_elapsed = shutdown_now - shutdown_start_time;
            double u = std::chrono::duration<double>(shutdown_elapsed).count() /
                       std::chrono::duration<double>(kShutdownTrajectoryDuration).count();
            if (u > 1.0) u = 1.0;
            double s = 3.0 * u * u - 2.0 * u * u * u;
            for (int i = 0; i < LARMDOF; i++) {
                lArm_pos_receive[i] = lArm_shutdown_start[i] + (lArm_shutdownPos[i] - lArm_shutdown_start[i]) * s;
            }
            for (int i = 0; i < RARMDOF; i++) {
                rArm_pos_receive[i] = rArm_shutdown_start[i] + (rArm_shutdownPos[i] - rArm_shutdown_start[i]) * s;
            }
            lArm_vel_receive.assign(LARMDOF, 0.0);
            rArm_vel_receive.assign(RARMDOF, 0.0);
            lArm_torComp_feedforward_receive.assign(LARMDOF, 0.0);
            lArm_torComp_grav_receive.assign(LARMDOF, 0.0);
            lArm_torComp_interia_receive.assign(LARMDOF, 0.0);
            rArm_torComp_feedforward_receive.assign(RARMDOF, 0.0);
            rArm_torComp_grav_receive.assign(RARMDOF, 0.0);
            rArm_torComp_interia_receive.assign(RARMDOF, 0.0);
            lArm_Kp_receive = lArm_defaultKp;
            lArm_Kd_receive = lArm_defaultKd;
            rArm_Kp_receive = rArm_defaultKp;
            rArm_Kd_receive = rArm_defaultKd;
            lArm_transMod.assign(LARMDOF, 1);
            rArm_transMod.assign(RARMDOF, 1);
            ifRunin = true;   // 保证每周期都执行插补

            // 到位判定 (斜坡结束后)
            if (shutdown_elapsed > kShutdownSettleDelay) {
                bool reached = true;
                for (int i = 0; i < LARMDOF; i++)
                    if (shutdown_need[i] && std::abs(servo_retMsg_unpacked[i].joint_posRet - lArm_shutdownPos[i]) > 0.02) reached = false;
                for (int i = 0; i < RARMDOF; i++)
                    if (shutdown_need[8+i] && std::abs(servo_retMsg_unpacked[i + LARMDOF].joint_posRet - rArm_shutdownPos[i]) > 0.02) reached = false;
                if (shutdown_need[7]  && std::abs(getGripperFbPos(0) - lGripper_shutdownPos) > 0.02) reached = false;
                if (shutdown_need[15] && std::abs(getGripperFbPos(1) - rGripper_shutdownPos) > 0.02) reached = false;
                if (shutdown_need[16] || shutdown_need[17]) {
                    float p = 0.0f, sp = 0.0f, c = 0.0f; uint16_t stt = 0;
                    for (int i = 0; i < HEAD_MOTORS; i++) { getMotorFb_head(i, p, sp, c, stt); if (std::abs(p) > 0.01f) reached = false; }
                }
                if (shutdown_need[18] || shutdown_need[19]) {
                    float p = 0.0f, sp = 0.0f, c = 0.0f; uint16_t stt = 0;
                    for (int i = 0; i < TORSO_MOTORS; i++) { getMotorFb_torso(i, p, sp, c, stt); if (std::abs(p) > 0.01f) reached = false; }
                }
                if (!reached) {
                    shutdown_settle_start_time = {};
                } else if (shutdown_settle_start_time == std::chrono::steady_clock::time_point{}) {
                    shutdown_settle_start_time = shutdown_now;
                } else if (shutdown_now - shutdown_settle_start_time >= kShutdownSettleDuration) {
                    std::cout << "关机回零完成，已到达设定姿态。" << std::endl;
                    break;
                }
            }
            if (shutdown_elapsed >= kShutdownTimeout) {
                std::cout << "关机回零超时 (8s)，强制失能退出。" << std::endl;
                break;
            }
        }

        //-----轨迹计算-----
        //解包、插补并计算 -- 待修改：在插补过程中添加归零模式和阻尼回零模式
        ctlMsg_runin();
        // 关机回零只以当前反馈初始化一次插补器，后续周期持续追踪同一个关机目标。
        if (shutdown_mode_) {
            if_runin_renew = 0;
        }

        //-----轨迹装入-----

        //数据处理过程量定义

        //位置量、速度量初始值
        std::vector<double> lArm_posOrig, rArm_posOrig;
        std::vector<double> lArm_velOrig, rArm_velOrig;
        std::vector<double> lArm_TorOrig, rArm_TorOrig;

        //滤波结果
        std::vector<std::vector<double>> pos_ftrRes;
        std::vector<std::vector<double>> vel_ftrRes;
        std::vector<std::vector<double>> Tor_ftrRes;

        //初始化
        lArm_posOrig.assign(LARMDOF, 0.0);
        rArm_posOrig.assign(RARMDOF, 0.0);
        lArm_velOrig.assign(LARMDOF, 0.0);
        rArm_velOrig.assign(RARMDOF, 0.0);
        lArm_TorOrig.assign(LARMDOF, 0.0);
        rArm_TorOrig.assign(RARMDOF, 0.0);
        pos_ftrRes.resize(2);
        pos_ftrRes[0].assign(LARMDOF, 0.0);
        pos_ftrRes[1].assign(RARMDOF, 0.0);
        vel_ftrRes.resize(2);
        vel_ftrRes[0].assign(LARMDOF, 0.0);
        vel_ftrRes[1].assign(RARMDOF, 0.0);
        Tor_ftrRes.resize(2);
        Tor_ftrRes[0].assign(LARMDOF, 0.0);
        Tor_ftrRes[1].assign(RARMDOF, 0.0);

        //获取本轮插补值
        //-----夹爪/躯干/头部插补 (独立于双臂插补器)-----
        gripper_runin();
        torso_runin();
        head_runin();

        if(ifRuninerInited){    // --- 若进行了手臂插补的操作 ---
            //-----获取初始轨迹-----
            outpTraj_process();

            //-----数据处理以及补偿量计算-----

            //滤波输入量装入
            for(int i = 0; i < LARMDOF; i++){
                lArm_posOrig[i] = servo_desMsg_orig[i].joint_pos;
                lArm_velOrig[i] = servo_desMsg_orig[i].joint_spd;
                lArm_TorOrig[i] = servo_desMsg_orig[i].joint_tor;
            }
            for(int i = 0; i < RARMDOF; i++){
                rArm_posOrig[i] = servo_desMsg_orig[i + LARMDOF].joint_pos;
                rArm_velOrig[i] = servo_desMsg_orig[i + LARMDOF].joint_spd;
                rArm_TorOrig[i] = servo_desMsg_orig[i + LARMDOF].joint_tor;
            }

            //滤波计算
            pos_ftrRes = posOutpFtr.Outp_ftr(
                lArm_posOrig, rArm_posOrig
            );

            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].joint_pos = pos_ftrRes[0][i];
            }
            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].joint_pos = pos_ftrRes[1][i];
            }

            vel_ftrRes = velOutpFtr.Outp_ftr(
                lArm_velOrig, rArm_velOrig
            );
            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].joint_spd = vel_ftrRes[0][i];
            }
            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].joint_spd = vel_ftrRes[1][i];
            }

            Tor_ftrRes = TorOutpFtr.Outp_ftr(
                lArm_TorOrig, rArm_TorOrig
            );
            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].joint_tor = Tor_ftrRes[0][i];
            }
            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].joint_tor = Tor_ftrRes[1][i];
            }

            //刚度补偿 -- 暂时不做

            //摩擦补偿 -- 暂时不做

            //----------初始段输出处理代码----------
            // Topic 位置流的滤波器首帧已用当前目标初始化，不应再强制覆盖为零位。
            // Demo/VR 继续保留原来的滤波器预热保持逻辑。
            const bool is_topic_joint_stream = if_topic_point_position;
            if(ftrInitOutpCount < ftrInitOutpTimes && !is_topic_joint_stream){
                if(ifFirstFtrInitOutp){
                    std::cout << " ** 滤波器初始化中 ** " << std::endl;
                    ifFirstFtrInitOutp = false;
                }
                for(int i = 0; i < LARMDOF; i++){
                    servo_desMsg_orig[i].joint_pos = lArm_zeroPos[i];
                    servo_desMsg_orig[i].joint_spd = 0.0;
                    servo_desMsg_orig[i].joint_tor = 0.0;
                }
                for(int i = 0; i < RARMDOF; i++){
                    servo_desMsg_orig[i + LARMDOF].joint_pos = rArm_zeroPos[i];
                    servo_desMsg_orig[i + LARMDOF].joint_spd = 0.0;
                    servo_desMsg_orig[i + LARMDOF].joint_tor = 0.0;
                }
            }else if(ftrInitOutpCount == ftrInitOutpTimes){
                std::cout << " ** 滤波器初始化完毕 ** " << std::endl;
            }

            if(ftrInitOutpCount < 100000){
                ftrInitOutpCount += 1;
            }

            //-----------------------------------

        }else{
            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].transMod = 1;

                servo_desMsg_orig[i].joint_pos = lArm_zeroPos[i];
                servo_desMsg_orig[i].joint_spd = 0.0;
                servo_desMsg_orig[i].joint_tor = 0.0;

                servo_desMsg_orig[i].joint_Kp = lArm_defaultKp[i];
                servo_desMsg_orig[i].joint_Kd = lArm_defaultKd[i];
            }

            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].transMod = 1;

                servo_desMsg_orig[i + LARMDOF].joint_pos = rArm_zeroPos[i];
                servo_desMsg_orig[i + LARMDOF].joint_spd = 0.0;
                servo_desMsg_orig[i + LARMDOF].joint_tor = 0.0;

                servo_desMsg_orig[i + LARMDOF].joint_Kp = rArm_defaultKp[i];
                servo_desMsg_orig[i + LARMDOF].joint_Kd = rArm_defaultKd[i];
            }
        }

        //状态监测 -- 输出底层状态

        //辅助量赋值
        std::vector<double> lArm_joint_tor, rArm_joint_tor;

        std::vector<double> lArm_pos_ret, rArm_pos_ret;
        std::vector<double> lArm_vel_ret, rArm_vel_ret;
        std::vector<double> lArm_current_ret, rArm_current_ret;
        std::vector<double> lArm_temperature, rArm_temperature;
        std::vector<double> l_Kp, r_Kp, l_Kd, r_Kd;

        lArm_joint_tor.assign(LARMDOF, 0.0);    rArm_joint_tor.assign(RARMDOF, 0.0);

        lArm_pos_ret.assign(LARMDOF, 0.0);      rArm_pos_ret.assign(RARMDOF, 0.0);
        lArm_vel_ret.assign(LARMDOF, 0.0);      rArm_vel_ret.assign(RARMDOF, 0.0);
        lArm_current_ret.assign(LARMDOF, 0.0);  rArm_current_ret.assign(RARMDOF, 0.0);
        lArm_temperature.assign(LARMDOF, 0.0);  rArm_temperature.assign(RARMDOF, 0.0);
        l_Kp.assign(LARMDOF, 0.0);              r_Kp.assign(RARMDOF, 0.0);
        l_Kd.assign(LARMDOF, 0.0);              r_Kd.assign(RARMDOF, 0.0);

        for(int i = 0; i < LARMDOF; i++){
            lArm_joint_tor[i] = servo_desMsg_orig[i].joint_tor;

            lArm_pos_ret[i] = servo_retMsg_unpacked[i].joint_posRet;
            lArm_vel_ret[i] = servo_retMsg_unpacked[i].joint_spdRet;
            lArm_current_ret[i] = servo_retMsg_unpacked[i].joint_currentRet;
            lArm_temperature[i] = servo_retMsg_unpacked[i].joint_temperature;

            l_Kp[i] = servo_desMsg_orig[i].joint_Kp;
            l_Kd[i] = servo_desMsg_orig[i].joint_Kd;
        };
        for(int i = 0; i < RARMDOF; i++){
            rArm_joint_tor[i] = servo_desMsg_orig[i + LARMDOF].joint_tor;

            rArm_pos_ret[i] = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
            rArm_vel_ret[i] = servo_retMsg_unpacked[i + LARMDOF].joint_spdRet;
            rArm_current_ret[i] = servo_retMsg_unpacked[i + LARMDOF].joint_currentRet;
            rArm_temperature[i] = servo_retMsg_unpacked[i + LARMDOF].joint_temperature;

            r_Kp[i] = servo_desMsg_orig[i + LARMDOF].joint_Kp;
            r_Kd[i] = servo_desMsg_orig[i + LARMDOF].joint_Kd;
        };

        auto m_now = this->now();
        double m_timeSpot = m_now.seconds() + m_now.nanoseconds() * 1e-9;
        performance_test->comprehensive_msg_pub->ctl_res_pub_PVTmod(
            lArm_posOrig, rArm_posOrig,
            pos_ftrRes[0], pos_ftrRes[1],
            lArm_velOrig, rArm_velOrig,
            vel_ftrRes[0], vel_ftrRes[1],
            lArm_joint_tor, rArm_joint_tor,

            l_Kp, r_Kp,
            l_Kd, r_Kd,

            lArm_pos_ret, rArm_pos_ret,
            lArm_vel_ret, rArm_vel_ret,
            lArm_current_ret, rArm_current_ret,

            m_timeSpot
        );


        //刷新逻辑（未使用）
        if(has_received_command_){
            has_received_command_ = false;
        }else{

        }

        //----------------------
        //阻抗控制力矩注入 (若已使能): 覆盖 servo_desMsg_orig 的力矩/位置/Kp,Kd
        applyImpedanceControl();

        //----------------------
        //进行一轮EtherCAT发送
        EtherCATrun();
        if (arm_shutdown_handoff_pending && shutdown_mode_) {
            EtherCAT_EndArmShutdownHandoff();
            arm_shutdown_handoff_pending = false;
            std::cout << "关机回零目标已写入，双臂控制权完成切换。" << std::endl;
        }

        //发布六维力传感器数据
        publish_sensor_data();

        //尾处理代码
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }

    //-----------------------------------------------
    // 关机收尾: 全部失能锁死
    bool all_off_mask[TOTAL_SLAVES] = {};
    EtherCAT_setPerMotorEnable(all_off_mask);
    EtherCAT_setMotorEnable(false);
    std::cout << "机械臂电机已失能，EtherCAT驱动安全停机。" << std::endl;
    RCLCPP_INFO(this->get_logger(), "程序已安全退出");
}

//目标值包装协议
void ServoNode::desMsg_pack_func(){
    /**
    int temp = 0;
    for(int i = 0; i < servo_jnt_topology.board_count; i++){
        for(int j = 0; j < servo_jnt_topology.joints_per_board[i]; j++){
            servo_desMsg[i][j] = servo_desMsg_orig[temp];
            temp += 1;
        }
    }
    /**/

    // 直接 EtherCAT: 平铺映射
    int temp = 0;
    for(int i = 0; i < servo_jnt_topology.board_count; i++){
        for(int j = 0; j < servo_jnt_topology.joints_per_board[i]; j++){
            servo_desMsg[i][j] = servo_desMsg_orig[temp];
            temp += 1;
        }
    }
}

//反馈值解包协议
void ServoNode::retMsg_unpack_func(){
    servo_retMsg_unpacked.clear();

    /**
    for(int i = 0; i < servo_jnt_topology.board_count; i++){
        for(int j = 0; j < servo_jnt_topology.joints_per_board[i]; j++){
            servo_retMsg_unpacked.push_back(servo_retMsg[i][j]);
        }
    }
    /**/

    // 直接 EtherCAT: 平铺解包
    for(int i = 0; i < servo_jnt_topology.board_count; i++){
        for(int j = 0; j < servo_jnt_topology.joints_per_board[i]; j++){
            servo_retMsg_unpacked.push_back(servo_retMsg[i][j]);
        }
    }
}

//传输信息回调函数 -- 接收消息并执行插补任务
void ServoNode::message_callback(const snowbot_ctl_msgs::msg::CtlInformation::SharedPtr msg){

    // 关机回零独占目标、插补和使能控制，不能被排队的上位机消息重新覆盖。
    if (shutdown_mode_) {
        return;
    }

    //--------------------接收消息--------------------

    //检查时间戳是否合理（例如，延迟不超过 5ms）
    auto now = this->now();
    auto latency = now - msg->header.stamp;
    if (latency.seconds() > 0.005) {
        RCLCPP_WARN(this->get_logger(), "High latency detected: %f seconds", latency.seconds());
    }

    //模式控制变量赋值
    tempTime_upperMonitor = msg->temp_time;
    runin_mod = msg->runin_mod;
    if_topic_point_position = msg->if_topic_point_position;
    if_runin_renew = msg->if_runin_renew;
    msg_get_ifUseDebugMod = msg->if_use_debugmod;
    if_useret_to_start_runin = msg->if_useret_to_start_runin;

    // 电机使能控制 (motor_enable=0 忽略, =1 按 mask)
    static bool retZero_lock = true;  // 回零后锁死
    if (msg->motor_enable == 0) {
        // motor_enable=0 是 idle 超时, 忽略
    } else if (retZero_lock) {
        retZero_lock = false;  // 第一条使能消息仅解锁
    } else if (!shutdown_mode_) {   // 关机回零期间忽略上位机使能指令
        for (int i = 0; i < LARMDOF; i++) EtherCAT_setSingleMotorEnable(i, msg->l_motor_enable[i] != 0);
        for (int i = 0; i < RARMDOF; i++) EtherCAT_setSingleMotorEnable(8 + i, msg->r_motor_enable[i] != 0);
        EtherCAT_setSingleMotorEnable(7, true);   // 左夹爪
        EtherCAT_setSingleMotorEnable(15, true);  // 右夹爪
        EtherCAT_setMotorEnable(true);
    }

    //关节底层控制模式变量赋值
    for(int i = 0; i < LARMDOF; i++){
        lArm_transMod[i] = msg->base_rigidity_ctl_mode_l[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        rArm_transMod[i] = msg ->base_rigidity_ctl_mode_r[i];
    }

    //左右臂插补位置初始化变量刷新
    for(int i = 0; i < LARMDOF; i++){
        l_posReceive_atRuninSt[i] = msg->l_position_at_runin_st[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        r_posReceive_atRuninSt[i] = msg->r_position_at_runin_st[i];
    }

    //控制消息获取
    for(int i = 0; i < LARMDOF; i++){
        lArm_pos_receive[i] = msg->l_position[i];
        lArm_vel_receive[i] = msg->l_speed[i];

        lArm_torComp_grav_receive[i] = msg->l_torque_grav_comp[i];
        lArm_torComp_interia_receive[i] = msg->l_torque_interia_comp[i];
        lArm_torComp_feedforward_receive[i] = msg->l_torque_feedforward[i];

        lArm_Kp_receive[i] = msg->l_kp[i];
        lArm_Kd_receive[i] = msg->l_kd[i];
    }

    for(int i = 0; i < RARMDOF; i++){
        rArm_pos_receive[i] = msg->r_position[i];
        rArm_vel_receive[i] = msg->r_speed[i];

        rArm_torComp_grav_receive[i] = msg->r_torque_grav_comp[i];
        rArm_torComp_interia_receive[i] = msg->r_torque_interia_comp[i];
        rArm_torComp_feedforward_receive[i] = msg->r_torque_feedforward[i];

        rArm_Kp_receive[i] = msg->r_kp[i];
        rArm_Kd_receive[i] = msg->r_kd[i];
    }

    //--------------------标志位更新--------------------

    //更新插补启动标志
    ifRunin = true;

    //更新获取反馈值标志位
    has_received_command_ = true;
}

//函数2 -- 执行插补函数
void ServoNode::ctlMsg_runin(){

    if(ifRunin){
        //更新插补启动标志
        ifRunin = false;

        //更新插补初始位置
        if(if_useret_to_start_runin){
            //若采用反馈值作为插补初始值，则需要更新插补初始关键量
            for(int i = 0; i < LARMDOF; i++){
                l_posReceive_atRuninSt[i] = servo_retMsg_unpacked[i].joint_posRet;
            }
            for(int i = 0; i < RARMDOF; i++){
                r_posReceive_atRuninSt[i] = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
            }
        }

        //判断是否要更新插补
        bool if_runin_renew_bool = false;
        if(if_runin_renew == 1){
            if_runin_renew_bool = true;
        }
        bool if_temp_runin_renew = ifFirstRunin || if_runin_renew_bool;
        ifFirstRunin = false;

        //执行刷新代码
        if(if_temp_runin_renew){
            runin_control_core.init(
                l_posReceive_atRuninSt, r_posReceive_atRuninSt,
                20
            );
            ifRuninerInited = true;
        }

        temp_len = 0;

        //获取插补长度
        time_res = this->now().seconds() + this->now().nanoseconds() * 1e-9;
        winlen = std::round((time_res - time_last) / 0.002) + 2;
        if(winlen > 100){
            winlen = 100;
        }
        time_last = time_res;

        // Topic 流按上层配置线性补点，避免每个 50Hz 轨迹点都被 B 样条重新拟合；
        // 其他控制源继续沿用既有的二次样条补点。
        const int servo_runin_mod = (runin_mod == 1) ? 1 : 2;
        runin_control_core.ModelRunin(
            lArm_pos_receive,
            lArm_vel_receive,
            lArm_torComp_grav_receive,
            lArm_torComp_interia_receive,
            lArm_torComp_feedforward_receive,

            rArm_pos_receive,
            rArm_vel_receive,
            rArm_torComp_grav_receive,
            rArm_torComp_interia_receive,
            rArm_torComp_feedforward_receive,

            lArm_Kp_receive,
            lArm_Kd_receive,
            rArm_Kp_receive,
            rArm_Kd_receive,

            servo_runin_mod,
            winlen
        );
    }
}

//函数3 -- 数据处理与转换函数
void ServoNode::outpTraj_process(){
    //获取轨迹输出解包
    runinTrajOutp trajOutp = runin_control_core.dataOutp(
        if_topic_point_position && !shutdown_mode_);

    //进行数据装入
    for(int i = 0; i < LARMDOF; i++){
        servo_desMsg_orig[i].transMod = lArm_transMod[i];

        servo_desMsg_orig[i].joint_pos = trajOutp.runinData.lArm_posDot[i];
        servo_desMsg_orig[i].joint_spd = trajOutp.runinData.lArm_velDot[i];

        servo_desMsg_orig[i].joint_tor = trajOutp.runinData.lArm_FeedForwardCompDot[i] + trajOutp.runinData.lArm_gravCompDot[i] + trajOutp.runinData.lArm_InteriaCompDot[i];
        
        servo_desMsg_orig[i].joint_Kp = trajOutp.lArmJnt_Kp[i];
        servo_desMsg_orig[i].joint_Kd = trajOutp.lArmJnt_Kd[i];
    }

    for(int i = 0; i < RARMDOF; i++){
        servo_desMsg_orig[i + LARMDOF].transMod = rArm_transMod[i];

        servo_desMsg_orig[i + LARMDOF].joint_pos = trajOutp.runinData.rArm_posDot[i];
        servo_desMsg_orig[i + LARMDOF].joint_spd = trajOutp.runinData.rArm_velDot[i];

        servo_desMsg_orig[i + LARMDOF].joint_tor = trajOutp.runinData.rArm_FeedForwardCompDot[i] + trajOutp.runinData.rArm_gravCompDot[i] + trajOutp.runinData.rArm_InteriaCompDot[i];

        servo_desMsg_orig[i + LARMDOF].joint_Kp = trajOutp.rArmJnt_Kp[i];
        servo_desMsg_orig[i + LARMDOF].joint_Kd = trajOutp.rArmJnt_Kd[i];
    }
}

//==================================================
//debug辅助函数

//debug函数1 -- 比对容器中直接获取的传输值与实际的传输值
void ServoNode::comp_desMsg_containerMsg(){
    std::vector<std::vector<joint_desMsg>> container_desMsg_ori = JntDesMsg_inContainer_Outp_v1();

    std::vector<joint_desMsg> container_desMsg_l, container_desMsg_r;

    for(int i = 0; i < container_desMsg_ori[1].size(); i++){
        container_desMsg_l.push_back(container_desMsg_ori[1][i]);
    }
    for(int i = 0; i < container_desMsg_ori[0].size(); i++){
        container_desMsg_l.push_back(container_desMsg_ori[0][i]);
    }

    for(int i = 0; i < container_desMsg_ori[3].size(); i++){
        container_desMsg_r.push_back(container_desMsg_ori[3][i]);
    }
    for(int i = 0; i < container_desMsg_ori[2].size(); i++){
        container_desMsg_r.push_back(container_desMsg_ori[2][i]);
    }

    //消息发送对比
    auto comp_pub = snowbot_ctl_msgs::msg::DbgInformation();
    for(int i = 0; i < 7; i++){
        comp_pub.float_reserve[i] = container_desMsg_l[i].joint_pos;
        comp_pub.float_reserve_b[i] = servo_desMsg_orig[i].joint_pos;
    }

    debugDataPub_comparison->publish(comp_pub);


}

// 夹爪控制回调 (data: [pos_rad, spd, tor], Kp/Kd跟手臂关节7)
void ServoNode::gripper_cmd_apply(const std_msgs::msg::Float64MultiArray::SharedPtr msg, bool is_left){
    if (shutdown_mode_) return;   // 关机回零期间忽略外部夹爪指令

    double pos = 0.0, spd = 0.0, tor = 0.0;   // 速度默认 0, 避免驱动器 KD 前馈产生位置偏差
    double kp = 100.0;   // 夹爪用中刚度, 不跟手臂J7
    double kd = 30.0;
    if(msg->data.size() >= 1) pos = msg->data[0];
    if(msg->data.size() >= 2) spd = msg->data[1];
    if(msg->data.size() >= 3) tor = msg->data[2];
    if(msg->data.size() >= 5){ kp = msg->data[3]; kd = msg->data[4]; }

    // 收到指令即使能夹爪电机 (与头/腰回调一致): 左夹爪=7, 右夹爪=15
    EtherCAT_setMotorEnable(true);
    EtherCAT_setSingleMotorEnable(is_left ? 7 : 15, true);

    // 诊断打印 (1Hz): 指令位置与电机反馈 (左右各自计数, 避免共享计数器只打印一侧)
    {
        static int dbg_cnt_l = 0, dbg_cnt_r = 0;
        int& dbg_cnt = is_left ? dbg_cnt_l : dbg_cnt_r;
        if (++dbg_cnt % 50 == 0) {
            printf("[Gripper] %s cmd_pos=%.3f spd=%.2f fb_pos=%.3f\n",
                   is_left ? "L" : "R", pos, spd, getGripperFbPos(is_left ? 0 : 1));
        }
    }

    double time_now = this->now().seconds() + this->now().nanoseconds() * 1e-9;

    if(is_left){
        g_left_pos_target = pos;
        if (!g_left_inited) {
            double fb = getGripperFbPos(0);
            g_left_pos_start = fb;
            g_left_pos_curr  = fb;
            g_left_inited    = true;
            g_left_time_last = time_now;
        } else {
            g_left_pos_start = g_left_pos_curr;
        }
        g_left_spd = spd; g_left_tor = tor; g_left_kp = kp; g_left_kd = kd;
        double dt = time_now - g_left_time_last;
        if (dt <= 0.0005) dt = 0.02;
        g_left_winlen = std::round(dt / 0.001);
        if(g_left_winlen < 2)   g_left_winlen = 2;
        if(g_left_winlen > 500) g_left_winlen = 500;
        g_left_time_last = time_now;
        g_left_step = 0;
        g_left_ifRunin = true;
    }else{
        g_right_pos_target = pos;
        if (!g_right_inited) {
            double fb = getGripperFbPos(1);
            g_right_pos_start = fb;
            g_right_pos_curr  = fb;
            g_right_inited    = true;
            g_right_time_last = time_now;
        } else {
            g_right_pos_start = g_right_pos_curr;
        }
        g_right_spd = spd; g_right_tor = tor; g_right_kp = kp; g_right_kd = kd;
        double dt = time_now - g_right_time_last;
        if (dt <= 0.0005) dt = 0.02;
        g_right_winlen = std::round(dt / 0.001);
        if(g_right_winlen < 2)   g_right_winlen = 2;
        if(g_right_winlen > 500) g_right_winlen = 500;
        g_right_time_last = time_now;
        g_right_step = 0;
        g_right_ifRunin = true;
    }
}

// 夹爪 1kHz 平滑插补 (Smoothstep，与躯干/头部逻辑完全对齐)
void ServoNode::gripper_runin(){
    if(g_left_ifRunin){
        g_left_step++;
        double u = (double)g_left_step / (double)g_left_winlen;
        if (u > 1.0) u = 1.0;
        double s = 3.0 * u * u - 2.0 * u * u * u;
        double pos = g_left_pos_start + (g_left_pos_target - g_left_pos_start) * s;
        g_left_pos_curr = pos;
        setLeftGripperCmd(pos, g_left_spd, g_left_tor, g_left_kp, g_left_kd);
        if(g_left_step >= g_left_winlen){
            g_left_ifRunin = false;
        }
    } else if (g_left_inited) {
        setLeftGripperCmd(g_left_pos_curr, g_left_spd, g_left_tor, g_left_kp, g_left_kd);
    }

    if(g_right_ifRunin){
        g_right_step++;
        double u = (double)g_right_step / (double)g_right_winlen;
        if (u > 1.0) u = 1.0;
        double s = 3.0 * u * u - 2.0 * u * u * u;
        double pos = g_right_pos_start + (g_right_pos_target - g_right_pos_start) * s;
        g_right_pos_curr = pos;
        setRightGripperCmd(pos, g_right_spd, g_right_tor, g_right_kp, g_right_kd);
        if(g_right_step >= g_right_winlen){
            g_right_ifRunin = false;
        }
    } else if (g_right_inited) {
        setRightGripperCmd(g_right_pos_curr, g_right_spd, g_right_tor, g_right_kp, g_right_kd);
    }
}

// 从 EtherCAT 读取六维力传感器并发布 WrenchStamped
void ServoNode::publish_sensor_data(){
    float fx, fy, fz, mx, my, mz;
    auto now = this->now();

    // 左传感器 (arm=0)
    getSensorData(0, fx, fy, fz, mx, my, mz);
    auto left_msg = geometry_msgs::msg::WrenchStamped();
    left_msg.header.stamp = now;
    left_msg.header.frame_id = "left_sensor";
    left_msg.wrench.force.x = fx; left_msg.wrench.force.y = fy; left_msg.wrench.force.z = fz;
    left_msg.wrench.torque.x = mx; left_msg.wrench.torque.y = my; left_msg.wrench.torque.z = mz;
    left_wrench_pub->publish(left_msg);

    // 右传感器 (arm=1)
    getSensorData(1, fx, fy, fz, mx, my, mz);
    auto right_msg = geometry_msgs::msg::WrenchStamped();
    right_msg.header.stamp = now;
    right_msg.header.frame_id = "right_sensor";
    right_msg.wrench.force.x = fx; right_msg.wrench.force.y = fy; right_msg.wrench.force.z = fz;
    right_msg.wrench.torque.x = mx; right_msg.wrench.torque.y = my; right_msg.wrench.torque.z = mz;
    right_wrench_pub->publish(right_msg);
}

// ========== 躯干/头部电机回调 & runin (纯位置模式) ==========

void ServoNode::torso_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    if (shutdown_mode_) return;   // 关机回零期间忽略外部指令
    // 支持纯位置格式 [pos0, pos1] 以及旧版 [pos,spd,tor,kp,kd] x 2
    if (msg->data.size() == 2) {
        t_pos_target[0] = msg->data[0];
        t_pos_target[1] = msg->data[1];
    } else if (msg->data.size() >= 10) {
        t_pos_target[0] = msg->data[0];
        t_pos_target[1] = msg->data[5];
    } else if (!msg->data.empty()) {
        for (size_t i = 0; i < msg->data.size() && i < TORSO_MOTORS; i++) {
            t_pos_target[i] = msg->data[i];
        }
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[TorsoCmd] Recv target: [%.4f, %.4f]", t_pos_target[0], t_pos_target[1]);

    // 确保仅开启躯干电机使能 (Slot 18~19)
    EtherCAT_setMotorEnable(true);
    EtherCAT_setSingleMotorEnable(18, true);
    EtherCAT_setSingleMotorEnable(19, true);

    double now = this->now().seconds() + this->now().nanoseconds() * 1e-9;

    // 起点处理：若首次收到指令，以当前实际反馈为起点；否则以上一次指令值/规划连续位置为起点，杜绝反馈滞后跳变！
    if (!t_inited) {
        for (int i = 0; i < TORSO_MOTORS; i++) {
            float pos, spd, cur; uint16_t st;
            getMotorFb_torso(i, pos, spd, cur, st);
            t_pos_start[i] = pos;
            t_pos_curr[i]  = pos;
        }
        t_inited = true;
        t_time_last = now;
    } else {
        for (int i = 0; i < TORSO_MOTORS; i++) {
            t_pos_start[i] = t_pos_curr[i];
        }
    }

    // 底层控制周期为 1ms (0.001s)，计算插补步数
    double dt = now - t_time_last;
    if (dt <= 0.0005) dt = 0.02; // 默认 50Hz 周期
    t_winlen = std::round(dt / 0.001);
    if (t_winlen < 2) t_winlen = 2;
    if (t_winlen > 500) t_winlen = 500;

    t_time_last = now;
    t_step = 0;
    t_ifRunin = true;
}

void ServoNode::head_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    if (shutdown_mode_) return;   // 关机回零期间忽略外部指令
    if (msg->data.size() == 2) {
        h_pos_target[0] = msg->data[0];
        h_pos_target[1] = msg->data[1];
    } else if (msg->data.size() >= 10) {
        h_pos_target[0] = msg->data[0];
        h_pos_target[1] = msg->data[5];
    } else if (!msg->data.empty()) {
        for (size_t i = 0; i < msg->data.size() && i < HEAD_MOTORS; i++) {
            h_pos_target[i] = msg->data[i];
        }
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[HeadCmd] Recv target: [%.4f, %.4f]", h_pos_target[0], h_pos_target[1]);

    // 确保仅开启头部电机使能 (Slot 16~17)
    EtherCAT_setMotorEnable(true);
    EtherCAT_setSingleMotorEnable(16, true);
    EtherCAT_setSingleMotorEnable(17, true);

    double now = this->now().seconds() + this->now().nanoseconds() * 1e-9;

    // 起点处理：若首次收到指令，以当前实际反馈为起点；否则以上一次指令值/规划连续位置为起点，杜绝反馈滞后跳变！
    if (!h_inited) {
        for (int i = 0; i < HEAD_MOTORS; i++) {
            float pos, spd, cur; uint16_t st;
            getMotorFb_head(i, pos, spd, cur, st);
            h_pos_start[i] = pos;
            h_pos_curr[i]  = pos;
        }
        h_inited = true;
        h_time_last = now;
    } else {
        for (int i = 0; i < HEAD_MOTORS; i++) {
            h_pos_start[i] = h_pos_curr[i];
        }
    }

    // 底层控制周期为 1ms (0.001s)，计算插补步数
    double dt = now - h_time_last;
    if (dt <= 0.0005) dt = 0.02; // 默认 50Hz 周期
    h_winlen = std::round(dt / 0.001);
    if (h_winlen < 2) h_winlen = 2;
    if (h_winlen > 500) h_winlen = 500;

    h_time_last = now;
    h_step = 0;
    h_ifRunin = true;
}

void ServoNode::torso_runin(){
    if (!t_ifRunin) {
        // 保持当前指令位置
        for (int i = 0; i < TORSO_MOTORS; i++) {
            setTorsoMotorCmd(i, t_pos_curr[i], 0.0, 0.0, 0.0, 0.0);
        }
        return;
    }

    t_step++;
    // 使用余弦/三次平滑插补 (Smoothstep: 3*u^2 - 2*u^3)，速度在两端平滑为0，无加速度阶跃冲击
    double u = (double)t_step / (double)t_winlen;
    if (u > 1.0) u = 1.0;
    double s = 3.0 * u * u - 2.0 * u * u * u;

    for (int i = 0; i < TORSO_MOTORS; i++) {
        double pos = t_pos_start[i] + (t_pos_target[i] - t_pos_start[i]) * s;
        t_pos_curr[i] = pos;
        setTorsoMotorCmd(i, pos, 0.0, 0.0, 0.0, 0.0);
    }
    if (t_step >= t_winlen) t_ifRunin = false;
}

void ServoNode::head_runin(){
    if (!h_ifRunin) {
        // 保持当前指令位置
        for (int i = 0; i < HEAD_MOTORS; i++) {
            setHeadMotorCmd(i, h_pos_curr[i], 0.0, 0.0, 0.0, 0.0);
        }
        return;
    }

    h_step++;
    // 使用余弦/三次平滑插补 (Smoothstep: 3*u^2 - 2*u^3)，速度在两端平滑为0，无加速度阶跃冲击
    double u = (double)h_step / (double)h_winlen;
    if (u > 1.0) u = 1.0;
    double s = 3.0 * u * u - 2.0 * u * u * u;

    for (int i = 0; i < HEAD_MOTORS; i++) {
        double pos = h_pos_start[i] + (h_pos_target[i] - h_pos_start[i]) * s;
        h_pos_curr[i] = pos;
        setHeadMotorCmd(i, pos, 0.0, 0.0, 0.0, 0.0);
    }
    if (h_step >= h_winlen) h_ifRunin = false;
}

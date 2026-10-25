// 移植自 ros2_impedance_controller（qleonardolp, Apache 2.0），去除 ros2_control 框架，
// 适配 servo_control 主循环直接驱动 EtherCAT 的架构。
// 修改要点:
//   - 不继承 controller_interface::ControllerInterface
//   - 关节状态由外部 update() 传入，而非从 LoanedStateInterface 读取
//   - 力矩命令通过 getEffortCommands() 由调用方读取，而非写入 LoanedCommandInterface
//   - 支持完整 q（含躯干/头部/双臂/夹爪）输入，但仅控制指定子集关节
#ifndef SNOWBOT_IMPEDANCE__CARTESIAN_IMPEDANCE_CONTROLLER_H
#define SNOWBOT_IMPEDANCE__CARTESIAN_IMPEDANCE_CONTROLLER_H

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "snowbot_ctl_msgs/msg/cartesian_impedance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace snowbot_impedance {

constexpr uint8_t kCartesianDim = 6;
constexpr double kDampingRatio = 1.0;
constexpr uint8_t kMaxJointSpaceDim = 48;

using DiagonalMatrix6d = Eigen::DiagonalMatrix<double, kCartesianDim>;
using Vector6d = Eigen::Matrix<double, kCartesianDim, 1>;
using Matrix6d = Eigen::Matrix<double, kCartesianDim, kCartesianDim>;
using Matrix6Xd =
    Eigen::Matrix<double, kCartesianDim, Eigen::Dynamic, 0, kCartesianDim, kMaxJointSpaceDim>;
using VectorXd =
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxJointSpaceDim, 1>;
using ReferenceType = snowbot_ctl_msgs::msg::CartesianImpedance;

// ============================================================================
// LuGre 动态摩擦模型 (单关节实例, 参考 joint_low_speed_control 实现)
//   描述预滑动+滑动区摩擦, 含 Stribeck 效应, 用于关节摩擦前馈补偿
//   数学: F = sigma0*z + sigma1*dz + sigma2*v
//         dz = v - sigma0*|v|/g(v)*z
//         g(v) = Fc + (Fs-Fc)*exp(-(v/vs)^2)
// ============================================================================
class LuGreFrictionJoint {
public:
  struct Params {
    double sigma0{1.0e5};   // bristle 刚度 [N·m/rad]
    double sigma1{300.0};   // bristle 阻尼 [N·m·s/rad]
    double sigma2{2.0};    // 粘性摩擦 [N·m·s/rad]
    double Fs{1.5};         // 静摩擦力矩 [N·m]
    double Fc{1.0};         // 库仑摩擦力矩 [N·m]
    double vs{0.01};        // Stribeck 速度 [rad/s]
  };

  explicit LuGreFrictionJoint(const Params& p);
  LuGreFrictionJoint() : p_(), z_(0.0) {}
  void reset() { z_ = 0.0; }
  void setParams(const Params& p) { p_ = p; z_ = 0.0; }
  const Params& params() const { return p_; }

  // 单步更新, 输入关节速度 v [rad/s] 和采样周期 dt [s], 返回摩擦力矩 [N·m]
  double step(double v, double dt);

private:
  double stribeck_(double v) const;
  Params p_;
  double z_;  // bristle 平均变形状态 (内部积分)
};

inline LuGreFrictionJoint::LuGreFrictionJoint(const Params& p) : p_(p), z_(0.0) {}

inline double LuGreFrictionJoint::step(double v, double dt) {
  double g_v = stribeck_(v);
  double dz_dt;
  if (std::abs(g_v) < 1e-9) {
    dz_dt = v;
  } else {
    dz_dt = v - p_.sigma0 * (std::abs(v) / g_v) * z_;
  }
  // 限幅防 bristle 状态发散 (Fs/sigma0 的 1.5 倍)
  double z_ss_max = (g_v > 1e-9) ? (p_.Fs / p_.sigma0 * 1.5) : 0.01;
  double z_new = z_ + dz_dt * dt;
  if (z_new >  z_ss_max) z_new =  z_ss_max;
  if (z_new < -z_ss_max) z_new = -z_ss_max;
  z_ = z_new;
  return p_.sigma0 * z_ + p_.sigma1 * dz_dt + p_.sigma2 * v;
}

inline double LuGreFrictionJoint::stribeck_(double v) const {
  double v_safe = (std::abs(p_.vs) < 1e-9) ? 1e-9 : p_.vs;
  double s = v / v_safe;
  return p_.Fc + (p_.Fs - p_.Fc) * std::exp(-s * s);
}

struct ImpedanceConfig {
  std::string urdf_path;             // URDF 文件绝对路径
  std::string ee_link;               // 末端 link 名 (arm_x_link_7)
  std::vector<std::string> controlled_joint_names;  // 该臂控制的关节名 (7 个)
  std::string ft_sensor_topic;       // F/T 传感器 topic (可选, 留空则不订阅)

  std::vector<double> stiffness;    // [N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad]
  std::vector<double> damping;      // 当 taskspace_mass=0 时使用
  double taskspace_mass{10.0};      // >0 启用惯量整形, =0 仅阻尼
  double pinv_damping{0.02};        // 雅可比阻尼伪逆的 λ (Tikhonov), 0=精确伪逆
  double cmd_lpf_alpha{0.7585469929947761};  // 力矩低通滤波系数
  double acc_lpf_alpha{0.4399008464884426};  // 加速度低通滤波系数
  bool has_effort_states{false};    // 是否有真实关节力矩反馈

  // 末端工具重力补偿参数
  //   tool_mass:       工具质量 [kg], >0 启用实时重力补偿, =0 不补偿
  //   tool_com_in_tcp: 工具质心在 TCP(夹爪中心) 坐标系下的位置 [x, y, z] [m]
  //                    用于计算重力产生的力矩 M = r × F
  // 实时补偿公式 (在基坐标系下):
  //   F_g_B = [0, 0, -m*g, 0, 0, 0]                (基系下纯重力, 恒定)
  //   r_tool_B = R_ee_in_base * (R_ee_tcp * tool_com_in_tcp + t_ee_tcp)
  //                                                 (质心在基系下的位置)
  //   M_g_B = r_tool_B × F_g_B.head<3>()            (绕 ee 原点的重力矩, 基系下表示)
  //   wrench_in_base -= [F_g_B; M_g_B]
  double tool_mass{0.0};
  std::vector<double> tool_com_in_tcp{0.0, 0.0, 0.0};

  // 末端几何变换 (相对 ee_link 原点的静态标定参数)
  //   sensor_to_ee: 传感器坐标系相对 ee_link 的位姿 [x, y, z, qx, qy, qz, qw]
  //                 用于把 sensor wrench 从传感器自身系变换到 base 系
  //   tcp_to_ee:    TCP(夹爪中心) 坐标系相对 ee_link 的位姿 [x, y, z, qx, qy, qz, qw]
  //                 用于把 FK 得到的 ee_link 位姿变换到 TCP, 与上位机期望位姿对齐
  //   默认为单位变换 (与 ee_link 重合), 向后兼容旧 yaml 不配置的情况
  std::vector<double> sensor_to_ee{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  std::vector<double> tcp_to_ee{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  // 关节力矩安全限幅 [Nm] (7 维, 与 controlled_joint_names 对应)
  //   - 标量: 7 个关节统一限幅 (向下兼容旧 yaml)
  //   - 7 维数组: 每个关节单独限幅, 适合各关节电机额定扭矩不同的场景
  //   updateEffortCommands 输出前会按此值做对称限幅: |tau| <= effort_limit
  //   缺省值 5.0 Nm 是保守安全限幅, 实际应按电机额定扭矩配置
  //   (参考: 视臂关节 1-4 额定约 30 Nm, 关节 5-7 额定约 9 Nm)
  std::vector<double> effort_limit_per_joint{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};

  // 重力补偿系数 (逐关节缩放 pinocchio 计算的重力项)
  //   g_sub_(i) = kt_gravity[i] * robot_data_->g(controlled_v_idx_[i])
  //   1.0=完全补偿, 0.0=不补偿, >1.0=过补偿, <1.0=欠补偿
  //   用途: URDF 质量参数不准 / 关节摩擦 / 连杆公差时逐关节微调
  std::vector<double> kt_gravity{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // LuGre 动态摩擦补偿参数 (参考 joint_low_speed_control)
  //   enable=false 时禁用摩擦补偿, tau_desired_ 不含摩擦前馈
  //   enable/sigma0/sigma1/sigma2 均可为标量 (7 关节统一) 或 7 维数组 (逐关节)
  //   Fs/Fc/vs 为 7 维数组 (逐关节)
  //   dq_lpf_alpha: 输入速度低通滤波系数 (0~1, 越小越平滑但延迟大)
  struct LuGreFrictionConfig {
    std::vector<double> enable{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   // 逐关节开关, >0.5 启用
    std::vector<double> sigma0{1.0e5, 1.0e5, 1.0e5, 1.0e5, 1.0e5, 1.0e5, 1.0e5};
    std::vector<double> sigma1{300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0};
    std::vector<double> sigma2{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    std::vector<double> Fs{1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
    std::vector<double> Fc{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<double> vs{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    double dq_lpf_alpha{0.3};
  };
  LuGreFrictionConfig lugre_friction;

  // 零空间控制参数 (冗余自由度调节, 投影到 J^T 零空间, 不影响末端跟踪)
  //   nullspace_stiffness: 标量 [Nm/rad], 0=禁用零空间控制
  //   nullspace_damping:   标量 [Nm*s/rad]
  //   注: 零空间期望构型在使能时自动捕获当前关节角作为初始值,
  //       上位机可通过 q_d_nullspace 话题覆盖.
  double nullspace_stiffness{0.0};
  double nullspace_damping{2.0};

  // 静差消除参数
  //   integral_gain_joint: 关节空间积分增益 (标量), 对常值扰动零静差
  //     关节空间泄漏积分: d(q_accum)/dt = J^#*dx - σ*q_accum
  //     tau_integral = Kq * q_accum (直接关节力矩, 绕过 J^T, 无运动耦合)
  //     仅在自由空间 (sensor_wrench < integral_force_threshold) 时累积积分,
  //     末端受力时清零积分器, 避免消除力致位姿偏差
  //   integral_force_threshold: 6 维 [N] / [Nm], 判定自由空间的力阈值
  //     所有维度均 < threshold 时积分开启, 任一维度超阈值时清零 (末端受力)
  //   force_feedback_gain: 末端外力闭环增益 [6 维], 基于 F/T 传感器
  //     F_ff = Kf * (0 - sensor_wrench_at_tcp) = -Kf * F_ext
  //     当 sensor_wrench 残留偏差 (静摩擦/重力残差) 时, 闭环修正期望力
  std::vector<double> integral_gain_joint{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Kq, 关节空间积分增益 (7 维)
  std::vector<double> integral_limit_joint{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};  // 关节空间积分抗饱和限幅 [Nm] (7 维)
  std::vector<double> integral_force_threshold{3.0, 3.0, 3.0, 0.5, 0.5, 0.5};  // 自由空间判定阈值
  double integral_leakage{1.0};  // 泄漏系数 σ (σ>0, 越大越稳定但静差抑制弱)
  // 非线性增益 (小误差放大): gain_boost = 1 + alpha * exp(-|dq_err|/sigma)
  //   alpha: 最大放大倍数 (误差→0 时增益放大 alpha 倍)
  //   sigma: 过渡尺度 [rad], 误差 << sigma 时满增益, >> sigma 时无放大
  double integral_gain_alpha{2.0};   // 非线性增益放大系数
  double integral_gain_sigma{0.02};  // 非线性增益过渡尺度 [rad]
  std::vector<double> force_feedback_gain{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double force_ff_lpf_alpha{0.1};  // 力闭环低通滤波 (0~1, 越小越平滑)

  // 六维力传感器系统偏差补偿 [fx, fy, fz, mx, my, mz]
  //   原始读数 wrench_compensated = wrench_raw - sensor_offset
  //   用于消除传感器零漂/安装偏差等系统性误差
  std::vector<double> sensor_offset{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // 参考信号滤波参数 (匹配上位机低频下发与 1kHz 控制周期)
  //   取值 0~1, 1=不滤波, 越小越平滑但跟随延迟越大
  //   实际滤波步长 step = 1 / (kappa * f + 1), kappa = -1/log(1-p)
  double filter_params_pose{0.95};        // 末端位姿滤波
  double filter_params_nullspace{0.95};  // 零空间构型滤波
  double update_frequency{1000.0};       // 控制频率 [Hz]

  /// 默认配置 (阻尼模式, 中等刚度).
  /// @param side "left" 或 "right", 用于确定 ee_link 和关节名前缀.
  /// @param urdf_path URDF 文件绝对路径.
  static ImpedanceConfig makeDefault(const std::string& side,
                                      const std::string& urdf_path);

  /// 从 yaml 文件加载配置.
  ///
  /// yaml 格式:
  ///   impedance:
  ///     left:
  ///       stiffness: [200, 200, 200, 20, 20, 20]
  ///       damping:    [20, 20, 20, 2, 2, 2]
  ///       taskspace_mass: 0.0
  ///       cmd_lpf_alpha: 0.4
  ///       acc_lpf_alpha: 0.2
  ///       has_effort_states: false
  ///     right:
  ///       ... (同上)
  ///
  /// @param yaml_path yaml 文件绝对路径
  /// @param side "left" 或 "right"
  /// @param urdf_path URDF 文件绝对路径 (yaml 中也可覆盖)
  /// @return 加载成功的配置; 加载失败时返回 makeDefault() 并打印警告.
  static ImpedanceConfig fromYaml(const std::string& yaml_path,
                                   const std::string& side,
                                   const std::string& urdf_path);
};

class CartesianImpedanceController {
 public:
  CartesianImpedanceController() = default;
  ~CartesianImpedanceController() = default;

  /// 初始化: 解析 URDF, 配置参数, 创建订阅/发布.
  /// node 用于参数读取和 ROS 接口, 完整生命周期内有效.
  bool init(rclcpp::Node::SharedPtr node, const ImpedanceConfig& config);

  /// 每个控制周期调用一次.
  /// q_full: 完整关节位置 (含躯干/头部/双臂/夹爪, 维度 = model.nq)
  /// dq_full: 完整关节速度
  /// efforts_full: 完整关节力矩反馈 (无反馈时传 0 向量)
  /// sensor_wrench: 末端传感器力 [Fx..Mz], **传感器自身坐标系下表示**
  ///                (内部会自动转换到基坐标系并减去使能时刻的偏置)
  /// dt: 控制周期 (秒)
  void update(const VectorXd& q_full, const VectorXd& dq_full,
              const VectorXd& efforts_full, const Vector6d& sensor_wrench,
              double dt);

  /// 获取该臂的关节力矩命令 (维度 = n_dof_)
  const VectorXd& getEffortCommands() const { return effort_commands_; }

  /// 设置前馈力矩覆盖值 (tau_ff 测试模式)
  ///   调用方 (rosLoop.cpp) 在 tau_ff 模式下传入实际施加的关节力矩,
  ///   publishStatus() 会将其追加到 effort_msg_.data 末尾发布, 便于 PlotJuggler 监控.
  ///   维度必须 = n_dof_, 全零表示阻抗模式 (仅 effort_commands_ 生效)
  void setTorqueFF(const VectorXd& tau_ff) {
    if (tau_ff.size() == n_dof_) tau_ff_applied_ = tau_ff;
  }
  // 使能时刻采样的重力补偿 (用于 EtherCAT CiA402 使能序列预注入, 方案 E)
  const VectorXd& getGravityAtEnable() const { return g_sub_at_enable_; }

  /// 实时计算重力补偿力矩 (n_dof_), 内部调用 updateRobot() + computeAllTerms.
  ///   用于阻抗未使能时也能向 EtherCAT 驱动器注入重力前馈,
  ///   避免 CiA402 使能序列期间 (enable_step < EN_STEP*6) 抱闸打开后机械臂下掉.
  ///   q_full/dq_full: 完整关节状态 (rosLoop 构造).
  const VectorXd& computeGravityLive(const VectorXd& q_full,
                                       const VectorXd& dq_full);

  /// 获取该臂末端位姿偏差 (6 维, 用于调试/上限检测)
  const Vector6d& getPoseDeviation() const { return pose_deviation_; }
  const Vector6d& getTwistDeviation() const { return twist_deviation_; }

  /// 获取当前 TCP (夹爪中心) 在 base 系下的位姿 (Affine3d).
  ///   阻抗控制的位姿跟踪对象是 TCP, 不是 ee_link -- 因为 ee_link → TCP
  ///   有标定偏移 t_ee_in_tcp_. 双臂协调 (DualArmCoordinator) 也用 TCP
  ///   作为夹持接触点.
  Eigen::Affine3d getTcpPoseInBase() const;

  /// 获取当前 TCP 在 base 系下的实际空间速度 [vx vy vz ωx ωy ωz]'.
  ///   由 J(q) * dq 计算, 双臂协调层用主臂此值推导从臂期望速度.
  const Vector6d& getActualTwist() const { return actual_twist_; }

  /// 获取当前 TCP 坐标系下的传感器 wrench (基坐标系表达, 参考点 = TCP 原点).
  ///   双臂协调层用于内力反馈计算: F_internal = 0.5*(-F_left - (-F_right)).
  const Vector6d& getSensorWrenchAtTcp() const { return sensor_wrench_at_tcp_; }

  /// 获取当前控制器内部实际使用的 impedance wrench (可能是 K*dx+D*dv,
  /// 也可能是外部注入的值). 仅用于诊断.
  const Vector6d& getImpedanceWrench() const { return impedance_wrench_; }

  /// 向阻抗环注入外部 Cartesian wrench (base 系, 参考点 = TCP 原点).
  ///   注入后 update() 内部将直接使用此值替代 K*dx+D*dv 的计算结果.
  ///   阶段 2 物体级阻抗模式下由 DualArmCoordinator 调用.
  void setExternalWrench(const Vector6d& F_ext);

  /// 清除外部 wrench 注入, 恢复为正常 K*dx+D*dv 内部计算.
  void clearExternalWrench();

  /// 是否处于外部 wrench 注入模式.
  bool hasExternalWrench() const { return has_external_wrench_; }

  /// 获取该臂零空间期望关节角 (滤波后, 维度 = n_dof_)
  ///   力位混合控制注入位置环时使用此值作为期望关节角,
  ///   让位置环跟踪上位机/默认构型, 而不是把当前反馈当作目标 (避免漂移累积)
  const VectorXd& getQDNullspaceFiltered() const { return q_d_nullspace_filtered_; }
  const Vector6d& getSensorOffset() const { return sensor_offset_; }

  bool isInitialized() const { return initialized_; }
  size_t getDof() const { return n_dof_; }

  /// 返回该臂 7 个控制关节在 pinocchio q_full 中的索引列表.
  ///   调用方 (rosLoop.cpp 的 applyImpedanceControl) 必须使用这些索引填充
  ///   q_full / dq_full, 而不是用硬编码的 URDF 顺序索引 -- 因为 pinocchio
  ///   buildModel 走 BFS, 在 body_link_2 处会交错排列 head/arm_r/arm_l 子树,
  ///   导致实际 idx_qs 与 URDF 文档顺序不同.
  const std::vector<int>& getControlledQIdx() const { return controlled_q_idx_; }
  const std::vector<int>& getControlledVIdx() const { return controlled_v_idx_; }

  /// 按关节名查询 pinocchio q_full / v 索引 (供 rosLoop 填充非控制关节如
  /// 躯干/头部/夹爪使用). 关节不存在时返回 -1.
  int getJointQIdx(const std::string& joint_name) const;
  int getJointVIdx(const std::string& joint_name) const;

  /// 返回 pinocchio model.nq (完整 q_full 维度, 含所有 revolute 关节).
  size_t getNqFull() const { return n_q_full_; }

  /// 使能时刻的初始化: 捕获当前末端位姿作为默认参考, 当前 sensor
  /// 读数作为重力/工具偏置 (在基坐标系下减去, 抵消末端工具重力).
  /// 必须在每个上升沿 (false -> true) 调用一次, 在第一次 update() 之前调用.
  /// q_full / dq_full: 使能时刻的完整关节状态
  /// sensor_wrench: 使能时刻的传感器读数 (传感器坐标系)
  void enable(const VectorXd& q_full, const VectorXd& dq_full,
              const Vector6d& sensor_wrench);

  /// 失能时清理: 清除 bias 和外部参考, 下次使能必须重新调用 enable().
  void disable();

  /// 外部主动设置参考 (同步, 用于 hold 模式或紧急停止)
  void setReference(const ReferenceType& ref);
  /// 便捷重载: 直接用 Affine3d 设置末端位姿参考.
  ///   twist: 若传入非空则作为期望速度; 否则置零.
  ///   accel: 始终置零 (协调层只关心位姿和速度).
  ///   nullspace: 根据 T_desired 通过 IK 求关节解作为零空间期望 (用 q_ik_last_ 做 seed).
  void setReferencePose(const Eigen::Affine3d& T_desired,
                        const Vector6d* twist_desired = nullptr);
  void clearReference();

 private:
  /// 增量式数值 IK (阻尼伪逆迭代).
  ///   给定末端目标位姿 T_desired, 在 q_seed 附近迭代收敛到关节解.
  ///   每帧目标连续时只需 2-3 次迭代即可收敛, 适合 1kHz RT 线程.
  ///   q_out: IK 结果 (维度 = n_dof_).
  ///   seed: 初始猜测 (维度 = n_dof_), 一般用上一帧 IK 解实现零空间连续性.
  bool computeIk(const Eigen::Affine3d& T_desired,
                 const VectorXd& q_seed,
                 VectorXd& q_out);
  bool configureRobotModel();
  void updateRobot();
  void updateDeviationAndReference();
  void updateEffortCommands();
  void computeHamiltonian();
  void publishStatus();
  /// 计算末端工具重力在基坐标系下的 [F; M] (绕 ee_link 原点的力矩).
  /// 调用前需先 updateRobot() 让 ee_rotation_in_base_ 更新.
  Vector6d computeToolGravityInBase() const;
  /// 参考位姿滤波 (target -> filtered), 移植自参考实现 updateFilteredPose.
  void updateFilteredReference();
  /// 参考信号滤波步长 (1 阶低通, 时间常数 = 1/(kappa*f))
  ///   step = 1 / (kappa * f + 1), kappa = -1/log(1-p)
  static double computeFilterStep(double filter_percent, double frequency);
  /// 1 阶低通: out = (1-step)*out + step*target
  static inline double filteredUpdate(double target, double current, double step);
  /// 把 [x, y, z, qx, qy, qz, qw] 解析为 Eigen::Isometry3d
  ///   用于 sensor_to_ee / tcp_to_ee 等几何变换. 长度必须为 7, 否则返回单位变换.
  static Eigen::Isometry3d parsePose(const std::vector<double>& pose7);

  /// 把 ee_link Jacobian (LOCAL_WORLD_ALIGNED, 6×n) 变换到 TCP Jacobian
  ///   v_tcp = v_ee - skew(r) * w_ee,  其中 r = R_ee_base * t_ee_in_tcp
  ///   => J_tcp.topRows(3) -= skew(r) * J.bottomRows(3); J.bottomRows(3) 不变
  ///   若 t_ee_in_tcp_ 范数 < 1e-9, 视为无偏移, 直接返回 (向后兼容)
  ///   注: 对 jacobian_dt_ 同样适用 (近似, 忽略 dT/dt 项)
  void applyTcpOffsetToJacobian(Matrix6Xd& J) const;

  ImpedanceConfig config_;
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};
  bool diag_params_printed_{false};  // 仅首帧打印 K/D 诊断信息
  Vector6d lambda_diag_{Vector6d::Zero()};      // 操作空间惯量矩阵 Lambda 对角元 (诊断用)
  int64_t last_diag_ns_{0};                     // 每秒诊断打印计时 (每实例独立, 0=未初始化)
  int64_t ns_diag_ns_{0};                       // 零空间诊断打印计时 (每实例独立)

  // Pinocchio
  pinocchio::Model robot_model_;
  std::shared_ptr<pinocchio::Data> robot_data_;
  pinocchio::FrameIndex ee_frame_{0};
  std::vector<int> controlled_q_idx_;  // 控制关节在 q_full 中的索引
  std::vector<int> controlled_v_idx_;  // 控制关节在 v (nv) 中的索引
  size_t n_dof_{0};
  size_t n_q_full_{0};

  // ROS 接口
  rclcpp::Subscription<ReferenceType>::SharedPtr reference_sub_;
  std::mutex rt_reference_mutex_;       // 保护 rt_reference_ (低频写入, 1kHz 读取)
  ReferenceType rt_reference_;
  std::atomic<bool> rt_reference_valid_{false};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  std::mutex status_pub_mutex_;         // 发布互斥 (try_lock 失败则跳过)
  std_msgs::msg::Float64MultiArray status_msg_;
  // effort_commands_ 单独话题发布 (避免主 status 数组过长)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
  std::mutex effort_pub_mutex_;
  std_msgs::msg::Float64MultiArray effort_msg_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr ft_sub_;

  // 阻抗参数
  DiagonalMatrix6d desired_stiffness_;
  DiagonalMatrix6d desired_damping_;
  DiagonalMatrix6d desired_inertia_inv_;
  Matrix6d desired_inertia_;
  Matrix6d actual_inertia_;
  bool inertia_shaping_{false};
  double cmd_lpf_alpha_{0.7585469929947761};
  double acc_lpf_alpha_{0.4399008464884426};
  double delta_t_{0.001};
  bool has_effort_states_{false};

  // 完整状态缓存 (供 updateRobot()/updateDeviationAndReference() 访问)
  VectorXd q_full_cache_;
  VectorXd dq_full_cache_;
  VectorXd efforts_full_cache_;

  // 状态 (仅控制关节子集)
  VectorXd robot_q_;       // n_dof_
  VectorXd robot_dq_;
  VectorXd robot_efforts_;
  VectorXd effort_commands_;

  // 雅克比
  Matrix6Xd jacobian_full_;  // (6, n_q_full_) 来自 pinocchio
  Matrix6Xd jacobian_;       // (6, n_dof_)   控制关节子集
  Matrix6Xd jacobian_dt_;    // (6, n_dof_)

  // 使能时刻捕获的偏置/默认参考
  // 1) sensor bias 在基坐标系下表示 (enable 时刻已扣除工具重力后的残余偏置)
  //    反映传感器零漂 + 未建模的固定偏置, 后续 update() 中实时减去
  bool   has_sensor_bias_{false};
  Vector6d sensor_wrench_bias_in_base_;
  // 2) 默认参考位姿 (使能时刻的末端位姿), 无外部参考时使用
  Eigen::Vector3d    default_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond default_quaternion_{Eigen::Quaterniond::Identity()};
  // 末端 frame 在基坐标系下的旋转 (用于 sensor wrench 坐标系变换)
  Eigen::Matrix3d    ee_rotation_in_base_{Eigen::Matrix3d::Identity()};

  // 末端工具重力补偿参数 (从 config 加载)
  double tool_mass_{0.0};
  Eigen::Vector3d tool_com_in_ee_{Eigen::Vector3d::Zero()};
  static constexpr double kGravityAccel = 9.81;

  // 末端几何变换 (init 时一次性预计算, 实时线程内只读取)
  //   T_ee_sensor_:  sensor 系相对 ee 系 (用于 sensor wrench 旋转 + 平移修正)
  //   T_ee_tcp_:     TCP 系相对 ee 系 (用于 FK 位姿变换到 TCP)
  //   R_sensor_to_ee_, t_sensor_in_ee_: 旋转/平移缓存, 避免重复取
  //   R_ee_to_tcp_, t_ee_in_tcp_:        TCP 表达用缓存
  // 默认为单位变换, 与 ee_link 重合 (向后兼容)
  Eigen::Isometry3d T_ee_sensor_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_ee_tcp_{Eigen::Isometry3d::Identity()};
  Eigen::Matrix3d   R_sensor_to_ee_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d  t_sensor_in_ee_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d   R_ee_to_tcp_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d  t_ee_in_tcp_{Eigen::Vector3d::Zero()};

  // 关节力矩限幅 (从 config 加载, 维度 = n_dof_)
  //   限幅前若 tau(i) 超过此值, 截断到 ±effort_limit_per_joint_(i)
  //   实时线程内执行, 无锁
  Eigen::VectorXd effort_limit_per_joint_;

  // 重力补偿系数 (逐关节缩放, 维度 = n_dof_)
  //   g_sub_(i) = kt_gravity_(i) * robot_data_->g(controlled_v_idx_[i])
  //   用途: 当 pinocchio 模型与实物存在偏差时逐关节微调
  Eigen::VectorXd kt_gravity_;

  // 零空间控制参数
  double nullspace_stiffness_{0.0};
  double nullspace_damping_{2.0};
  // 零空间期望构型: target (上位机下发的最新值), filtered (滤波后的平滑值)
  //   注意: 这里必须用本文件 using 定义的 VectorXd (Matrix<double, Dynamic, 1, 0,
  //   kMaxJointSpaceDim, 1>) 而非 Eigen::VectorXd, 否则与 getter 返回类型不匹配,
  //   编译器会做隐式转换构造临时对象, 返回临时对象的引用 => 悬空引用 => 段错误
  VectorXd q_d_nullspace_target_;
  VectorXd q_d_nullspace_filtered_;
  bool   has_nullspace_target_{false};

  // IK 迭代参数
  int    ik_max_iter_{3};
  double ik_pos_tol_{1e-4};   // [m]
  double ik_rot_tol_{1e-3};  // [rad]
  VectorXd q_ik_last_;       // 上一帧 IK 解, 作为 seed 保证零空间连续性

  // 参考信号滤波参数
  double filter_params_pose_{0.95};
  double filter_params_nullspace_{0.95};
  double update_frequency_{1000.0};
  // 末端位姿参考: target (上位机最新值), filtered (滤波后平滑值, 用于控制)
  //   注意: default_position_/default_quaternion_ 是使能时捕获的 fallback
  //         当上位机从未下发参考时, target 不会被更新, filtered 追随 default
  Eigen::Vector3d    position_d_target_;
  Eigen::Vector3d    position_d_filtered_;
  Eigen::Quaterniond orientation_d_target_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond orientation_d_filtered_{Eigen::Quaterniond::Identity()};
  // 末端速度/加速度参考: 同样采用 target -> filtered 1 阶低通滤波
  //   (与 position_d_target_ 同步处理, 避免位姿滤波后 twist/accel 仍跳变)
  Vector6d desired_twist_target_;
  Vector6d desired_twist_filtered_;
  Vector6d desired_pose_accel_target_;
  Vector6d desired_pose_accel_filtered_;
  bool   has_pose_target_{false};

  // 零空间力矩中间量 (n_dof_)
  VectorXd nullspace_torques_;

  // 中间量
  // jacobian_pinv_: (n_dof_, 6)  = pinv(J)  J 是 (6, n_dof_)
  // jacobianT_pinv_: (n_dof_, n_dof_) = inv(J^T)  J^T 是 (n_dof_, 6) 仅当 n_dof_==6 才方阵;
  //   对 7DoF 冗余臂 J^T 是 7x6 不可逆, 上面 completeOrthogonalDecomposition 已处理
  //   这里两个矩阵声明为动态大小, 避免固定 6x6 在 n_dof_=7 时触发 Eigen resize 断言
  Eigen::MatrixXd jacobian_pinv_;
  Eigen::MatrixXd jacobianT_pinv_;
  Eigen::MatrixXd jsim_jpinv_;      // (n_dof_, 6)
  Eigen::MatrixXd jsim_jpinv_dj_;   // (n_dof_, n_dof_)
  VectorXd tau_desired_;
  VectorXd twist_compensation_;
  VectorXd accel_feedforward_;
  VectorXd impedance_torques_;
  VectorXd g_sub_;                  // 重力补偿项 (n_dof_, pinocchio::rnea 中的 g)
  VectorXd g_sub_at_enable_;         // 使能时刻采样的重力补偿 (n_dof_, 用于 EtherCAT CiA402 使能预注入)
  VectorXd g_sub_live_;              // 实时重力补偿 (n_dof_, computeGravityLive 返回)
  VectorXd friction_torques_;        // LuGre 摩擦前馈力矩 (n_dof_, 调试用)
  VectorXd tau_ff_applied_;           // tau_ff 测试模式实际下发力矩 (n_dof_, 调试用)
  std::vector<LuGreFrictionJoint> lugre_friction_;  // 每关节一个 LuGre 实例
  VectorXd dq_filt_;                // 速度低通滤波值 (供 LuGre 输入)
  std::vector<char> lugre_enabled_;        // 逐关节是否启用 LuGre 摩擦补偿 (>0.5 启用)

  // 任务空间状态
  Vector6d actual_pose_;
  Eigen::Vector3d p_tcp_in_base_;   // 末端 TCP 在 base 系下的实际位置 (供 publishStatus)
  Eigen::Matrix3d R_tcp_in_base_{Eigen::Matrix3d::Identity()};  // 末端 TCP 在 base 系下的实际旋转 (供 publishStatus 转 RPY)
  Vector6d actual_twist_;
  Vector6d actual_twist_last_;
  Vector6d actual_accel_;
  Vector6d desired_twist_;
  Vector6d desired_pose_accel_;
  Eigen::Quaterniond desired_quaternion_;
  Eigen::Vector3d desired_position_;
  Vector6d pose_deviation_;
  Vector6d twist_deviation_;
  Vector6d accel_deviation_;
  Vector6d sensor_wrench_;
  Vector6d sensor_wrench_at_tcp_;  // 传感器力矩在 TCP 原点处的基坐标系表示 (调试用)
  Vector6d sensor_offset_;          // 六维力传感器系统偏差补偿
  Vector6d impedance_wrench_;
  Vector6d external_wrench_;        // 外部注入的 Cartesian wrench (阶段 2 物体级阻抗)
  bool has_external_wrench_{false}; // true 时 impedance_wrench_ 将直接使用 external_wrench_
  Vector6d estimated_wrench_;

  // 静差消除: 关节空间积分项 + 力闭环反馈
  Eigen::VectorXd integral_gain_joint_;       // Kq, 关节空间积分增益 (7 维)
  Eigen::VectorXd q_integral_accum_;      // 关节空间泄漏积分累加器 (n_dof_)
  Eigen::VectorXd integral_limit_joint_;       // 关节空间积分抗饱和限幅 [Nm] (7 维)
  Vector6d integral_force_threshold_;    // 自由空间判定阈值 (逐维)
  double integral_leakage_{1.0};           // 泄漏系数 σ
  double integral_gain_alpha_{2.0};        // 非线性增益放大系数
  double integral_gain_sigma_{0.02};       // 非线性增益过渡尺度 [rad]
  Eigen::VectorXd tau_integral_;          // 关节空间积分力矩输出 (调试发布用)
  DiagonalMatrix6d force_feedback_gain_;  // Kf (6 维对角)
  double force_ff_lpf_alpha_{0.1};        // 力闭环低通滤波系数
  Vector6d force_ff_filtered_;            // 力闭环滤波后输出

  // Hamiltonian 监控
  double hamiltonian_{0};
  double hamiltonian_filtered_{0};
  double hamiltonian_last_{0};
  double hamiltonian_derivative_{0};

  ReferenceType reference_;
  bool blank_reference_{true};
};

}  // namespace snowbot_impedance

#endif  // SNOWBOT_IMPEDANCE__CARTESIAN_IMPEDANCE_CONTROLLER_H

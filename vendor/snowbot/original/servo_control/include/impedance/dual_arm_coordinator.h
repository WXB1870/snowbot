#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/logger.hpp>

#include "impedance/cartesian_impedance_controller.h"

namespace snowbot_impedance {

enum class DualArmMode {
  OFF,
  LEADER,
  FOLLOWER,
  OBJECT_IMPEDANCE,
};

inline DualArmMode stringToDualArmMode(const std::string& s) {
  if (s == "leader")           return DualArmMode::LEADER;
  if (s == "follower")         return DualArmMode::FOLLOWER;
  if (s == "object_impedance") return DualArmMode::OBJECT_IMPEDANCE;
  return DualArmMode::OFF;
}

inline std::string dualArmModeToString(DualArmMode m) {
  switch (m) {
    case DualArmMode::LEADER:           return "leader";
    case DualArmMode::FOLLOWER:         return "follower";
    case DualArmMode::OBJECT_IMPEDANCE: return "object_impedance";
    default:                            return "off";
  }
}

struct DualArmConfig {
  DualArmMode mode = DualArmMode::OFF;
  std::string leader_side = "left";

  // === 阶段 1: 固定相对位姿约束 ===
  Eigen::Affine3d T_tool_constraint = Eigen::Affine3d::Identity();

  // === 阶段 2: 物体级阻抗 ===
  // 工具参考点 (TRP) 在各自 TCP 系下的固定变换
  //   T_trp_in_base = T_left_tcp_in_base * T_left_tcp_to_trp
  //                  (或 = T_right_tcp_in_base * T_right_tcp_to_trp)
  Eigen::Affine3d T_left_tcp_to_trp  = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_right_tcp_to_trp = Eigen::Affine3d::Identity();

  // 物体级阻抗参数 (6×1 对角, 顺序 [fx fy fz mx my mz])
  //   单位: stiffness [N/m, N·m/rad], damping [N·s/m, N·m·s/rad]
  Vector6d object_stiffness = (Vector6d() << 200.0, 200.0, 200.0, 20.0, 20.0, 20.0).finished();
  Vector6d object_damping   = (Vector6d() << 20.0, 20.0, 20.0, 5.0, 5.0, 5.0).finished();

  // 夹持安全阈值: 两臂 TCP 间距相对初始 latch 值的最大允许偏差 (m).
  // 刚性夹持下偏差应接近 0, 超过此值说明物体滑落/未夹持, 自动降级为 OFF.
  double max_grasp_deviation = 0.02;   // ±2cm 默认

  // === 阶段 2-内力控制 (Step 5) ===
  // 双臂相对位姿的 Full 3D PD 控制器, 合力恒为零:
  //   F_corr = Kp ⊙ (Δp_desired - Δp_actual) + Kd ⊙ (0 - Δv_relative)
  //   left.F  += F_corr        → 左臂被拉向右臂 (相对约束)
  //   right.F -= F_corr        → 右臂被拉向左臂 (合力零!)
  // 比 1D 连线方向 PI 更优: 检测全方向相对滑移 / 旋转漂移 / 拉伸.
  // 默认零增益 → 默认关闭 (纯对称分配, Step 4 行为).
  // 全方向独立增益, 顺序 [x y z].
  Eigen::Vector3d internal_force_kp = Eigen::Vector3d::Zero();  // N/m
  Eigen::Vector3d internal_force_kd = Eigen::Vector3d::Zero();  // N·s/m
  double internal_force_max_wrench = 20.0;  // 输出限幅 (N/轴), 防内力过大
};

class DualArmCoordinator {
public:
  DualArmCoordinator() = default;

  void configure(const DualArmConfig& cfg);
  const DualArmConfig& config() const { return cfg_; }

  void setMode(DualArmMode mode);
  DualArmMode mode() const { return cfg_.mode; }

  void setToolConstraint(const Eigen::Affine3d& T);

  /// 设置物体参考点 (TRP) 的期望位姿 (base 系下).
  ///   OBJECT_IMPEDANCE 模式的跟踪目标, 由上位机 ROS 话题下发.
  void setTrpReference(const Eigen::Affine3d& T_trp_desired,
                       const Vector6d& twist_desired = Vector6d::Zero());

  /// PRE-update 阶段: 在两臂各自 update() 之前调用 (每 1kHz).
  ///   OFF/LEADER/FOLLOWER: 直接 return.
  ///   OBJECT_IMPEDANCE:   计算 TRP 实际位姿/速度 → 物体阻抗 wrench → 分配到两臂
  ///                       → 调用 setExternalWrench() 注入.
  void applyPreUpdate(CartesianImpedanceController* left,
                      CartesianImpedanceController* right,
                      const rclcpp::Logger& logger = rclcpp::get_logger("dual_arm"),
                      double dt = 0.001);

  /// POST-update 阶段: 在两臂各自 update() 之后调用 (每 1kHz).
  ///   OFF/LEADER:         直接 return.
  ///   FOLLOWER:           自动捕获约束 (首次) + 主臂 TCP × T_constraint → 从臂 reference.
  ///   OBJECT_IMPEDANCE:   两臂 reference 维持为各自 TCP 实际位姿 (纯力控制模式).
  void applyPostUpdate(CartesianImpedanceController* left,
                       CartesianImpedanceController* right,
                       const rclcpp::Logger& logger = rclcpp::get_logger("dual_arm"));

  void captureConstraint(CartesianImpedanceController* leader,
                         CartesianImpedanceController* follower,
                         const rclcpp::Logger& logger = rclcpp::get_logger("dual_arm"));

  const Eigen::Affine3d& capturedConstraint() const { return cfg_.T_tool_constraint; }
  bool constraintExplicitlySet() const { return constraint_explicitly_set_; }

private:
  // TRP 实际位姿/速度计算 (融合两臂 TCP 测量)
  Eigen::Affine3d computeTrpPose(
      CartesianImpedanceController* left,
      CartesianImpedanceController* right) const;

  Vector6d computeTrpTwist(
      CartesianImpedanceController* left,
      CartesianImpedanceController* right) const;

  // 物体级阻抗 wrench: F_obj = K_obj * dx + D_obj * dv (反号, base 系下)
  Vector6d computeObjectWrench(
      const Eigen::Affine3d& T_trp_desired,
      const Vector6d& twist_desired,
      const Eigen::Affine3d& T_trp_actual,
      const Vector6d& twist_actual) const;

  // 对称力分配 + 空间 wrench 传递 (TRP → 各臂 TCP)
  //   力是自由矢量: F_arm = F_obj / 2  (空间不变)
  //   力矩需要偏移修正: M_arm = M_obj/2 + r_tcp×trp × (F_obj/2)
  //     其中 r_tcp×trp = p_tcp - p_trp  (base frame 下)
  struct ArmWrenchPair { Vector6d left; Vector6d right; };
  ArmWrenchPair distributeWrench(const Vector6d& F_obj,
                                 const Eigen::Vector3d& p_trp,
                                 const Eigen::Vector3d& p_left_tcp,
                                 const Eigen::Vector3d& p_right_tcp) const;

  // Step 5: Full 3D PD 内力控制器
  //   输入: 双臂 TCP 位置 + twist (均 base frame)
  //   期望: Δp_desired = p_r_init - p_l_init (进入模式时 latch)
  //         Δv_desired = 0  (期望双臂无相对运动)
  //   输出: F_corr (3D 修正力向量), 叠加: left.F += F_corr, right.F -= F_corr
  Eigen::Vector3d computeInternalForceCorrection(
      const Eigen::Vector3d& p_l,
      const Eigen::Vector3d& p_r,
      const Vector6d& twist_l,
      const Vector6d& twist_r) const;

  DualArmConfig cfg_;

  // 阶段 1 状态
  bool constraint_explicitly_set_{false};
  bool constraint_captured_{false};

  // 阶段 2 状态
  bool trp_reference_valid_{false};
  Eigen::Affine3d T_trp_desired_ = Eigen::Affine3d::Identity();
  Vector6d twist_trp_desired_ = Vector6d::Zero();
  bool diag_obj_printed_{false};

  // 夹持安全门 + Step 5 内力 PD 共用的状态
  bool grasp_latched_{false};
  double initial_tcp_distance_{0.0};           // 安全门用: 标量距离
  Eigen::Vector3d initial_relative_pose_{Eigen::Vector3d::Zero()};  // Step 5: Δp_desired = p_r_init - p_l_init
  double max_grasp_deviation_{0.4};           // ±2cm 默认阈值
};

}  // namespace snowbot_impedance

#include "impedance/dual_arm_coordinator.h"

#include <rclcpp/logging.hpp>

namespace snowbot_impedance {

void DualArmCoordinator::configure(const DualArmConfig& cfg) {
  cfg_ = cfg;
  constraint_explicitly_set_ = !cfg_.T_tool_constraint.isApprox(
      Eigen::Affine3d::Identity(), 1e-10);
  max_grasp_deviation_ = cfg_.max_grasp_deviation;
}

void DualArmCoordinator::setMode(DualArmMode mode) {
  if (mode != cfg_.mode) {
    trp_reference_valid_ = false;
    grasp_latched_ = false;
    diag_obj_printed_ = false;
    constraint_captured_ = false;
  }
  cfg_.mode = mode;
}

void DualArmCoordinator::setToolConstraint(const Eigen::Affine3d& T) {
  cfg_.T_tool_constraint = T;
  constraint_explicitly_set_ = !T.isApprox(Eigen::Affine3d::Identity(), 1e-10);
  constraint_captured_ = constraint_explicitly_set_;
}

void DualArmCoordinator::setTrpReference(const Eigen::Affine3d& T_trp_desired,
                                          const Vector6d& twist_desired) {
  T_trp_desired_ = T_trp_desired;
  twist_trp_desired_ = twist_desired;
  trp_reference_valid_ = true;
}

// =========================================================================
// 阶段 1: captureConstraint + applyPostUpdate (FOLLOWER)
// =========================================================================

void DualArmCoordinator::captureConstraint(
    CartesianImpedanceController* leader,
    CartesianImpedanceController* follower,
    const rclcpp::Logger& logger) {
  Eigen::Affine3d T_leader = leader->getTcpPoseInBase();
  Eigen::Affine3d T_follower = follower->getTcpPoseInBase();
  cfg_.T_tool_constraint = T_leader.inverse() * T_follower;
  constraint_captured_ = true;
  constraint_explicitly_set_ = false;

  Eigen::Vector3d p = cfg_.T_tool_constraint.translation();
  Eigen::Quaterniond q(cfg_.T_tool_constraint.linear());
  Eigen::Vector3d rpy = cfg_.T_tool_constraint.linear().eulerAngles(2, 1, 0).reverse();
  RCLCPP_INFO(logger,
              "T_tool_constraint captured:\n"
              "  translation: [%.4f, %.4f, %.4f] m\n"
              "  quaternion:  [w=%.4f, x=%.4f, y=%.4f, z=%.4f]\n"
              "  RPY(ZYX):    [%.4f, %.4f, %.4f] rad\n"
              "  matrix:\n"
              "    [%.4f  %.4f  %.4f | %.4f]\n"
              "    [%.4f  %.4f  %.4f | %.4f]\n"
              "    [%.4f  %.4f  %.4f | %.4f]\n"
              "    [  0     0     0   |   1  ]",
              p.x(), p.y(), p.z(),
              q.w(), q.x(), q.y(), q.z(),
              rpy(0), rpy(1), rpy(2),
              cfg_.T_tool_constraint(0, 0), cfg_.T_tool_constraint(0, 1), cfg_.T_tool_constraint(0, 2), cfg_.T_tool_constraint(0, 3),
              cfg_.T_tool_constraint(1, 0), cfg_.T_tool_constraint(1, 1), cfg_.T_tool_constraint(1, 2), cfg_.T_tool_constraint(1, 3),
              cfg_.T_tool_constraint(2, 0), cfg_.T_tool_constraint(2, 1), cfg_.T_tool_constraint(2, 2), cfg_.T_tool_constraint(2, 3));
}

// =========================================================================
// 阶段 2: TRP 实际位姿/速度融合
// =========================================================================

Eigen::Affine3d DualArmCoordinator::computeTrpPose(
    CartesianImpedanceController* left,
    CartesianImpedanceController* right) const {
  Eigen::Affine3d T_trp_left = left->getTcpPoseInBase() * cfg_.T_left_tcp_to_trp;
  Eigen::Affine3d T_trp_right = right->getTcpPoseInBase() * cfg_.T_right_tcp_to_trp;

  // 加权平均: 平移直接元素平均, 旋转用四元数 slerp
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translation() = 0.5 * (T_trp_left.translation() + T_trp_right.translation());

  Eigen::Quaterniond q_l(T_trp_left.linear());
  Eigen::Quaterniond q_r(T_trp_right.linear());
  q_l.normalize();
  q_r.normalize();
  Eigen::Quaterniond q_avg = q_l.slerp(0.5, q_r);
  q_avg.normalize();
  T.linear() = q_avg.toRotationMatrix();

  return T;
}

Vector6d DualArmCoordinator::computeTrpTwist(
    CartesianImpedanceController* left,
    CartesianImpedanceController* right) const {
  // 每臂 TCP twist → TRP twist (刚体运动学: w 不变, v 修正 w×r)
  const Vector6d& twist_left_tcp = left->getActualTwist();
  const Vector6d& twist_right_tcp = right->getActualTwist();

  Eigen::Vector3d p_l = left->getTcpPoseInBase().linear()
                        * cfg_.T_left_tcp_to_trp.translation();
  Eigen::Vector3d p_r = right->getTcpPoseInBase().linear()
                        * cfg_.T_right_tcp_to_trp.translation();

  Vector6d twist_l, twist_r;
  twist_l.head<3>() = twist_left_tcp.head<3>()
                      + twist_left_tcp.tail<3>().cross(p_l);
  twist_l.tail<3>() = twist_left_tcp.tail<3>();
  twist_r.head<3>() = twist_right_tcp.head<3>()
                      + twist_right_tcp.tail<3>().cross(p_r);
  twist_r.tail<3>() = twist_right_tcp.tail<3>();

  return 0.5 * (twist_l + twist_r);
}

// =========================================================================
// 阶段 2: 物体级阻抗 wrench + 力分配
// =========================================================================

Vector6d DualArmCoordinator::computeObjectWrench(
    const Eigen::Affine3d& T_desired,
    const Vector6d& twist_desired,
    const Eigen::Affine3d& T_actual,
    const Vector6d& twist_actual) const {
  Vector6d dx;
  dx.head<3>() = T_desired.translation() - T_actual.translation();

  // dR = R_actual^T * R_desired, aa.axis() 位于 T_actual 的局部 frame,
  // 必须旋转到 base frame 才能与平动误差 (已在 base frame) 保持一致.
  Eigen::Matrix3d dR = T_actual.linear().transpose() * T_desired.linear();
  Eigen::AngleAxisd aa(dR);
  Eigen::Vector3d axis_base = T_actual.linear() * aa.axis();
  dx.tail<3>() = axis_base * aa.angle();

  // twist 来自 Pinocchio LOCAL_WORLD_ALIGNED, v 和 w 都已经在 base frame, 直接相减.
  Vector6d dv = twist_desired - twist_actual;

  // dx/dv 已经是 desired - actual, 直接做 K*dx + D*dv 即得回复力,
  // 与 cartesian_impedance_controller 正常路径的最终输出一致:
  //   正常路径内部 (actual-desired) 再乘 -1 → K*(desired-actual) + D*(desired-actual)
  //   这里 dx 本身就是 desired-actual, 不需要额外翻号.
  Vector6d F_obj;
  F_obj.noalias() =
      cfg_.object_stiffness.cwiseProduct(dx) + cfg_.object_damping.cwiseProduct(dv);
  return F_obj;
}

DualArmCoordinator::ArmWrenchPair DualArmCoordinator::distributeWrench(
    const Vector6d& F_obj,
    const Eigen::Vector3d& p_trp,
    const Eigen::Vector3d& p_left_tcp,
    const Eigen::Vector3d& p_right_tcp) const {
  ArmWrenchPair pair;

  // F_obj = [F_obj; M_obj] (base frame, 在 TRP 点定义)
  const Eigen::Vector3d F_half = 0.5 * F_obj.head<3>();
  const Eigen::Vector3d M_half = 0.5 * F_obj.tail<3>();

  // === 左臂 ===
  // 空间 wrench 传递: 力不变, 力矩修正 r × F
  //   M_tcp = M_trp + (p_tcp - p_trp) × F
  {
    Eigen::Vector3d r_l = p_left_tcp - p_trp;
    pair.left.head<3>() = F_half;
    pair.left.tail<3>() = M_half + r_l.cross(F_half);
  }

  // === 右臂 ===
  {
    Eigen::Vector3d r_r = p_right_tcp - p_trp;
    pair.right.head<3>() = F_half;
    pair.right.tail<3>() = M_half + r_r.cross(F_half);
  }

  return pair;
}

// =========================================================================
// Step 5: Full 3D PD 内力控制器
// =========================================================================

Eigen::Vector3d DualArmCoordinator::computeInternalForceCorrection(
    const Eigen::Vector3d& p_l,
    const Eigen::Vector3d& p_r,
    const Vector6d& twist_l,
    const Vector6d& twist_r) const {
  // === 默认关闭 ===
  if (cfg_.internal_force_kp.isZero(1e-9) && cfg_.internal_force_kd.isZero(1e-9)) {
    return Eigen::Vector3d::Zero();
  }

  // 当前两臂相对位姿 (右臂 - 左臂, base frame)
  const Eigen::Vector3d delta_p_actual = p_r - p_l;

  // 相对速度: (右臂线速度 - 左臂线速度, base frame, LOCAL_WORLD_ALIGNED)
  // 期望相对速度 = 0 (双臂无相对运动)
  const Eigen::Vector3d delta_v_actual = twist_r.head<3>() - twist_l.head<3>();

  // === P 项: 相对位姿误差 ===
  //   e_pose = Δp_desired - Δp_actual
  //   F_corr 是"弹簧力向量", 叠加时左 = -F_corr / 右 = +F_corr (合力为零):
  //     挤压 → |Δp_actual| < |Δp_desired| → 两臂各自被推开 ✅
  //     拉伸 → |Δp_actual| > |Δp_desired| → 两臂各自被拉近 ✅
  const Eigen::Vector3d e_pose = initial_relative_pose_ - delta_p_actual;

  // === D 项: 相对速度误差 ===
  //   期望相对速度 = 0 (刚体夹持)
  //   delta_v_actual 沿 e_pose 正向 = 阻尼沿反向 (抑制相对运动) ✅
  const Eigen::Vector3d e_vel = -delta_v_actual;

  // === 合力向量, 逐轴 clamp ===
  Eigen::Vector3d F_corr =
      cfg_.internal_force_kp.cwiseProduct(e_pose) +
      cfg_.internal_force_kd.cwiseProduct(e_vel);

  const double lim = cfg_.internal_force_max_wrench;
  F_corr = F_corr.cwiseMax(-lim).cwiseMin(lim);

  return F_corr;
}

// =========================================================================
// PRE-update: OBJECT_IMPEDANCE 模式注入外部 wrench
// =========================================================================

void DualArmCoordinator::applyPreUpdate(
    CartesianImpedanceController* left,
    CartesianImpedanceController* right,
    const rclcpp::Logger& logger,
    double dt) {
  if (cfg_.mode != DualArmMode::OBJECT_IMPEDANCE) {
    return;
  }
  if (!left || !right) return;

  // === 安全检查: object_stiffness 和 object_damping 不能同时为零 ===
  // K=D=0 意味着 F_obj = 0 → 注入零 wrench → 两臂完全丧失笛卡尔约束,
  // 只有零空间力输出, 可能导致失控. 检测到时降级为 OFF 模式.
  const bool k_zero = cfg_.object_stiffness.isApprox(Vector6d::Zero(), 1e-6);
  const bool d_zero = cfg_.object_damping.isApprox(Vector6d::Zero(), 1e-6);
  if (k_zero && d_zero) {
    RCLCPP_ERROR(logger,
                 "OBJECT_IMPEDANCE: object_stiffness AND object_damping are BOTH zero "
                 "— this would disable Cartesian constraints! Downgrading to OFF. "
                 "Please configure [dual_arm.object_impedance] in YAML.");
    cfg_.mode = DualArmMode::OFF;
    left->clearExternalWrench();
    right->clearExternalWrench();
    return;
  } else if (k_zero || d_zero) {
    RCLCPP_WARN(logger,
                "OBJECT_IMPEDANCE: %s is all zero — object impedance will be soft. "
                "Consider configuring both stiffness and damping.",
                k_zero ? "object_stiffness" : "object_damping");
  }

  if (!trp_reference_valid_) {
    // 首次进入: 用当前 TRP 位姿作为期望 (零跟踪误差, 零输出 wrench)
    T_trp_desired_ = computeTrpPose(left, right);
    twist_trp_desired_ = computeTrpTwist(left, right);
    trp_reference_valid_ = true;

    // === TRP 几何标定检查 ===
    // 若 YAML 中 trp_geometry 段被注释掉 (默认单位阵), 两臂 TCP→TRP 都是 I,
    // 意味着 T_trp_left = 左臂TCP, T_trp_right = 右臂TCP,
    // 最终融合出的 TRP = 两臂 TCP 中点, 仅适用于对称夹持且 TCP 间距较小的场景.
    const bool trp_identity =
        cfg_.T_left_tcp_to_trp.isApprox(Eigen::Affine3d::Identity(), 1e-8) &&
        cfg_.T_right_tcp_to_trp.isApprox(Eigen::Affine3d::Identity(), 1e-8);
    if (trp_identity) {
      RCLCPP_WARN(logger,
                  "OBJECT_IMPEDANCE: trp_geometry NOT configured, using TCP midpoint "
                  "as TRP (left_tcp_to_trp = right_tcp_to_trp = Identity). "
                  "Arm TCPs are at [%.3f,%.3f,%.3f] and [%.3f,%.3f,%.3f] — "
                  "midpoint approx only valid for symmetric grasp.",
                  left->getTcpPoseInBase().translation().x(),
                  left->getTcpPoseInBase().translation().y(),
                  left->getTcpPoseInBase().translation().z(),
                  right->getTcpPoseInBase().translation().x(),
                  right->getTcpPoseInBase().translation().y(),
                  right->getTcpPoseInBase().translation().z());
    }

    RCLCPP_INFO(logger,
                "OBJECT_IMPEDANCE: auto-latched current TRP pose as reference\n"
                "  object_stiffness: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n"
                "  object_damping:   [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                cfg_.object_stiffness(0), cfg_.object_stiffness(1), cfg_.object_stiffness(2),
                cfg_.object_stiffness(3), cfg_.object_stiffness(4), cfg_.object_stiffness(5),
                cfg_.object_damping(0), cfg_.object_damping(1), cfg_.object_damping(2),
                cfg_.object_damping(3), cfg_.object_damping(4), cfg_.object_damping(5));
  }

  // === 夹持安全门 ===
  // 首次进入 OBJECT_IMPEDANCE 时 latch 两臂 TCP 间距作为刚性夹持基线.
  // 每帧检测当前间距偏差, 超过阈值说明物体已滑落 / 未夹持,
  // 对称力分配假设 (F_left = F_right = 0.5·F_obj) 不再成立 — 非受扰手臂
  // 会被无端牵拉 → 可能碰撞. 自动降级为 OFF 模式保护.
  const Eigen::Vector3d p_l = left->getTcpPoseInBase().translation();
  const Eigen::Vector3d p_r = right->getTcpPoseInBase().translation();
  const double dist_current = (p_r - p_l).norm();

  if (!grasp_latched_) {
    initial_tcp_distance_ = dist_current;
    initial_relative_pose_ = p_r - p_l;   // Step 5 Full 3D PD: 期望相对位姿
    grasp_latched_ = true;
    RCLCPP_INFO(logger,
                "OBJECT_IMPEDANCE: latched grasp distance = %.4f m, "
                "max allowed deviation = ±%.4f m",
                initial_tcp_distance_, max_grasp_deviation_);
  } else {
    const double delta = dist_current - initial_tcp_distance_;
    if (std::abs(delta) > max_grasp_deviation_) {
      RCLCPP_ERROR(logger,
                   "OBJECT_IMPEDANCE: grasp LOST! TCP distance changed by %.4f m "
                   "(initial=%.4f m, current=%.4f m, threshold=±%.4f m). "
                   "Possible object slip or no grasp. Downgrading to OFF "
                   "to avoid arm collision.",
                   delta, initial_tcp_distance_, dist_current, max_grasp_deviation_);
      cfg_.mode = DualArmMode::OFF;
      left->clearExternalWrench();
      right->clearExternalWrench();
      return;
    }
  }

  Eigen::Affine3d T_trp_actual = computeTrpPose(left, right);
  Vector6d twist_trp_actual = computeTrpTwist(left, right);

  Vector6d F_obj = computeObjectWrench(
      T_trp_desired_, twist_trp_desired_, T_trp_actual, twist_trp_actual);

  ArmWrenchPair pair = distributeWrench(
      F_obj,
      T_trp_actual.translation(),
      left->getTcpPoseInBase().translation(),
      right->getTcpPoseInBase().translation());

  // === Step 5: Full 3D PD 内力修正 ===
  // 叠加后合力恒为零, 不影响物体级阻抗的平移/旋转行为:
  //   left.F  += -F_corr  (压缩时推开, 拉伸时拉近 — 弹簧+阻尼直觉)
  //   right.F +=  F_corr  (牛顿第三定律, 反作用力)
  // 力矩不修正: 修正力作用在 TCP 点, 对 TCP 点力矩贡献为零.
  const Vector6d& twist_l = left->getActualTwist();
  const Vector6d& twist_r = right->getActualTwist();
  const Eigen::Vector3d F_corr = computeInternalForceCorrection(p_l, p_r, twist_l, twist_r);
  if (!F_corr.isZero(1e-6)) {
    pair.left.head<3>()  += -F_corr;
    pair.right.head<3>() +=  F_corr;
  }

  left->setExternalWrench(pair.left);
  right->setExternalWrench(pair.right);

  if (!diag_obj_printed_) {
    RCLCPP_INFO(logger,
                "[ObjImpDiag] TRP pos=[%.3f,%.3f,%.3f] F_obj=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                T_trp_actual.translation().x(),
                T_trp_actual.translation().y(),
                T_trp_actual.translation().z(),
                F_obj(0), F_obj(1), F_obj(2), F_obj(3), F_obj(4), F_obj(5));
    diag_obj_printed_ = true;
  }
}

// =========================================================================
// POST-update: FOLLOWER 设 reference, OBJECT_IMPEDANCE 维持零跟踪
// =========================================================================

void DualArmCoordinator::applyPostUpdate(
    CartesianImpedanceController* left,
    CartesianImpedanceController* right,
    const rclcpp::Logger& logger) {
  if (cfg_.mode == DualArmMode::OFF || cfg_.mode == DualArmMode::LEADER) {
    return;
  }
  if (!left || !right) return;

  if (cfg_.mode == DualArmMode::FOLLOWER) {
    CartesianImpedanceController* leader =
        (cfg_.leader_side == "right") ? right : left;
    CartesianImpedanceController* follower =
        (cfg_.leader_side == "right") ? left : right;

    if (!constraint_captured_ && !constraint_explicitly_set_) {
      RCLCPP_INFO(logger, "First FOLLOWER tick: auto-capturing T_tool_constraint");
      captureConstraint(leader, follower, logger);
    } else if (!constraint_captured_ && constraint_explicitly_set_) {
      constraint_captured_ = true;
      Eigen::Vector3d p = cfg_.T_tool_constraint.translation();
      Eigen::Quaterniond q(cfg_.T_tool_constraint.linear());
      RCLCPP_INFO(logger, "Using YAML-configured T_tool_constraint: pos=[%.4f,%.4f,%.4f] quat=[%.4f,%.4f,%.4f,%.4f]",
                  p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z());
    }

    Eigen::Affine3d T_leader_tcp = leader->getTcpPoseInBase();
    Eigen::Affine3d T_follower_target = T_leader_tcp * cfg_.T_tool_constraint;

    const Vector6d& leader_twist = leader->getActualTwist();
    Eigen::Vector3d v_leader = leader_twist.head<3>();
    Eigen::Vector3d w_leader = leader_twist.tail<3>();
    Eigen::Vector3d p_constraint = cfg_.T_tool_constraint.translation();
    Eigen::Vector3d p_in_base = T_leader_tcp.linear() * p_constraint;

    Vector6d follower_twist;
    follower_twist.head<3>() = v_leader + w_leader.cross(p_in_base);
    follower_twist.tail<3>() = w_leader;

    follower->setReferencePose(T_follower_target, &follower_twist);
    return;
  }

  if (cfg_.mode == DualArmMode::OBJECT_IMPEDANCE) {
    // 物体级阻抗模式: 两臂 reference 设为各自当前 TCP 实际位姿
    // 这样 K*dx+D*dv 的内部计算产生零 wrench → 但 setExternalWrench 已在 pre 阶段
    // 注入了物体级力, update() 会使用外部值. 设 reference 是为了保证位置环
    // (如果有的话) 不会产生额外力矩干扰.
    Eigen::Affine3d T_left_actual = left->getTcpPoseInBase();
    Eigen::Affine3d T_right_actual = right->getTcpPoseInBase();
    Vector6d zero_twist = Vector6d::Zero();
    left->setReferencePose(T_left_actual, &zero_twist);
    right->setReferencePose(T_right_actual, &zero_twist);
    return;
  }
}

}  // namespace snowbot_impedance

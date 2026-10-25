// 移植自 ros2_impedance_controller (qleonardolp, Apache 2.0)
// 适配 servo_control 主循环直接驱动 EtherCAT 的架构.
#include "impedance/cartesian_impedance_controller.h"

#include <cmath>
#include <limits>

namespace snowbot_impedance {

//==============================================================================
// 阻尼伪逆 (Damped Least-Squares / Tikhonov 正则化)
//   参考: CentroEPiaggio/kuka-lwr pseudo_inversion.h (public domain)
//   公式: M^# = V * diag(σ_i/(σ_i^2+λ^2)) * U^T
//   λ=0  -> 标准 Moore-Penrose 伪逆
//   λ>0  -> 奇异点附近平滑, 避免力矩/速度爆炸
//==============================================================================
static void dampedPseudoInverse(const Eigen::MatrixXd& M, Eigen::MatrixXd* M_pinv,
                                double lambda = 0.02) {
  if (lambda <= 0.0) {
    *M_pinv = M.completeOrthogonalDecomposition().pseudoInverse();
    return;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::VectorXd& sing = svd.singularValues();
  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(M.cols(), M.rows());
  const double lambda2 = lambda * lambda;
  for (int i = 0; i < sing.size(); ++i)
    S(i, i) = sing(i) / (sing(i) * sing(i) + lambda2);
  *M_pinv = svd.matrixV() * S * svd.matrixU().transpose();
}

//==============================================================================
// ImpedanceConfig 静态方法
//==============================================================================

ImpedanceConfig ImpedanceConfig::makeDefault(const std::string& side,
                                              const std::string& urdf_path) {
  ImpedanceConfig cfg;
  cfg.urdf_path = urdf_path;
  cfg.ee_link = (side == "left") ? "arm_l_link_7" : "arm_r_link_7";
  cfg.controlled_joint_names.clear();
  const std::string prefix = (side == "left") ? "arm_l_joint_" : "arm_r_joint_";
  for (int i = 1; i <= 7; ++i) {
    cfg.controlled_joint_names.push_back(prefix + std::to_string(i));
  }
  cfg.ft_sensor_topic.clear();
  cfg.stiffness = {200.0, 200.0, 200.0, 20.0, 20.0, 20.0};
  cfg.damping   = {20.0, 20.0, 20.0, 2.0, 2.0, 2.0};
  cfg.taskspace_mass = 0.0;
  cfg.pinv_damping = 0.02;
  cfg.cmd_lpf_alpha = 0.4;
  cfg.acc_lpf_alpha = 0.2;
  cfg.has_effort_states = false;
  cfg.tool_mass = 0.0;
  cfg.tool_com_in_tcp = {0.0, 0.0, 0.0};
  // 默认几何变换为单位变换 (与 ee_link 重合, 向后兼容)
  cfg.sensor_to_ee = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  cfg.tcp_to_ee    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  cfg.effort_limit_per_joint = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
  cfg.kt_gravity = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  cfg.lugre_friction.enable = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  cfg.lugre_friction.sigma0 = {1.0e5, 1.0e5, 1.0e5, 1.0e5, 1.0e5, 1.0e5, 1.0e5};
  cfg.lugre_friction.sigma1 = {300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0};
  cfg.lugre_friction.sigma2 = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  cfg.lugre_friction.Fs = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
  cfg.lugre_friction.Fc = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  cfg.lugre_friction.vs = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
  cfg.lugre_friction.dq_lpf_alpha = 0.3;
  cfg.nullspace_stiffness = 0.0;
  cfg.nullspace_damping = 2.0;
  cfg.filter_params_pose = 0.95;
  cfg.filter_params_nullspace = 0.95;
  cfg.update_frequency = 1000.0;
  return cfg;
}

ImpedanceConfig ImpedanceConfig::fromYaml(const std::string& yaml_path,
                                          const std::string& side,
                                          const std::string& urdf_path) {
  // 先构造默认值, 后续按 yaml 覆盖
  ImpedanceConfig cfg = makeDefault(side, urdf_path);

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception& e) {
    std::cerr << "[ImpedanceConfig] 无法加载 yaml 文件: " << yaml_path
              << " (" << e.what() << "), 使用默认配置" << std::endl;
    return cfg;
  }

  // 支持两种顶层结构:
  //   1. impedance: { left: {...}, right: {...} }
  //   2. 直接 left: {...}, right: {...}  (兼容)
  YAML::Node side_node;
  if (root["impedance"] && root["impedance"][side]) {
    side_node = root["impedance"][side];
  } else if (root[side]) {
    side_node = root[side];
  } else {
    std::cerr << "[ImpedanceConfig] yaml 中未找到 impedance/" << side
              << " 节点, 使用默认配置" << std::endl;
    return cfg;
  }

  // 可选: 在 yaml 中覆盖 urdf_path / ee_link
  if (side_node["urdf_path"]) {
    cfg.urdf_path = side_node["urdf_path"].as<std::string>();
  }
  if (side_node["ee_link"]) {
    cfg.ee_link = side_node["ee_link"].as<std::string>();
  }
  if (side_node["controlled_joint_names"]) {
    cfg.controlled_joint_names =
        side_node["controlled_joint_names"].as<std::vector<std::string>>();
  }
  if (side_node["ft_sensor_topic"]) {
    cfg.ft_sensor_topic = side_node["ft_sensor_topic"].as<std::string>();
  }

  // 阻抗参数
  if (side_node["stiffness"]) {
    cfg.stiffness = side_node["stiffness"].as<std::vector<double>>();
  }
  if (side_node["damping"]) {
    cfg.damping = side_node["damping"].as<std::vector<double>>();
  }
  if (side_node["taskspace_mass"]) {
    cfg.taskspace_mass = side_node["taskspace_mass"].as<double>();
  }
  if (side_node["pinv_damping"]) {
    cfg.pinv_damping = side_node["pinv_damping"].as<double>();
  }
  if (side_node["cmd_lpf_alpha"]) {
    cfg.cmd_lpf_alpha = side_node["cmd_lpf_alpha"].as<double>();
  }
  if (side_node["acc_lpf_alpha"]) {
    cfg.acc_lpf_alpha = side_node["acc_lpf_alpha"].as<double>();
  }
  if (side_node["has_effort_states"]) {
    cfg.has_effort_states = side_node["has_effort_states"].as<bool>();
  }

  // 末端工具重力补偿参数
  if (side_node["tool_mass"]) {
    cfg.tool_mass = side_node["tool_mass"].as<double>();
  }
  if (side_node["tool_com_in_tcp"]) {
    cfg.tool_com_in_tcp = side_node["tool_com_in_tcp"].as<std::vector<double>>();
    if (cfg.tool_com_in_tcp.size() != 3) {
      std::cerr << "[ImpedanceConfig] 警告: tool_com_in_tcp 应为 3 维, "
                << "实际 " << cfg.tool_com_in_tcp.size()
                << ", 已回退到 [0,0,0]" << std::endl;
      cfg.tool_com_in_tcp = {0.0, 0.0, 0.0};
    }
  }
  // 末端几何变换 (sensor/tcp 相对 ee_link)
  //   长度必须为 7 [x, y, z, qx, qy, qz, qw], 否则保持默认单位变换
  if (side_node["sensor_to_ee"]) {
    auto v = side_node["sensor_to_ee"].as<std::vector<double>>();
    if (v.size() == 7) cfg.sensor_to_ee = v;
    else std::cerr << "[ImpedanceConfig] 警告: sensor_to_ee 应为 7 维, "
                   << "实际 " << v.size() << ", 已忽略" << std::endl;
  }
  if (side_node["tcp_to_ee"]) {
    auto v = side_node["tcp_to_ee"].as<std::vector<double>>();
    if (v.size() == 7) cfg.tcp_to_ee = v;
    else std::cerr << "[ImpedanceConfig] 警告: tcp_to_ee 应为 7 维, "
                   << "实际 " << v.size() << ", 已忽略" << std::endl;
  }

  // 关节力矩限幅 (支持标量或 7 维数组)
  if (side_node["effort_limit_per_joint"]) {
    YAML::Node lim_node = side_node["effort_limit_per_joint"];
    if (lim_node.IsScalar()) {
      // 标量: 7 个关节统一限幅
      const double v = lim_node.as<double>();
      cfg.effort_limit_per_joint = {v, v, v, v, v, v, v};
    } else {
      cfg.effort_limit_per_joint =
          lim_node.as<std::vector<double>>();
      if (cfg.effort_limit_per_joint.size() != 7) {
        std::cerr << "[ImpedanceConfig] 警告: effort_limit_per_joint 应为 7 维, "
                  << "实际 " << cfg.effort_limit_per_joint.size()
                  << ", 已回退到默认 5.0" << std::endl;
        cfg.effort_limit_per_joint = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
      }
      // 校验非负
      for (double v : cfg.effort_limit_per_joint) {
        if (v < 0.0) {
          std::cerr << "[ImpedanceConfig] 警告: effort_limit_per_joint 含负值 "
                    << v << ", 已取绝对值" << std::endl;
        }
      }
    }
  }

  // 重力补偿系数 (支持标量或 7 维数组, 默认 1.0=完全补偿)
  if (side_node["kt_gravity"]) {
    YAML::Node kt_node = side_node["kt_gravity"];
    if (kt_node.IsScalar()) {
      const double v = kt_node.as<double>();
      cfg.kt_gravity = {v, v, v, v, v, v, v};
    } else {
      cfg.kt_gravity = kt_node.as<std::vector<double>>();
      if (cfg.kt_gravity.size() != 7) {
        std::cerr << "[ImpedanceConfig] 警告: kt_gravity 应为 7 维, 实际 "
                  << cfg.kt_gravity.size() << ", 已回退到默认 1.0" << std::endl;
        cfg.kt_gravity = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
      }
    }
  }

  // LuGre 动态摩擦补偿参数
  if (side_node["lugre_friction"]) {
    YAML::Node lf = side_node["lugre_friction"];
    // enable/sigma0/sigma1/sigma2 均可为标量 (统一) 或 7 维数组 (逐关节)
    auto parse7 = [&](const char* key, std::vector<double>& out,
                      const std::vector<double>& def) {
      if (lf[key]) {
        YAML::Node n = lf[key];
        if (n.IsScalar()) {
          double v = n.as<double>();
          out = {v, v, v, v, v, v, v};
        } else {
          out = n.as<std::vector<double>>();
          if (out.size() != 7) {
            std::cerr << "[ImpedanceConfig] 警告: lugre_friction." << key
                      << " 应为 7 维, 实际 " << out.size()
                      << ", 已回退到默认" << std::endl;
            out = def;
          }
        }
      }
    };
    parse7("enable", cfg.lugre_friction.enable,
           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    parse7("sigma0", cfg.lugre_friction.sigma0,
           {1.0e5, 1.0e5, 1.0e5, 1.0e5, 1.0e5, 1.0e5, 1.0e5});
    parse7("sigma1", cfg.lugre_friction.sigma1,
           {300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0});
    parse7("sigma2", cfg.lugre_friction.sigma2,
           {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    parse7("Fs", cfg.lugre_friction.Fs,
           {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5});
    parse7("Fc", cfg.lugre_friction.Fc,
           {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    parse7("vs", cfg.lugre_friction.vs,
           {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
    if (lf["dq_lpf_alpha"])
      cfg.lugre_friction.dq_lpf_alpha = lf["dq_lpf_alpha"].as<double>();
  }

  // 零空间控制参数
  if (side_node["nullspace_stiffness"]) {
    cfg.nullspace_stiffness = side_node["nullspace_stiffness"].as<double>();
  }
  if (side_node["nullspace_damping"]) {
    cfg.nullspace_damping = side_node["nullspace_damping"].as<double>();
  }

  // 参考信号滤波参数
  if (side_node["filter_params_pose"]) {
    cfg.filter_params_pose = side_node["filter_params_pose"].as<double>();
  }
  if (side_node["filter_params_nullspace"]) {
    cfg.filter_params_nullspace =
        side_node["filter_params_nullspace"].as<double>();
  }
  if (side_node["update_frequency"]) {
    cfg.update_frequency = side_node["update_frequency"].as<double>();
  }

  // 静差消除参数 (可选, 缺省全零=禁用)
  if (side_node["integral_gain_joint"]) {
    // 支持 7 维数组或标量 (标量自动展开为 7 维)
    if (side_node["integral_gain_joint"].IsSequence()) {
      cfg.integral_gain_joint =
          side_node["integral_gain_joint"].as<std::vector<double>>();
    } else {
      double scalar = side_node["integral_gain_joint"].as<double>();
      cfg.integral_gain_joint.assign(7, scalar);
    }
  }
  if (side_node["integral_limit_joint"]) {
    if (side_node["integral_limit_joint"].IsSequence()) {
      cfg.integral_limit_joint =
          side_node["integral_limit_joint"].as<std::vector<double>>();
    } else {
      double scalar = side_node["integral_limit_joint"].as<double>();
      cfg.integral_limit_joint.assign(7, scalar);
    }
  }
  if (side_node["integral_force_threshold"]) {
    cfg.integral_force_threshold =
        side_node["integral_force_threshold"].as<std::vector<double>>();
  }
  if (side_node["integral_leakage"]) {
    cfg.integral_leakage = side_node["integral_leakage"].as<double>();
  }
  if (side_node["integral_gain_alpha"]) {
    cfg.integral_gain_alpha = side_node["integral_gain_alpha"].as<double>();
  }
  if (side_node["integral_gain_sigma"]) {
    cfg.integral_gain_sigma = side_node["integral_gain_sigma"].as<double>();
  }
  if (side_node["force_feedback_gain"]) {
    cfg.force_feedback_gain =
        side_node["force_feedback_gain"].as<std::vector<double>>();
  }
  if (side_node["force_ff_lpf_alpha"]) {
    cfg.force_ff_lpf_alpha = side_node["force_ff_lpf_alpha"].as<double>();
  }

  // 六维力传感器系统偏差补偿
  if (side_node["sensor_offset"]) {
    cfg.sensor_offset =
        side_node["sensor_offset"].as<std::vector<double>>();
  }

  // 维度校验
  if (cfg.stiffness.size() != 6 || cfg.damping.size() != 6) {
    std::cerr << "[ImpedanceConfig] 警告: stiffness/damping 应为 6 维, "
              << "实际 stiffness=" << cfg.stiffness.size()
              << ", damping=" << cfg.damping.size()
              << ", 已回退到默认值" << std::endl;
    cfg.stiffness = {200.0, 200.0, 200.0, 20.0, 20.0, 20.0};
    cfg.damping   = {20.0, 20.0, 20.0, 2.0, 2.0, 2.0};
  }
  if (cfg.controlled_joint_names.size() != 7) {
    std::cerr << "[ImpedanceConfig] 警告: controlled_joint_names 应为 7 个, "
              << "实际 " << cfg.controlled_joint_names.size()
              << ", 已回退到默认值" << std::endl;
    const std::string prefix =
        (side == "left") ? "arm_l_joint_" : "arm_r_joint_";
    cfg.controlled_joint_names.clear();
    for (int i = 1; i <= 7; ++i) {
      cfg.controlled_joint_names.push_back(prefix + std::to_string(i));
    }
  }

  std::cout << "[ImpedanceConfig] 加载成功 (" << side << ") from "
            << yaml_path << std::endl;
  return cfg;
}

//==============================================================================
// CartesianImpedanceController 实现
//==============================================================================

bool CartesianImpedanceController::init(rclcpp::Node::SharedPtr node,
                                         const ImpedanceConfig& config) {
  node_ = node;
  config_ = config;
  cmd_lpf_alpha_ = config.cmd_lpf_alpha;
  acc_lpf_alpha_ = config.acc_lpf_alpha;
  has_effort_states_ = config.has_effort_states;
  delta_t_ = 0.001;

  // 加载末端工具重力补偿参数
  tool_mass_ = config.tool_mass;
  // 预计算末端几何变换 (sensor/tcp 相对 ee_link)
  T_ee_sensor_ = parsePose(config.sensor_to_ee);
  T_ee_tcp_    = parsePose(config.tcp_to_ee);
  R_sensor_to_ee_ = T_ee_sensor_.rotation();
  t_sensor_in_ee_ = T_ee_sensor_.translation();
  R_ee_to_tcp_    = T_ee_tcp_.rotation();        // 注意: tcp_to_ee 表示 TCP 在 ee 系下
  t_ee_in_tcp_    = T_ee_tcp_.translation();    // 因此 R_ee_to_tcp = R(T_ee_tcp)
  // 工具质心: 从 tcp 系转到 ee 系, 复用原 tool_com_in_ee_ 内部接口
  if (config.tool_com_in_tcp.size() == 3) {
    tool_com_in_ee_ = R_ee_to_tcp_ * Eigen::Vector3d(config.tool_com_in_tcp.data())
                   + t_ee_in_tcp_;
  } else {
    tool_com_in_ee_.setZero();
    tool_mass_ = 0.0;
  }
  if (tool_mass_ > 0.0) {
    RCLCPP_INFO(node_->get_logger(),
                "Tool gravity comp enabled: mass=%.3f kg, com_in_ee=[%.3f, %.3f, %.3f]",
                tool_mass_, tool_com_in_ee_.x(), tool_com_in_ee_.y(),
                tool_com_in_ee_.z());
  }
  RCLCPP_INFO(node_->get_logger(),
              "End-effector transforms: sensor_to_ee t=[%.3f, %.3f, %.3f], "
              "tcp_to_ee t=[%.3f, %.3f, %.3f]",
              t_sensor_in_ee_.x(), t_sensor_in_ee_.y(), t_sensor_in_ee_.z(),
              t_ee_in_tcp_.x(), t_ee_in_tcp_.y(), t_ee_in_tcp_.z());

  // 关节力矩限幅 (维度 = n_dof_, 这里 n_dof_ 还未赋值, 在 configureRobotModel 后赋)
  // 临时缓存到 std::vector, configureRobotModel 后拷贝到 effort_limit_per_joint_
  // (此处仅记录日志)
  if (config.effort_limit_per_joint.size() == 7) {
    RCLCPP_INFO(node_->get_logger(),
                "Effort limit per joint: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f] Nm",
                config.effort_limit_per_joint[0], config.effort_limit_per_joint[1],
                config.effort_limit_per_joint[2], config.effort_limit_per_joint[3],
                config.effort_limit_per_joint[4], config.effort_limit_per_joint[5],
                config.effort_limit_per_joint[6]);
  }

  // 零空间控制 + 参考滤波参数 (从 config 加载到成员变量, 后续 update 使用)
  nullspace_stiffness_ = config.nullspace_stiffness;
  nullspace_damping_   = config.nullspace_damping;
  filter_params_pose_ = config.filter_params_pose;
  filter_params_nullspace_ = config.filter_params_nullspace;
  update_frequency_ = config.update_frequency;

  // 静差消除参数加载 (标量参数先存, 向量参数在 n_dof_ 赋值后初始化)
  integral_leakage_ = config.integral_leakage;
  integral_gain_alpha_ = config.integral_gain_alpha;
  integral_gain_sigma_ = config.integral_gain_sigma;
  for (int i = 0; i < 6; ++i) {
    integral_force_threshold_(i) =
        (i < static_cast<int>(config.integral_force_threshold.size()))
            ? config.integral_force_threshold[i] : 3.0;
    force_feedback_gain_.diagonal()(i) =
        (i < static_cast<int>(config.force_feedback_gain.size()))
            ? config.force_feedback_gain[i] : 0.0;
  }
  force_ff_lpf_alpha_ = config.force_ff_lpf_alpha;
  force_ff_filtered_.setZero();
  // 六维力传感器系统偏差
  for (int i = 0; i < 6; ++i) {
    sensor_offset_(i) =
        (i < static_cast<int>(config.sensor_offset.size()))
            ? config.sensor_offset[i] : 0.0;
  }
  RCLCPP_INFO(node_->get_logger(),
              "Sensor offset: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              sensor_offset_(0), sensor_offset_(1), sensor_offset_(2),
              sensor_offset_(3), sensor_offset_(4), sensor_offset_(5));
  // Steady-state elimination 参数打印移到 n_dof_ 赋值后 (见下方)
  RCLCPP_INFO(node_->get_logger(),
              "Nullspace: K=%.2f Nm/rad, D=%.2f Nm*s/rad (%s)",
              nullspace_stiffness_, nullspace_damping_,
              (nullspace_stiffness_ > 0.0 ? "enabled" : "disabled"));
  RCLCPP_INFO(node_->get_logger(),
              "Reference filter: pose=%.3f, nullspace=%.3f (update_freq=%.0f Hz)",
              filter_params_pose_, filter_params_nullspace_, update_frequency_);

  // 1. 配置 URDF + 关节模型
  if (!configureRobotModel()) {
    RCLCPP_ERROR(node_->get_logger(), "Robot model configuration failed");
    return false;
  }

  // 2. 阻抗参数 (移植自 cartesian_controller.cpp read_parameters)
  if (config_.stiffness.size() != kCartesianDim) {
    RCLCPP_ERROR(node_->get_logger(), "stiffness must have 6 elements");
    return false;
  }
  desired_stiffness_ = Vector6d(Eigen::Map<const Vector6d>(config_.stiffness.data())).asDiagonal();

  if (std::fpclassify(config_.taskspace_mass) == FP_ZERO) {
    // 阻尼模式: 直接使用 damping 参数
    if (config_.damping.size() != kCartesianDim) {
      RCLCPP_ERROR(node_->get_logger(), "damping must have 6 elements when mass=0");
      return false;
    }
    desired_damping_ =
        Vector6d(Eigen::Map<const Vector6d>(config_.damping.data())).asDiagonal();
    desired_inertia_.setIdentity();
    inertia_shaping_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "Inertia shaping disabled (taskspace_mass=0), using explicit damping.");
  } else {
    // 惯量整形: 由 M,K 推导 D = 2*zeta*sqrt(K*M)
    const double mass = config_.taskspace_mass;
    const double I = 0.4 * mass * (0.425 * 0.425);  // 球体转动惯量
    desired_inertia_.diagonal() << mass, mass, mass, I, I, I;
    desired_inertia_inv_ = desired_inertia_.diagonal().cwiseInverse().asDiagonal();
    desired_damping_.diagonal() =
        2 * kDampingRatio *
        (desired_inertia_.diagonal().array() * desired_stiffness_.diagonal().array())
            .abs()
            .sqrt();
    inertia_shaping_ = true;
  }

  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Damping diagonal: " << desired_damping_.diagonal().transpose());

  // 3. 动态 Eigen 成员尺寸
  robot_q_.resize(n_dof_);
  robot_dq_.resize(n_dof_);
  robot_efforts_.resize(n_dof_);
  effort_commands_.resize(n_dof_);
  jacobian_ = Matrix6Xd::Zero(kCartesianDim, n_dof_);
  jacobian_dt_ = Matrix6Xd::Zero(kCartesianDim, n_dof_);
  jacobian_full_ = Matrix6Xd::Zero(kCartesianDim, n_q_full_);
  jsim_jpinv_ = Eigen::MatrixXd::Zero(n_dof_, kCartesianDim);
  jsim_jpinv_dj_ = Eigen::MatrixXd::Zero(n_dof_, n_dof_);
  tau_desired_.resize(n_dof_);
  // 关节力矩限幅 (维度与 n_dof_ 一致)
  effort_limit_per_joint_.resize(n_dof_);
  if (config_.effort_limit_per_joint.size() == n_dof_) {
    for (size_t i = 0; i < n_dof_; ++i) {
      effort_limit_per_joint_(i) = std::abs(config_.effort_limit_per_joint[i]);
    }
  } else {
    effort_limit_per_joint_.setConstant(5.0);
    RCLCPP_WARN(node_->get_logger(),
                "effort_limit_per_joint 维度 (%zu) != n_dof (%zu), 已用 5.0 Nm",
                config_.effort_limit_per_joint.size(), n_dof_);
  }

  // 重力补偿系数 (维度 = n_dof_, 默认 1.0=完全补偿)
  kt_gravity_.resize(n_dof_);
  if (config_.kt_gravity.size() == n_dof_) {
    for (size_t i = 0; i < n_dof_; ++i) {
      kt_gravity_(i) = config_.kt_gravity[i];
    }
  } else {
    kt_gravity_.setConstant(1.0);
    RCLCPP_WARN(node_->get_logger(),
                "kt_gravity 维度 (%zu) != n_dof (%zu), 已用 1.0 (完全补偿)",
                config_.kt_gravity.size(), n_dof_);
  }

  // LuGre 摩擦补偿: 为每个关节一个实例, enable/sigma0/sigma1/sigma2/Fs/Fc/vs 逐关节
  lugre_enabled_.assign(n_dof_, 0);
  lugre_friction_.clear();
  lugre_friction_.reserve(n_dof_);
  friction_torques_.resize(n_dof_);
  friction_torques_.setZero();
  tau_ff_applied_.resize(n_dof_);
  tau_ff_applied_.setZero();
  dq_filt_.resize(n_dof_);
  dq_filt_.setZero();
  for (size_t i = 0; i < n_dof_; ++i) {
    LuGreFrictionJoint::Params p;
    if (config_.lugre_friction.sigma0.size() == n_dof_)
      p.sigma0 = config_.lugre_friction.sigma0[i];
    if (config_.lugre_friction.sigma1.size() == n_dof_)
      p.sigma1 = config_.lugre_friction.sigma1[i];
    if (config_.lugre_friction.sigma2.size() == n_dof_)
      p.sigma2 = config_.lugre_friction.sigma2[i];
    if (config_.lugre_friction.Fs.size() == n_dof_)
      p.Fs = config_.lugre_friction.Fs[i];
    if (config_.lugre_friction.Fc.size() == n_dof_)
      p.Fc = config_.lugre_friction.Fc[i];
    if (config_.lugre_friction.vs.size() == n_dof_)
      p.vs = config_.lugre_friction.vs[i];
    lugre_friction_.emplace_back(p);
    if (config_.lugre_friction.enable.size() == n_dof_)
      lugre_enabled_[i] = (config_.lugre_friction.enable[i] > 0.5) ? 1 : 0;
  }
  // 至少一个关节启用 -> 打印一次汇总
  int n_enabled = 0;
  for (size_t i = 0; i < n_dof_; ++i) if (lugre_enabled_[i]) ++n_enabled;
  if (n_enabled > 0) {
    RCLCPP_INFO(node_->get_logger(),
                "LuGre 摩擦补偿启用: %d/%zu 关节, dq_lpf_alpha=%.3f",
                n_enabled, n_dof_, config_.lugre_friction.dq_lpf_alpha);
  } else {
    RCLCPP_INFO(node_->get_logger(), "LuGre 摩擦补偿禁用 (所有关节 enable<0.5)");
  }
  // 零空间期望构型 (target/filtered), 维度 = n_dof_
  //   init 时先置零, enable() 时会捕获当前关节角覆盖
  q_d_nullspace_target_.resize(n_dof_);
  q_d_nullspace_filtered_.resize(n_dof_);
  q_d_nullspace_target_.setZero();
  q_d_nullspace_filtered_.setZero();
  q_ik_last_.resize(n_dof_);
  q_ik_last_.setZero();
  nullspace_torques_.resize(n_dof_);
  nullspace_torques_.setZero();
  twist_compensation_.resize(n_dof_);
  accel_feedforward_.resize(n_dof_);
  impedance_torques_.resize(n_dof_);
  g_sub_.resize(n_dof_);
  g_sub_.setZero();
  g_sub_at_enable_.resize(n_dof_);
  g_sub_at_enable_.setZero();

  // 4. ROS 接口
  auto qos_lowlatency = rclcpp::QoS(1);
  qos_lowlatency.best_effort().durability_volatile();
  qos_lowlatency.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

  // status publisher: 用 ee_link 区分左右臂, 避免命名冲突
  const std::string ns = "~/impedance/" + config_.ee_link;
  status_pub_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>(ns + "/status",
                                                                  qos_lowlatency);
  // effort_commands_ 单独话题 (n_dof_ 维, 最终下发力矩, 含滤波与限幅)
  //   拆分发布原因: 主 status 数组过长时 PlotJuggler 等工具可能无法显示 data 字段
  effort_pub_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>(ns + "/effort_cmds",
                                                                  qos_lowlatency);
  // 状态数组布局 (Float64MultiArray, 索引连续, 总维度 ≤ 100):
  //   [0..5]   能量/功率/耗散标量 (mechanical_energy - H, dq·tau, twist·wrench,
  //                                H, dH/dt, twist_dev·impedance_wrench)
  //   [6..11]  pose_deviation     (6)
  //   [12..17] sensor_wrench_at_tcp_ (6, 基坐标系下, 已扣除工具重力与偏置, 参考点=TCP原点)
  //   [18..20] desired_position_  (3, 滤波后末端期望位置)
  //   [21..23] position_d_target_ (3, 上位机下发末端期望位置 target)
  //   [24..29] desired_twist_    (6, 滤波后末端期望速度)
  //   [30..35] desired_twist_target_ (6, 上位机下发末端期望速度 target)
  //   [36..38] p_tcp_in_base_    (3, 末端 TCP 在 base 系下的实际位置)
  //   [39..41] actual_rpy        (3, 末端 TCP 实际姿态 [roll,pitch,yaw], xyz 约定)
  //   [42..44] rpy_d_target      (3, 上位机下发期望姿态 target 转 RPY [roll,pitch,yaw])
  //   [45..47] desired_rpy       (3, 滤波后期望姿态 desired_quaternion_ 转 RPY [roll,pitch,yaw])
  //   [48..48+n_dof_-1]               robot_q_               (n_dof_, 实际关节角)
  //   [48+n_dof_..48+2*n_dof_-1]      q_d_nullspace_filtered_ (n_dof_, 滤波后零空间期望)
  //   [48+2*n_dof_..48+3*n_dof_-1]    accel_feedforward_     (n_dof_, 期望加速度前馈)
  //   [48+3*n_dof_..48+4*n_dof_-1]    impedance_torques_     (n_dof_, 阻抗力矩)
  //   [48+4*n_dof_..48+5*n_dof_-1]    nullspace_torques_     (n_dof_, 零空间力矩)
  //   [48+5*n_dof_..48+6*n_dof_-1]    twist_compensation_    (n_dof_, 速度补偿)
  //   [48+6*n_dof_..48+7*n_dof_-1]    g_sub_                 (n_dof_, pinocchio 重力项)
  // 注: twist_deviation_ / accel_deviation_ 已移除 (维度 ≤100 约束)
  //     effort_commands_ / friction_torques_ / tau_ff / tau_integral 单独发布到
  //     ~/effort_cmds 话题, 避免主 status 数组过长
  // 维度校验: 48 + 7*n_dof_ ≤ 48 + 7*7 = 97 ≤ 100 (要求 n_dof_ ≤ 7)
  status_msg_.data.resize(48 + 7 * n_dof_, 0.0);

  reference_sub_ = node_->create_subscription<ReferenceType>(
      ns + "/reference", qos_lowlatency,
      [this](const ReferenceType::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(rt_reference_mutex_);
          rt_reference_ = *msg;
        }
        rt_reference_valid_.store(true, std::memory_order_release);
        blank_reference_ = false;
      });

  // 零空间期望构型缺省值 (上位机不下发时使用, 在 enable() 中也会重新设置)
  // 已在 configureRobotModel() 之后 resize 并填充 default 值

  // 5. 初始化为零
  robot_q_.setZero();
  robot_dq_.setZero();
  robot_efforts_.setZero();
  effort_commands_.setZero();
  pose_deviation_.setZero();
  twist_deviation_.setZero();
  accel_deviation_.setZero();
  desired_pose_accel_.setZero();
  desired_quaternion_.setIdentity();
  desired_position_.setZero();
  desired_twist_.setZero();
  actual_pose_.setZero();
  actual_twist_.setZero();
  actual_twist_last_.setZero();
  actual_accel_.setZero();
  sensor_wrench_.setZero();
  impedance_wrench_.setZero();
  external_wrench_.setZero();
  has_external_wrench_ = false;
  estimated_wrench_.setZero();

  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(),
              "CartesianImpedanceController initialized: nq=%zu, n_dof=%zu, ee=%s",
              n_q_full_, n_dof_, config_.ee_link.c_str());
  return true;
}

bool CartesianImpedanceController::configureRobotModel() {
  try {
    pinocchio::urdf::buildModel(config_.urdf_path, robot_model_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "URDF parse failed: %s", e.what());
    return false;
  }
  robot_data_ = std::make_shared<pinocchio::Data>(robot_model_);
  n_q_full_ = robot_model_.nq;

  if (!robot_model_.existFrame(config_.ee_link)) {
    RCLCPP_ERROR(node_->get_logger(), "Frame '%s' not found in URDF",
                 config_.ee_link.c_str());
    return false;
  }
  ee_frame_ = robot_model_.getFrameId(config_.ee_link);

  // 解析 controlled 关节在 q_full 和 v (nv) 中的索引
  controlled_q_idx_.clear();
  controlled_v_idx_.clear();
  controlled_q_idx_.reserve(config_.controlled_joint_names.size());
  controlled_v_idx_.reserve(config_.controlled_joint_names.size());
  for (const auto& jname : config_.controlled_joint_names) {
    if (!robot_model_.existJointName(jname)) {
      RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found in URDF", jname.c_str());
      return false;
    }
    pinocchio::JointIndex jid = robot_model_.getJointId(jname);
    controlled_q_idx_.push_back(robot_model_.idx_qs[jid]);
    controlled_v_idx_.push_back(robot_model_.idx_vs[jid]);
  }
  n_dof_ = controlled_q_idx_.size();
  q_integral_accum_.setZero(n_dof_);
  tau_integral_.setZero(n_dof_);
  integral_gain_joint_.setZero(n_dof_);
  integral_limit_joint_.setZero(n_dof_);
  for (size_t i = 0; i < n_dof_; ++i) {
    integral_gain_joint_(i) =
        (i < config_.integral_gain_joint.size())
            ? config_.integral_gain_joint[i] : 0.0;
    integral_limit_joint_(i) =
        (i < config_.integral_limit_joint.size())
            ? config_.integral_limit_joint[i] : 2.0;
  }
  RCLCPP_INFO(node_->get_logger(),
              "Steady-state elimination: Kq=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] "
              "lim=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] "
              "F_thresh=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] "
              "leakage=%.2f alpha=%.2f sigma=%.4f "
              "Kf=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] ff_lpf=%.2f",
              integral_gain_joint_(0), integral_gain_joint_(1),
              integral_gain_joint_(2), integral_gain_joint_(3),
              integral_gain_joint_(4), integral_gain_joint_(5),
              integral_gain_joint_(6),
              integral_limit_joint_(0), integral_limit_joint_(1),
              integral_limit_joint_(2), integral_limit_joint_(3),
              integral_limit_joint_(4), integral_limit_joint_(5),
              integral_limit_joint_(6),
              integral_force_threshold_(0), integral_force_threshold_(1),
              integral_force_threshold_(2), integral_force_threshold_(3),
              integral_force_threshold_(4), integral_force_threshold_(5),
              integral_leakage_, integral_gain_alpha_, integral_gain_sigma_,
              force_feedback_gain_.diagonal()(0), force_feedback_gain_.diagonal()(1),
              force_feedback_gain_.diagonal()(2), force_feedback_gain_.diagonal()(3),
              force_feedback_gain_.diagonal()(4), force_feedback_gain_.diagonal()(5),
              force_ff_lpf_alpha_);

  // 调试: 打印 pinocchio 模型的关节索引映射, 用于排查 q_full 填充顺序
  // 与 controlled_q_idx_ 是否一致
  RCLCPP_INFO(node_->get_logger(),
              "[Impedance %s] nq=%zu nv=%zu, ee_frame='%s' (id=%d), n_dof=%zu",
              config_.ee_link.c_str(), robot_model_.nq, robot_model_.nv,
              config_.ee_link.c_str(), ee_frame_, n_dof_);
  for (size_t i = 0; i < n_dof_; ++i) {
    RCLCPP_INFO(node_->get_logger(),
                "[Impedance %s] controlled[%zu] joint='%s' q_idx=%d v_idx=%d",
                config_.ee_link.c_str(), i,
                config_.controlled_joint_names[i].c_str(),
                controlled_q_idx_[i], controlled_v_idx_[i]);
  }
  // 同时打印 pinocchio model 中所有关节的索引, 用于对比
  for (pinocchio::JointIndex jid = 1; jid < robot_model_.joints.size(); ++jid) {
    const auto& jname = robot_model_.names[jid];
    if (jname != "universe" && jname != "root_joint") {
      RCLCPP_INFO(node_->get_logger(),
                  "[Impedance %s] pinocchio joint '%s' idx_q=%d idx_v=%d nq=%d",
                  config_.ee_link.c_str(), jname.c_str(),
                  robot_model_.idx_qs[jid], robot_model_.idx_vs[jid],
                  robot_model_.joints[jid].nq());
    }
  }
  return true;
}

int CartesianImpedanceController::getJointQIdx(const std::string& joint_name) const {
  if (!robot_model_.existJointName(joint_name)) return -1;
  pinocchio::JointIndex jid = robot_model_.getJointId(joint_name);
  return robot_model_.idx_qs[jid];
}

int CartesianImpedanceController::getJointVIdx(const std::string& joint_name) const {
  if (!robot_model_.existJointName(joint_name)) return -1;
  pinocchio::JointIndex jid = robot_model_.getJointId(joint_name);
  return robot_model_.idx_vs[jid];
}

const VectorXd& CartesianImpedanceController::computeGravityLive(
    const VectorXd& q_full, const VectorXd& dq_full) {
  if (!initialized_) {
    if (g_sub_live_.size() != n_dof_) g_sub_live_.setZero(n_dof_);
    return g_sub_live_;
  }
  // 缓存关节状态并计算重力
  q_full_cache_ = q_full;
  dq_full_cache_ = dq_full;
  if (efforts_full_cache_.size() < q_full.size()) {
    efforts_full_cache_ = VectorXd::Zero(q_full.size());
  }
  updateRobot();
  if (g_sub_live_.size() != n_dof_) g_sub_live_.resize(n_dof_);
  for (size_t i = 0; i < n_dof_; ++i) {
    g_sub_live_(i) = kt_gravity_(i) * robot_data_->g(controlled_v_idx_[i]);
  }
  return g_sub_live_;
}

void CartesianImpedanceController::updateRobot() {
  // 提取控制关节子集到 robot_q_, robot_dq_, robot_efforts_
  // 注意: pinocchio 的 q 索引按 idx_qs, dq 按 idx_v. 单 DoF revolute 关节 q_idx == v_idx
  for (size_t i = 0; i < n_dof_; ++i) {
    robot_q_(i) = q_full_cache_(controlled_q_idx_[i]);
    robot_dq_(i) = dq_full_cache_(controlled_v_idx_[i]);
    if (has_effort_states_) {
      robot_efforts_(i) = efforts_full_cache_(controlled_v_idx_[i]);
    }
  }

  // computeAllTerms 需要 full q, dq
  pinocchio::computeAllTerms(robot_model_, *robot_data_, q_full_cache_,
                             dq_full_cache_);
  // 对称化 M (computeAllTerms 仅填上三角)
  robot_data_->M.triangularView<Eigen::StrictlyLower>() =
      robot_data_->M.transpose().triangularView<Eigen::StrictlyLower>();

  // computeAllTerms 不会自动更新 data.oMf[frame_id], 必须显式调用
  //   updateFramePlacements 才能让后续 oMf[ee_frame_] 读到真实位姿
  //   否则 oMf[ee_frame_] 一直是默认单位变换, 导致 enable()/update() 中
  //   p_ee_in_base 恒为 0, default_position_ 退化为 t_ee_in_tcp_ (yaml 平移部分)
  pinocchio::updateFramePlacements(robot_model_, *robot_data_);
}

void CartesianImpedanceController::updateDeviationAndReference() {
  // 完整雅克比 (6, nq_full)
  pinocchio::computeFrameJacobian(robot_model_, *robot_data_, q_full_cache_,
                                   ee_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
                                   jacobian_full_);

  // 提取控制关节子集列 (雅克比按 v 索引)
  for (size_t i = 0; i < n_dof_; ++i) {
    jacobian_.col(i) = jacobian_full_.col(controlled_v_idx_[i]);
  }

  // 将 ee_link Jacobian 变换到 TCP Jacobian (刚体偏移修正)
  //   v_tcp = v_ee - skew(r) * w_ee,  其中 r = R_ee_base * t_ee_in_tcp (base 系下)
  //   => J_tcp.topRows(3) = J_ee.topRows(3) - skew(r) * J_ee.bottomRows(3)
  //      J_tcp.bottomRows(3) = J_ee.bottomRows(3)
  //   若 tcp_to_ee 未配置 (默认单位变换), t_ee_in_tcp_ = 0, 变换为 noop (向后兼容)
  //   注: jacobian_dt_ 同样变换 (近似, 忽略 dT/dt 项, 慢运动下误差可忽略)
  applyTcpOffsetToJacobian(jacobian_);

  actual_twist_.noalias() = jacobian_ * robot_dq_;

  // 末端加速度: 数值差分 + 低通滤波
  actual_accel_ = acc_lpf_alpha_ / delta_t_ * (actual_twist_ - actual_twist_last_) +
                  (1.0 - acc_lpf_alpha_) * actual_accel_;
  actual_twist_last_ = actual_twist_;

  // 读取参考 (非阻塞: 用 try_lock, 失败则保持上一帧参考)
  bool has_ref = rt_reference_valid_.load(std::memory_order_acquire);
  if (has_ref) {
    {
      std::lock_guard<std::mutex> lock(rt_reference_mutex_);
      reference_ = rt_reference_;
    }
    // 解析上位机下发的参考到 target 状态 (后续 updateFilteredReference 平滑)
    position_d_target_.x() = reference_.pose.position.x;
    position_d_target_.y() = reference_.pose.position.y;
    position_d_target_.z() = reference_.pose.position.z;
    orientation_d_target_.w() = reference_.pose.orientation.w;
    orientation_d_target_.x() = reference_.pose.orientation.x;
    orientation_d_target_.y() = reference_.pose.orientation.y;
    orientation_d_target_.z() = reference_.pose.orientation.z;
    orientation_d_target_.normalize();

    desired_twist_target_(0) = reference_.pose_twist.linear.x;
    desired_twist_target_(1) = reference_.pose_twist.linear.y;
    desired_twist_target_(2) = reference_.pose_twist.linear.z;
    desired_twist_target_(3) = reference_.pose_twist.angular.x;
    desired_twist_target_(4) = reference_.pose_twist.angular.y;
    desired_twist_target_(5) = reference_.pose_twist.angular.z;

    desired_pose_accel_target_(0) = reference_.pose_accel.linear.x;
    desired_pose_accel_target_(1) = reference_.pose_accel.linear.y;
    desired_pose_accel_target_(2) = reference_.pose_accel.linear.z;
    desired_pose_accel_target_(3) = reference_.pose_accel.angular.x;
    desired_pose_accel_target_(4) = reference_.pose_accel.angular.y;
    desired_pose_accel_target_(5) = reference_.pose_accel.angular.z;

    // 零空间期望构型 (msg.q_d_nullspace 是 array<double,7>)
    for (size_t i = 0; i < n_dof_ && i < 7; ++i) {
      q_d_nullspace_target_(i) = reference_.q_d_nullspace[i];
    }
    has_nullspace_target_ = true;
    has_pose_target_ = true;
  } else {
    // 无外部参考: 使用使能时刻捕获的默认位姿作为 target (hold 模式)
    position_d_target_ = default_position_;
    orientation_d_target_ = default_quaternion_;
    desired_twist_target_.setZero();
    desired_pose_accel_target_.setZero();
    // 零空间 target 保持上一次值 (initial = enable 时的当前关节角)
  }

  // 参考信号滤波 (1 阶低通, 匹配低频下发与 1kHz 控制)
  updateFilteredReference();

  // 使用滤波后的位姿/速度/加速度作为控制参考
  desired_position_ = position_d_filtered_;
  desired_quaternion_ = orientation_d_filtered_;
  desired_quaternion_.normalize();
  desired_twist_ = desired_twist_filtered_;
  desired_pose_accel_ = desired_pose_accel_filtered_;

  // 偏差: 实际 TCP 位姿 - 期望 TCP 位姿 (上位机参考应在 TCP 系下表达)
  //   TCP 在 base 系下的位姿: T_base_tcp = T_base_ee * T_ee_tcp
  //   若 tcp_to_ee 未配置 (默认单位变换), 退化为 ee_link 偏差 (向后兼容)
  const Eigen::Vector3d& p_ee_in_base = robot_data_->oMf[ee_frame_].translation();
  const Eigen::Matrix3d& R_ee_in_base = robot_data_->oMf[ee_frame_].rotation();
  p_tcp_in_base_ = p_ee_in_base + R_ee_in_base * t_ee_in_tcp_;
  R_tcp_in_base_ = R_ee_in_base * R_ee_to_tcp_;
  pose_deviation_.head<3>().noalias() = p_tcp_in_base_ - desired_position_;
  pose_deviation_.tail<3>().noalias() = pinocchio::log3(
      R_tcp_in_base_ * desired_quaternion_.toRotationMatrix().transpose());

  twist_deviation_.noalias() = actual_twist_ - desired_twist_;
}

void CartesianImpedanceController::updateEffortCommands() {
  // 雅克比阻尼伪逆 (DLS / Tikhonov):
  //   J 是 (6, n_dof_), pinv(J) 是 (n_dof_, 6)
  //   对 7DoF 冗余臂, J 是 6x7, pinv(J) 是 7x6
  //   λ>0 在奇异点附近平滑, 避免力矩跳变; λ=0 退化为标准 Moore-Penrose
  const double pinv_lambda = config_.pinv_damping;
  dampedPseudoInverse(jacobian_, &jacobian_pinv_, pinv_lambda);
  // J^T 的伪逆: J^T 是 (n_dof_, 6), pinv(J^T) 是 (6, n_dof_)
  //   注意: 仅当 n_dof_=6 时 J^T 才是方阵可逆.
  //   对 7DoF 冗余臂 J^T 是 7x6 不可逆, 用阻尼伪逆 (与 N = I - J^T*pinv(J^T) 兼容)
  dampedPseudoInverse(jacobian_.transpose(), &jacobianT_pinv_, pinv_lambda);

  // 估算外部力 (可选, 用于监控)
  if (has_effort_states_) {
    estimated_wrench_.noalias() =
        jacobianT_pinv_ * (robot_efforts_ - effort_commands_);
  }

  // 计算 dJ/dt (frame jacobian time variation)
  // 注意: pinocchio::getFrameJacobianTimeVariation 需要 full q 已 computeAllTerms
  // 我们在 updateRobot() 已 computeAllTerms, 这里取控制关节子集
  pinocchio::Data::Matrix6x jacobian_dt_full;
  jacobian_dt_full.resize(6, robot_model_.nv);
  pinocchio::getFrameJacobianTimeVariation(robot_model_, *robot_data_, ee_frame_,
                                            pinocchio::LOCAL_WORLD_ALIGNED,
                                            jacobian_dt_full);
  for (size_t i = 0; i < n_dof_; ++i) {
    jacobian_dt_.col(i) = jacobian_dt_full.col(controlled_v_idx_[i]);
  }
  // 同步把 dJ/dt 变换到 TCP 系 (近似: 忽略 dT/dt 项, 慢运动下误差可忽略)
  applyTcpOffsetToJacobian(jacobian_dt_);

  accel_deviation_.noalias() = actual_accel_ - desired_pose_accel_;

  // 提取控制关节子矩阵 M_sub (n_dof_, n_dof_), C_sub, g_sub
  Eigen::MatrixXd M_sub(n_dof_, n_dof_);
  for (size_t i = 0; i < n_dof_; ++i) {
    for (size_t j = 0; j < n_dof_; ++j) {
      M_sub(i, j) = robot_data_->M(controlled_v_idx_[i], controlled_v_idx_[j]);
    }
  }
  Eigen::MatrixXd C_sub(n_dof_, n_dof_);
  for (size_t i = 0; i < n_dof_; ++i) {
    for (size_t j = 0; j < n_dof_; ++j) {
      C_sub(i, j) = robot_data_->C(controlled_v_idx_[i], controlled_v_idx_[j]);
    }
  }
  for (size_t i = 0; i < n_dof_; ++i) {
    g_sub_(i) = kt_gravity_(i) * robot_data_->g(controlled_v_idx_[i]);
  }

  // LuGre 摩擦补偿前馈
  //   输入: 速度 (低通滤波后) -> 平滑 bristle 状态
  //   输出: 摩擦力矩估计 (正号: 抵消摩擦, 即 tau_desired_ += friction_torques_)
  if (lugre_friction_.size() == n_dof_) {
    const double alpha = config_.lugre_friction.dq_lpf_alpha;
    const double dt = 1.0 / config_.update_frequency;
    for (size_t i = 0; i < n_dof_; ++i) {
      if (i < lugre_enabled_.size() && !lugre_enabled_[i]) {
        friction_torques_(i) = 0.0;
        dq_filt_(i) = 0.0;
        continue;
      }
      const double dq_raw = robot_dq_(i);
      // 一阶低通: dq_filt = alpha*dq_raw + (1-alpha)*dq_filt
      dq_filt_(i) = alpha * dq_raw + (1.0 - alpha) * dq_filt_(i);
      friction_torques_(i) = lugre_friction_[i].step(dq_filt_(i), dt);
    }
  } else {
    friction_torques_.setZero();
  }

  // 操作空间惯量矩阵: Lambda = (J * M^-1 * J^T)^-1  (6x6)
  // 惯量整形公式需要 J^T * Lambda (7x6), 用于:
  //   tau = J^T * Lambda * M_d^-1 * (F_imp + F_ext) - J^T * F_ext
  // 恒等式: J^T * Lambda = M * J^#, 其中 J^# = M^-1 * J^T * Lambda
  //         是惯量加权伪逆.
  // 注: 对冗余臂 (n_dof=7), 未加权伪逆 J^+ != J^#,
  //     因此不能用 M * J^+ 简化公式 (Otis 法仅适用于 n_dof=6 方阵 J).
  //     这里直接通过 M^-1 * J^T 解算, 然后构造 Lambda.
  {
    // 解 M^-1 * J^T (7x6): 用 LDLT 分解, 比 M^-1 直接求逆更稳定
    Eigen::MatrixXd M_inv_JT =
        M_sub.ldlt().solve(jacobian_.transpose());  // 7x6
    // Lambda_inv = J * M^-1 * J^T (6x6, 对冗余臂正定)
    Eigen::MatrixXd Lambda_inv = jacobian_ * M_inv_JT;
    // Lambda = (Lambda_inv)^-1
    Eigen::MatrixXd Lambda = Lambda_inv.inverse();
    // J^T * Lambda (7x6) — 这是 jsim_jpinv_ 的正确值
    jsim_jpinv_ = jacobian_.transpose() * Lambda;
    // 缓存 Lambda 对角元用于诊断 (操作空间等效惯量)
    lambda_diag_ = Lambda.diagonal();
  }
  jsim_jpinv_dj_ = jsim_jpinv_ * jacobian_dt_;

  actual_inertia_ = jacobianT_pinv_ * jsim_jpinv_;

  // 阻抗 wrench: F = K*dx + D*dv (反向作用, TCP 系下表达)
  //   若有外部注入 wrench (阶段 2 物体级阻抗), 直接使用外部值, 跳过 K*dx+D*dv.
  if (has_external_wrench_) {
    impedance_wrench_.noalias() = external_wrench_;
  } else {
    impedance_wrench_.noalias() =
        (desired_stiffness_ * pose_deviation_ + desired_damping_ * twist_deviation_) * (-1);
  }

  // 诊断打印: 确认 K/D/inertia_shaping 的运行时实际值 (仅首帧打印, 避免刷屏)
  // if (!diag_params_printed_) {
  //   RCLCPP_INFO(node_->get_logger(),
  //               "[ImpedanceDiag %s] inertia_shaping=%d K=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] "
  //               "D=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] "
  //               "M_sub_diag=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]",
  //               config_.ee_link.c_str(), inertia_shaping_ ? 1 : 0,
  //               desired_stiffness_.diagonal()(0), desired_stiffness_.diagonal()(1),
  //               desired_stiffness_.diagonal()(2), desired_stiffness_.diagonal()(3),
  //               desired_stiffness_.diagonal()(4), desired_stiffness_.diagonal()(5),
  //               desired_damping_.diagonal()(0), desired_damping_.diagonal()(1),
  //               desired_damping_.diagonal()(2), desired_damping_.diagonal()(3),
  //               desired_damping_.diagonal()(4), desired_damping_.diagonal()(5),
  //               M_sub(0,0), M_sub(1,1), M_sub(2,2), M_sub(3,3),
  //               M_sub(4,4), M_sub(5,5), M_sub(6,6));
  //   diag_params_printed_ = true;
  // }

  // 把 sensor_wrench_ (at ee 原点) 变换到 TCP 原点, 与 TCP 系阻抗公式一致
  //   F 不变 (自由矢量), M_tcp = M_ee - r × F,  其中 r = R_ee_base * t_ee_in_tcp
  //   (把参考点从 ee 移到 TCP, r 是 TCP 相对 ee 在 base 系下的位置)
  //   若 t_ee_in_tcp_ 范数 < 1e-9, 视为无偏移, 直接用 sensor_wrench_ (向后兼容)
  sensor_wrench_at_tcp_ = sensor_wrench_;
  if (t_ee_in_tcp_.norm() > 1e-9) {
    const Eigen::Vector3d r = ee_rotation_in_base_ * t_ee_in_tcp_;
    sensor_wrench_at_tcp_.tail<3>().noalias() -= r.cross(sensor_wrench_.head<3>());
  }

  // === 静差消除: 关节空间泄漏积分 ===
  //   笛卡尔偏差映射到关节空间: dq_error = J^+ * dx
  //   关节空间泄漏积分: q_accum = q_accum*(1-σ*dt) + dq_error*dt*(-1)
  //   输出直接为关节力矩: tau_integral = Kq * q_accum
  //   绕过 J^T, 各关节独立积分, 无运动耦合振荡
  tau_integral_.setZero();
  // 检查是否有任一关节积分增益 > 0
  bool integral_enabled = false;
  for (int i = 0; i < n_dof_; ++i) {
    if (integral_gain_joint_(i) > 1e-9) { integral_enabled = true; break; }
  }
  if (integral_enabled) {
    // 自由空间判定: 所有维度传感器力 < threshold 时积分
    bool free_space = true;
    // for (int i = 0; i < 6; ++i) {
    //   if (std::abs(sensor_wrench_at_tcp_(i)) >= integral_force_threshold_(i)) {
    //     free_space = false;
    //     break;
    //   }
    // }

    if (free_space) {
      Eigen::VectorXd dq_error = q_d_nullspace_filtered_ - robot_q_;
      // 泄漏积分: accum = accum*(1-σ*dt) + dq_error*dt
      q_integral_accum_ = q_integral_accum_ * (1.0 - integral_leakage_ * delta_t_)
                        + dq_error * delta_t_ * (1.0);
      // 逐关节 anti-windup (使用逐关节限幅)
      for (int i = 0; i < n_dof_; ++i) {
        const double lim = integral_limit_joint_(i);
        if (q_integral_accum_(i) > lim)       q_integral_accum_(i) = lim;
        else if (q_integral_accum_(i) < -lim) q_integral_accum_(i) = -lim;
      }
      // 逐关节非线性增益: 小误差放大, 克服静摩擦死区
      //   gain_boost = 1 + alpha * exp(-|dq_err|/sigma)
      //   误差→0 时增益放大 (1+alpha) 倍; 误差 >> sigma 时增益趋近 1
      Eigen::VectorXd gain_boost(n_dof_);
      for (int i = 0; i < n_dof_; ++i) {
        const double error_mag = std::abs(dq_error(i));
        gain_boost(i) = 1.0 + integral_gain_alpha_ * std::exp(-error_mag / integral_gain_sigma_);
      }
      // 逐关节积分力矩, 不经过 J^T
      tau_integral_ = (integral_gain_joint_.cwiseProduct(gain_boost))
                        .cwiseProduct(q_integral_accum_ + dq_error)
                    - integral_gain_joint_.cwiseProduct(robot_dq_) * 0.1;

    } else {
      q_integral_accum_.setZero();
      tau_integral_.setZero();
    }
  }

  // 方案2: 末端外力闭环反馈
  //   F_ff = Kf * (0 - sensor_wrench_at_tcp_) = -Kf * F_ext
  //   传感器残留偏差 (静摩擦/重力残差) 被闭环修正
  //   低通滤波避免噪声放大
  Vector6d F_force_ff = Vector6d::Zero();
  if (force_feedback_gain_.diagonal().norm() > 1e-9) {
    Vector6d F_raw = force_feedback_gain_ * (-sensor_wrench_at_tcp_);
    force_ff_filtered_ = force_ff_lpf_alpha_ * F_raw
                        + (1.0 - force_ff_lpf_alpha_) * force_ff_filtered_;
    F_force_ff = force_ff_filtered_;
  }

  // 力矩换算 (积分力矩绕过 J^T, 直接叠加到关节力矩)
  if (inertia_shaping_) {
    // 惯量整形: tau = J^+T * M_d^-1 * (F_imp + F_ff + F_ext) - J^T * F_ext
    impedance_torques_.noalias() =
        jsim_jpinv_ * desired_inertia_inv_ *
        (impedance_wrench_ + F_force_ff + sensor_wrench_at_tcp_);
    impedance_torques_.noalias() -= jacobian_.transpose() * sensor_wrench_at_tcp_;
  } else {
    // 纯阻尼: tau = J^T * (F_imp + F_ff)
    impedance_torques_.noalias() =
        jacobian_.transpose() * (impedance_wrench_ + F_force_ff);
    desired_inertia_ = actual_inertia_;  // 用于 Hamiltonian
  }
  // 关节空间积分力矩直接叠加 (绕过 J^T, 无运动耦合)
  impedance_torques_.noalias() += tau_integral_;

  // 零空间力矩: 投影到 J^T 零空间, 不影响末端跟踪
  //   tau_nullspace = (I - J^T * pinv(J^T)) * (K_n*(q_d - q) - D_n*dq)
  //   注: jacobianT_pinv_ = pinv(J^T), 维度 (6, n_dof_)
  //       N = I_n - J^T * pinv(J^T) 是 (n_dof_, n_dof_) 投影矩阵
  if (nullspace_stiffness_ > 0.0) {
    Eigen::VectorXd tau_ns_raw =
        nullspace_stiffness_ * (q_d_nullspace_filtered_ - robot_q_)
        - nullspace_damping_ * robot_dq_;
    // 投影: N = I_n - J^T * (J^T)^+ (7x7)
    //   N * tau_raw 提取不影响末端的零空间分量 (运动学零空间投影)
    Eigen::MatrixXd N_null = Eigen::MatrixXd::Identity(n_dof_, n_dof_)
                             - jacobian_.transpose() * jacobianT_pinv_;
    nullspace_torques_ = N_null * tau_ns_raw;

    // 诊断: 每秒打印一次零空间计算中间量
    // {
    //   const int64_t now_ns = node_->now().nanoseconds();
    //   if (ns_diag_ns_ == 0 || (now_ns - ns_diag_ns_) > 1'000'000'000) {
    //     RCLCPP_INFO(node_->get_logger(),
    //                 "[NullspaceDiag %s] K=%.1f D=%.1f |q_d-q|=%.4f |dq|=%.4f "
    //                 "|tau_raw|=%.4f |N_null|=%.4f |tau_ns|=%.4f",
    //                 config_.ee_link.c_str(),
    //                 nullspace_stiffness_, nullspace_damping_,
    //                 (q_d_nullspace_filtered_ - robot_q_).norm(),
    //                 robot_dq_.norm(), tau_ns_raw.norm(),
    //                 N_null.norm(), nullspace_torques_.norm());
    //     ns_diag_ns_ = now_ns;
    //   }
    // }
  } else {
    nullspace_torques_.setZero();
  }

  // 前馈: 加速度 + 科氏/重力补偿
  accel_feedforward_ = jsim_jpinv_ * desired_pose_accel_;
  twist_compensation_.noalias() = (C_sub - jsim_jpinv_dj_) * robot_dq_;

  // 摩擦补偿符号说明:
  //   LuGre step() 返回的是"需要施加的补偿力矩" (与运动方向同号),
  //   即 v>0 时输出正值, 电机施加正向力矩抵消摩擦 (摩擦沿 -v 方向).
  //   因此 tau_desired_ += friction_torques_.
  tau_desired_ = accel_feedforward_ + impedance_torques_ + nullspace_torques_
               + twist_compensation_ + g_sub_ + friction_torques_;

  // 低通滤波
  effort_commands_ = cmd_lpf_alpha_ * tau_desired_ + (1.0 - cmd_lpf_alpha_) * effort_commands_;

  // 关节力矩限幅 (对称限幅: |tau| <= effort_limit_per_joint_)
  //   防止阻抗在偏差突增 (如碰撞, 参考位姿突变) 时输出过大力矩
  //   限幅在低通滤波之后做, 确保注入到 EtherCAT 的最终值不超限
  for (size_t i = 0; i < n_dof_; ++i) {
    const double lim = effort_limit_per_joint_(i);
    if (lim > 0.0) {  // lim=0 视为不限制
      double& v = effort_commands_(i);
      if (v > lim)       v = lim;
      else if (v < -lim) v = -lim;
    }
  }

  computeHamiltonian();
}

void CartesianImpedanceController::computeHamiltonian() {
  // H = 1/2 * dv^T * M_d * dv + 1/2 * dx^T * K * dx
  hamiltonian_ = twist_deviation_.transpose() * desired_inertia_ * twist_deviation_;
  hamiltonian_ += pose_deviation_.transpose() * desired_stiffness_ * pose_deviation_;
  hamiltonian_ = 0.5 * hamiltonian_;

  hamiltonian_filtered_ =
      cmd_lpf_alpha_ * hamiltonian_ + (1.0 - cmd_lpf_alpha_) * hamiltonian_filtered_;
  hamiltonian_derivative_ = (hamiltonian_filtered_ - hamiltonian_last_) / delta_t_;
  hamiltonian_last_ = hamiltonian_filtered_;
}

void CartesianImpedanceController::publishStatus() {
  if (!status_pub_) return;
  if (!status_pub_mutex_.try_lock()) return;  // 实时线程优先, 锁竞争则跳过
  std::lock_guard<std::mutex> guard(status_pub_mutex_, std::adopt_lock);

  status_msg_.data[0] = robot_data_->mechanical_energy - hamiltonian_filtered_;
  status_msg_.data[1] = robot_dq_.transpose() * effort_commands_;
  status_msg_.data[2] = actual_twist_.transpose() * sensor_wrench_at_tcp_;
  status_msg_.data[3] = hamiltonian_filtered_;
  status_msg_.data[4] = hamiltonian_derivative_;
  status_msg_.data[5] = twist_deviation_.transpose() * impedance_wrench_;
  for (int i = 0; i < 6; ++i) {
    status_msg_.data[6 + i] = pose_deviation_(i);
    // 调试跟踪: sensor_wrench_at_tcp (基坐标系, 已扣工具重力与偏置, 参考点=TCP原点)
    status_msg_.data[12 + i] = sensor_wrench_at_tcp_(i);
    // 调试跟踪: desired_twist / desired_twist_target (滤波后 vs 上位机 target)
    status_msg_.data[24 + i] = desired_twist_(i);
    status_msg_.data[30 + i] = desired_twist_target_(i);
  }
  // 调试跟踪: 末端期望位置 (滤波后 desired_position_ 与上位机 target)
  for (int i = 0; i < 3; ++i) {
    status_msg_.data[18 + i] = desired_position_(i);
    status_msg_.data[21 + i] = position_d_target_(i);
    // 调试跟踪: 末端 TCP 实际位置 (紧挨 actual_rpy, 便于对照)
    status_msg_.data[36 + i] = p_tcp_in_base_(i);
  }
  // 末端 TCP 实际姿态 RPY (xyz 内蕴欧拉角, 与脚本 from_euler('xyz', rpy) 约定一致)
  //   Eigen eulerAngles(0,1,2) 返回 [roll, pitch, yaw] 满足 R = Rx*Ry*Rz
  //   供 teleop 脚本从 status 读取当前姿态作为初始参考, 避免使能时参考跳变
  {
    const Eigen::Vector3d actual_rpy = R_tcp_in_base_.eulerAngles(0, 1, 2);
    status_msg_.data[39] = actual_rpy(0);  // roll  (X)
    status_msg_.data[40] = actual_rpy(1);  // pitch (Y)
    status_msg_.data[41] = actual_rpy(2);  // yaw   (Z)
  }
  // 末端期望姿态 RPY (target 与滤波后, 同 xyz 约定, 与 actual_rpy 对照便于调试)
  //   orientation_d_target_ / desired_quaternion_ 均为 Eigen::Quaterniond
  {
    Eigen::Quaterniond q_t = orientation_d_target_;
    q_t.normalize();
    const Eigen::Vector3d rpy_target = q_t.toRotationMatrix().eulerAngles(0, 1, 2);
    status_msg_.data[42] = rpy_target(0);  // roll
    status_msg_.data[43] = rpy_target(1);  // pitch
    status_msg_.data[44] = rpy_target(2);  // yaw

    Eigen::Quaterniond q_d = desired_quaternion_;
    q_d.normalize();
    const Eigen::Vector3d rpy_desired = q_d.toRotationMatrix().eulerAngles(0, 1, 2);
    status_msg_.data[45] = rpy_desired(0);  // roll
    status_msg_.data[46] = rpy_desired(1);  // pitch
    status_msg_.data[47] = rpy_desired(2);  // yaw
  }
  // 调试跟踪: 关节角 robot_q_ 与零空间期望 q_d_nullspace_filtered_
  const size_t offset_q = 48;
  for (size_t i = 0; i < n_dof_; ++i) {
    status_msg_.data[offset_q + i] = robot_q_(i);
    status_msg_.data[offset_q + n_dof_ + i] = q_d_nullspace_filtered_(i);
  }
  // 调试跟踪: 控制力矩分解 (updateEffortCommands 中各分量, 用于排查力矩比重)
  //   tau_desired_ = accel_feedforward_ + impedance_torques_ + nullspace_torques_
  //                + twist_compensation_ + g_sub_ + friction_torques_
  const size_t offset_tau = 48 + 2 * n_dof_;
  for (size_t i = 0; i < n_dof_; ++i) {
    status_msg_.data[offset_tau + 0 * n_dof_ + i] = accel_feedforward_(i);
    status_msg_.data[offset_tau + 1 * n_dof_ + i] = impedance_torques_(i);
    status_msg_.data[offset_tau + 2 * n_dof_ + i] = nullspace_torques_(i);
    status_msg_.data[offset_tau + 3 * n_dof_ + i] = twist_compensation_(i);
    status_msg_.data[offset_tau + 4 * n_dof_ + i] = g_sub_(i);
  }
  // effort_commands_ 与 friction_torques_ 联合发布到 ~/effort_cmds 话题
  //   布局: [0..n_dof_-1]=effort_commands_, [n_dof_..2*n_dof_-1]=friction_torques_
  //   避免主 status 消息数组过长导致 PlotJuggler 等工具显示异常
  if (effort_pub_ && effort_pub_mutex_.try_lock()) {
    std::lock_guard<std::mutex> guard(effort_pub_mutex_, std::adopt_lock);
    // 布局: [0..n-1]=effort_commands_, [n..2n-1]=friction_torques_,
    //        [2n..3n-1]=tau_ff_applied_ (tau_ff 测试模式实际下发力矩),
    //        [3n..4n-1]=tau_integral_ (关节空间积分力矩, n_dof_ 维),
    //        [4n..5n-1]=tau_desired_ (滤波前的期望力矩, n_dof_ 维),
    //        [5n..5n+5]=impedance_wrench_ (笛卡尔阻抗 wrench, 6 维: Fx,Fy,Fz,Mx,My,Mz)
    const size_t off_wrench = 5 * n_dof_;
    effort_msg_.data.resize(off_wrench + 6);
    for (size_t i = 0; i < n_dof_; ++i) {
      effort_msg_.data[i]                 = effort_commands_(i);
      effort_msg_.data[n_dof_ + i]        = friction_torques_(i);
      effort_msg_.data[2 * n_dof_ + i]    = tau_ff_applied_(i);
      effort_msg_.data[3 * n_dof_ + i]    = tau_integral_(i);
      effort_msg_.data[4 * n_dof_ + i]    = tau_desired_(i);
    }
    for (int i = 0; i < 6; ++i) {
      effort_msg_.data[off_wrench + i] = impedance_wrench_(i);
    }
    effort_pub_->publish(effort_msg_);
  }

  // 诊断: 每秒打印一次 K/D/impedance_wrench/impedance_torques 的实际值,
  // 确认 stiffness/damping=0 时 impedance_torques 是否真为 0
  // 注: 用成员变量 last_diag_ns_ 替代函数 static, 避免左右臂实例共享同一计时器
  // const int64_t now_ns = node_->now().nanoseconds();
  // if (last_diag_ns_ == 0 || (now_ns - last_diag_ns_) > 1'000'000'000) {
  //   // 计算惯量整形有效缩放因子: Lambda * M_d^-1 的对角元
  //   // 当 inertia_shaping=true 时, 阻抗力矩 = J^T * (Lambda/M_d) * F_imp
  //   // Lambda/M_d << 1 会导致弹簧效果明显减弱
  //   if (inertia_shaping_) {
  //     RCLCPP_INFO(node_->get_logger(),
  //                 "[ImpedanceDiag %s] K_norm=%.4f D_norm=%.4f |dx|=%.4f |dv|=%.4f "
  //                 "|F_imp|=%.4f |tau_imp|=%.4f inertia_shaping=%d "
  //                 "Lambda=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] "
  //                 "Md=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] "
  //                 "scale=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f] "
  //                 "|F_ext|=%.4f",
  //                 config_.ee_link.c_str(),
  //                 desired_stiffness_.diagonal().norm(),
  //                 desired_damping_.diagonal().norm(),
  //                 pose_deviation_.norm(), twist_deviation_.norm(),
  //                 impedance_wrench_.norm(), impedance_torques_.norm(),
  //                 inertia_shaping_ ? 1 : 0,
  //                 lambda_diag_(0), lambda_diag_(1), lambda_diag_(2),
  //                 lambda_diag_(3), lambda_diag_(4), lambda_diag_(5),
  //                 desired_inertia_.diagonal()(0), desired_inertia_.diagonal()(1),
  //                 desired_inertia_.diagonal()(2), desired_inertia_.diagonal()(3),
  //                 desired_inertia_.diagonal()(4), desired_inertia_.diagonal()(5),
  //                 lambda_diag_(0) * desired_inertia_inv_.diagonal()(0),
  //                 lambda_diag_(1) * desired_inertia_inv_.diagonal()(1),
  //                 lambda_diag_(2) * desired_inertia_inv_.diagonal()(2),
  //                 lambda_diag_(3) * desired_inertia_inv_.diagonal()(3),
  //                 lambda_diag_(4) * desired_inertia_inv_.diagonal()(4),
  //                 lambda_diag_(5) * desired_inertia_inv_.diagonal()(5),
  //                 sensor_wrench_.norm());
  //   } else {
  //     RCLCPP_INFO(node_->get_logger(),
  //                 "[ImpedanceDiag %s] K_norm=%.4f D_norm=%.4f |dx|=%.4f |dv|=%.4f "
  //                 "|F_imp|=%.4f |tau_imp|=%.4f inertia_shaping=%d "
  //                 "|F_ext|=%.4f",
  //                 config_.ee_link.c_str(),
  //                 desired_stiffness_.diagonal().norm(),
  //                 desired_damping_.diagonal().norm(),
  //                 pose_deviation_.norm(), twist_deviation_.norm(),
  //                 impedance_wrench_.norm(), impedance_torques_.norm(),
  //                 inertia_shaping_ ? 1 : 0,
  //                 sensor_wrench_.norm());
  //   }
  //   last_diag_ns_ = now_ns;
  // }

  status_pub_->publish(status_msg_);
}

Vector6d CartesianImpedanceController::computeToolGravityInBase() const {
  // 工具重力在基坐标系下的 [F; M] (绕 ee_link 原点的力矩).
  //   F_g_B = [0, 0, -m*g]              基系下纯重力 (z 朝上, 重力沿 -z)
  //   r_tool_B = R_ee_in_base * r_tool_in_ee   质心在基系下的位置 (相对 ee 原点)
  //   M_g_B = r_tool_B × F_g_B           重力绕 ee 原点的力矩, 在基系下表示
  Vector6d gravity_wrench = Vector6d::Zero();
  if (tool_mass_ <= 0.0) return gravity_wrench;

  Eigen::Vector3d F_g_B(0.0, 0.0, -tool_mass_ * kGravityAccel);
  Eigen::Vector3d r_tool_B = ee_rotation_in_base_ * tool_com_in_ee_;
  Eigen::Vector3d M_g_B = r_tool_B.cross(F_g_B);
  gravity_wrench << F_g_B, M_g_B;
  return gravity_wrench;
}

void CartesianImpedanceController::update(const VectorXd& q_full,
                                           const VectorXd& dq_full,
                                           const VectorXd& efforts_full,
                                           const Vector6d& sensor_wrench,
                                           double dt) {
  if (!initialized_) return;
  q_full_cache_ = q_full;
  dq_full_cache_ = dq_full;
  efforts_full_cache_ = efforts_full;
  delta_t_ = (dt > 1e-6) ? dt : 0.001;

  updateRobot();  // 此后 robot_data_->oMf[ee_frame_] 已就绪

  // 末端 frame 在基坐标系下的旋转 (用于 sensor wrench 坐标系变换)
  ee_rotation_in_base_ = robot_data_->oMf[ee_frame_].rotation();

  // 1) 将 sensor wrench 从传感器自身坐标系变换到基坐标系
  //    实际硬件中传感器与 ee_link 通常不重合 (有 R_ee_sensor, t_ee_sensor):
  //      a) 力 (自由矢量): 直接用 R_sensor_base = R_ee_base * R_ee_sensor 旋转
  //      b) 力矩: 先用 R_sensor_base 旋转, 再平移到 ee 原点 (参考点变更公式):
  //         M_at_ee_in_base = M_at_sensor_in_base + (p_sensor - p_ee) × F_in_base
  //         其中 p_sensor - p_ee = R_ee_base * t_sensor_in_ee (在 base 系下表示)
  //    若 sensor_to_ee 未配置 (默认单位变换), 退化为旧实现 (直接用 ee 旋转)
  Vector6d wrench_in_base;
  const Eigen::Matrix3d R_sensor_base = ee_rotation_in_base_ * R_sensor_to_ee_;
  const Eigen::Vector3d F_in_base = R_sensor_base * sensor_wrench.head<3>();
  const Eigen::Vector3d M_in_base = R_sensor_base * sensor_wrench.tail<3>();
  // 平移修正: 把力矩参考点从 sensor 原点移到 ee 原点
  //   (M_at_ee = M_at_sensor - r × F  等价于  M_at_ee = M_at_sensor + (p_sensor - p_ee) × F)
  //   传感器原点相对 ee 原点在 base 系下的位置: r_sensor_in_base = R_ee_base * t_sensor_in_ee
  //   公式: M_at_ee_in_base = M_at_sensor_in_base + r_sensor_in_base × F_in_base
  //       (因 r 是从 ee 原点指向 sensor 原点的向量, 力矩参考点从 sensor 移到 ee 应加 r×F)
  const Eigen::Vector3d r_sensor_in_base = ee_rotation_in_base_ * t_sensor_in_ee_;
  wrench_in_base.head<3>() = F_in_base;
  wrench_in_base.tail<3>() = M_in_base + r_sensor_in_base.cross(F_in_base);

  // 2) 实时补偿末端工具重力 (基坐标系下计算, 任意姿态都准确)
  //    F_g_B 恒定, M_g_B = (R*com) × F_g_B 随姿态变化
  //    工具质心已从 tcp 系预转到 ee 系 (tool_com_in_ee_), 在 init() 中完成
  wrench_in_base -= computeToolGravityInBase();

  // 3) 减去使能时刻捕获的残余偏置 (基坐标系下表示)
  //    反映传感器零漂 + 未建模的固定偏置 (工具重力已通过 yaml 实时补偿)
  if (has_sensor_bias_) {
    wrench_in_base -= sensor_wrench_bias_in_base_;
  }
  sensor_wrench_ = wrench_in_base;

  updateDeviationAndReference();
  updateEffortCommands();
  publishStatus();
}

void CartesianImpedanceController::enable(const VectorXd& q_full,
                                          const VectorXd& dq_full,
                                          const Vector6d& sensor_wrench) {
  if (!initialized_) return;

  // 缓存使能时刻的关节状态
  q_full_cache_ = q_full;
  dq_full_cache_ = dq_full;
  efforts_full_cache_ = VectorXd::Zero(q_full.size());

  // 计算 FK, 此后 oMf[ee_frame_] 可用
  updateRobot();
  ee_rotation_in_base_ = robot_data_->oMf[ee_frame_].rotation();

  // 方案 E: 采样使能时刻的重力补偿 (用于 EtherCAT CiA402 使能序列预注入)
  //   updateRobot() 已调用 computeAllTerms, g 向量可用
  g_sub_at_enable_.resize(n_dof_);
  for (size_t i = 0; i < n_dof_; ++i) {
    g_sub_at_enable_(i) = kt_gravity_(i) * robot_data_->g(controlled_v_idx_[i]);
  }
  // 调试: 打印 g_sub_at_enable_ 采样值, 确认非零
  printf("[DBG-EN-G] ee=%s n_dof=%zu g_at_enable=[", config_.ee_link.c_str(), n_dof_);
  for (size_t i = 0; i < n_dof_; ++i) printf("%.3f ", g_sub_at_enable_(i));
  printf("] v_idx=[");
  for (size_t i = 0; i < n_dof_; ++i) printf("%d ", controlled_v_idx_[i]);
  printf("] g_full=[");
  for (int i = 0; i < robot_data_->g.size(); ++i) printf("%.3f ", robot_data_->g(i));
  printf("]\n");

  // 1) 捕获使能时刻的末端 TCP 位姿作为默认参考 (无外部参考时使用)
  //    TCP 在 base 系下的位姿: T_base_tcp = T_base_ee * T_ee_tcp
  //    - 位置: p_tcp_in_base = p_ee_in_base + R_ee_in_base * t_ee_in_tcp
  //    - 姿态: R_tcp_in_base = R_ee_in_base * R_ee_to_tcp
  //    若 tcp_to_ee 未配置 (默认单位变换), 退化为 ee_link 位姿 (向后兼容)
  {
    const Eigen::Vector3d& p_ee_in_base = robot_data_->oMf[ee_frame_].translation();
    const Eigen::Matrix3d& R_ee_in_base = robot_data_->oMf[ee_frame_].rotation();
    default_position_ = p_ee_in_base + R_ee_in_base * t_ee_in_tcp_;
    default_quaternion_ = Eigen::Quaterniond(R_ee_in_base * R_ee_to_tcp_);
    default_quaternion_.normalize();
  }

  // 1a) 初始化参考信号滤波器: target = filtered = default (使能时无突变)
  //      上位机下发新参考后, target 立即更新, filtered 平滑追随
  position_d_target_      = default_position_;
  position_d_filtered_    = default_position_;
  orientation_d_target_   = default_quaternion_;
  orientation_d_filtered_ = default_quaternion_;
  // 速度/加速度初始为 0 (使能时机械臂静止, hold 模式参考应为零)
  desired_twist_target_.setZero();
  desired_twist_filtered_.setZero();
  desired_pose_accel_target_.setZero();
  desired_pose_accel_filtered_.setZero();
  has_pose_target_ = false;  // 还未收到上位机参考

  // 1b) 零空间期望构型初始化:
  //     使能时捕获当前关节角作为初始构型, 避免 YAML 配置与实际不一致
  //     导致使能瞬间产生零空间运动. 上位机可通过 q_d_nullspace 话题覆盖.
  q_d_nullspace_target_   = robot_q_;
  q_d_nullspace_filtered_ = robot_q_;
  q_ik_last_              = robot_q_;
  has_nullspace_target_ = false;

  // 2) 捕获使能时刻的残余偏置 (基坐标系下表示)
  //    bias = sensor_wrench_at_enable (变换到 ee 原点, base 系) - tool_gravity_in_base
  //    bias 只反映传感器零漂 + 未建模的固定偏置, 工具重力通过 yaml 实时补偿
  //    坐标变换与 update() 中保持一致: R_ee_sensor 旋转 + 平移修正到 ee 原点
  Vector6d wrench_in_base_at_enable;
  {
    const Eigen::Matrix3d R_sb = ee_rotation_in_base_ * R_sensor_to_ee_;
    const Eigen::Vector3d F_b = R_sb * sensor_wrench.head<3>();
    const Eigen::Vector3d M_b = R_sb * sensor_wrench.tail<3>();
    const Eigen::Vector3d r_sb = ee_rotation_in_base_ * t_sensor_in_ee_;
    wrench_in_base_at_enable.head<3>() = F_b;
    wrench_in_base_at_enable.tail<3>() = M_b + r_sb.cross(F_b);
  }
  sensor_wrench_bias_in_base_ = wrench_in_base_at_enable - computeToolGravityInBase();
  has_sensor_bias_ = true;

  // 复位状态变量, 让阻抗从零偏差开始
  pose_deviation_.setZero();
  twist_deviation_.setZero();
  accel_deviation_.setZero();
  actual_twist_last_.setZero();
  actual_accel_.setZero();
  effort_commands_.setZero();
  q_integral_accum_.setZero();
  force_ff_filtered_.setZero();
  hamiltonian_ = 0;
  hamiltonian_filtered_ = 0;
  hamiltonian_last_ = 0;
  hamiltonian_derivative_ = 0;

  // 清除可能的外部参考, 强制使用 default 位姿
  clearReference();

  // 清除外部 wrench 注入, 恢复为内部 K*dx+D*dv 阻抗模式
  clearExternalWrench();

  RCLCPP_INFO(node_->get_logger(),
              "Impedance enabled: ee_pos=[%.3f, %.3f, %.3f], "
              "residual_bias_base=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              default_position_.x(), default_position_.y(), default_position_.z(),
              sensor_wrench_bias_in_base_(0), sensor_wrench_bias_in_base_(1),
              sensor_wrench_bias_in_base_(2), sensor_wrench_bias_in_base_(3),
              sensor_wrench_bias_in_base_(4), sensor_wrench_bias_in_base_(5));
}

void CartesianImpedanceController::disable() {
  has_sensor_bias_ = false;
  sensor_wrench_bias_in_base_.setZero();
  clearReference();
  clearExternalWrench();
  RCLCPP_INFO(node_->get_logger(), "Impedance disabled: bias cleared");
}

void CartesianImpedanceController::setReference(const ReferenceType& ref) {
  {
    std::lock_guard<std::mutex> lock(rt_reference_mutex_);
    rt_reference_ = ref;
  }
  rt_reference_valid_.store(true, std::memory_order_release);
  blank_reference_ = false;
}

void CartesianImpedanceController::clearReference() {
  {
    std::lock_guard<std::mutex> lock(rt_reference_mutex_);
    rt_reference_ = ReferenceType{};
  }
  rt_reference_valid_.store(false, std::memory_order_release);
  blank_reference_ = true;
}

Eigen::Affine3d CartesianImpedanceController::getTcpPoseInBase() const {
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = R_tcp_in_base_;
  T.translation() = p_tcp_in_base_;
  return T;
}

void CartesianImpedanceController::setExternalWrench(const Vector6d& F_ext) {
  external_wrench_ = F_ext;
  has_external_wrench_ = true;
}

void CartesianImpedanceController::clearExternalWrench() {
  has_external_wrench_ = false;
  external_wrench_.setZero();
}

bool CartesianImpedanceController::computeIk(
    const Eigen::Affine3d& T_desired,
    const VectorXd& q_seed,
    VectorXd& q_out) {
  q_out = q_seed;

  VectorXd q_full_ik = q_full_cache_;
  Matrix6Xd J_full_ik(Matrix6Xd::Zero(kCartesianDim, robot_model_.nv));
  Matrix6Xd J_ik(Matrix6Xd::Zero(kCartesianDim, n_dof_));

  for (int iter = 0; iter < ik_max_iter_; ++iter) {
    for (size_t i = 0; i < n_dof_; ++i) {
      q_full_ik(controlled_q_idx_[i]) = q_out(i);
    }
    pinocchio::forwardKinematics(robot_model_, *robot_data_, q_full_ik);
    pinocchio::updateFramePlacements(robot_model_, *robot_data_);
    pinocchio::computeFrameJacobian(robot_model_, *robot_data_, q_full_ik,
                                    ee_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
                                    J_full_ik);
    for (size_t i = 0; i < n_dof_; ++i) {
      J_ik.col(i) = J_full_ik.col(controlled_v_idx_[i]);
    }
    applyTcpOffsetToJacobian(J_ik);

    Eigen::Affine3d T_actual = Eigen::Affine3d::Identity();
    T_actual.linear() = robot_data_->oMf[ee_frame_].rotation() * R_ee_to_tcp_;
    T_actual.translation() = robot_data_->oMf[ee_frame_].translation() +
        robot_data_->oMf[ee_frame_].rotation() * t_ee_in_tcp_;

    Vector6d twist_err(Vector6d::Zero());
    twist_err.head<3>() = T_desired.translation() - T_actual.translation();
    twist_err.tail<3>() = pinocchio::log3(
        T_actual.linear().transpose() * T_desired.linear());

    if (twist_err.head<3>().norm() < ik_pos_tol_ &&
        twist_err.tail<3>().norm() < ik_rot_tol_) {
      break;
    }

    Eigen::MatrixXd J_ik_pinv;
    dampedPseudoInverse(J_ik, &J_ik_pinv, config_.pinv_damping);
    q_out += J_ik_pinv * twist_err;
  }

  q_ik_last_ = q_out;
  return true;
}

void CartesianImpedanceController::setReferencePose(
    const Eigen::Affine3d& T_desired,
    const Vector6d* twist_desired) {
  ReferenceType ref{};
  ref.pose.position.x = T_desired.translation().x();
  ref.pose.position.y = T_desired.translation().y();
  ref.pose.position.z = T_desired.translation().z();
  Eigen::Quaterniond q(T_desired.linear());
  ref.pose.orientation.w = q.w();
  ref.pose.orientation.x = q.x();
  ref.pose.orientation.y = q.y();
  ref.pose.orientation.z = q.z();
  if (twist_desired) {
    ref.pose_twist.linear.x  = (*twist_desired)(0);
    ref.pose_twist.linear.y  = (*twist_desired)(1);
    ref.pose_twist.linear.z  = (*twist_desired)(2);
    ref.pose_twist.angular.x = (*twist_desired)(3);
    ref.pose_twist.angular.y = (*twist_desired)(4);
    ref.pose_twist.angular.z = (*twist_desired)(5);
  } else {
    ref.pose_twist = geometry_msgs::msg::Twist{};
  }
  ref.pose_accel = geometry_msgs::msg::Accel{};
  VectorXd q_ik_result;
  computeIk(T_desired, q_ik_last_, q_ik_result);
  for (size_t i = 0; i < n_dof_ && i < 7; ++i) {
    ref.q_d_nullspace[i] = q_ik_result(i);
  }
  setReference(ref);
}

double CartesianImpedanceController::computeFilterStep(
    double filter_percent, double frequency) {
  // 移植自参考实现 filterStep()
  //   step = 1 / (kappa * f + 1)
  //   kappa = -1 / log(1 - min(p, 0.999999))
  //   p 越大, step 越大, 跟随越快 (p=1 时 step=1, 不滤波)
  const double p = std::min(std::max(filter_percent, 1e-7), 0.999999);
  const double kappa = -1.0 / std::log(1.0 - p);
  return 1.0 / (kappa * std::max(frequency, 1.0) + 1.0);
}

inline double CartesianImpedanceController::filteredUpdate(
    double target, double current, double step) {
  // 1 阶低通: out = (1-step)*current + step*target
  return (1.0 - step) * current + step * target;
}

Eigen::Isometry3d CartesianImpedanceController::parsePose(
    const std::vector<double>& pose7) {
  // 把 [x, y, z, qx, qy, qz, qw] 解析为 Eigen::Isometry3d
  //   长度 != 7 时返回单位变换 (向后兼容, 等同于 sensor/tcp 与 ee_link 重合)
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (pose7.size() != 7) return T;
  T.translation() = Eigen::Vector3d(pose7[0], pose7[1], pose7[2]);
  Eigen::Quaterniond q(pose7[6], pose7[3], pose7[4], pose7[5]);
  q.normalize();   // 防止用户输入非单位四元数
  T.linear() = q.toRotationMatrix();
  return T;
}

void CartesianImpedanceController::applyTcpOffsetToJacobian(Matrix6Xd& J) const {
  // 把 ee_link Jacobian (LOCAL_WORLD_ALIGNED, 6×n) 变换到 TCP Jacobian
  //   刚体速度变换: v_tcp_linear = v_ee_linear + w_ee × r
  //                 v_tcp_angular = w_ee
  //   其中 r = R_ee_base * t_ee_in_tcp (TCP 原点相对 ee 原点, base 系下表示)
  //   利用 w × r = -skew(r) * w (矩阵形式), 得到:
  //     J_tcp.topRows(3) = J_ee.topRows(3) - skew(r) * J_ee.bottomRows(3)
  //     J_tcp.bottomRows(3) = J_ee.bottomRows(3)
  //   若 t_ee_in_tcp_ 范数 < 1e-9, 视为 TCP 与 ee_link 重合, 直接返回 (向后兼容)
  if (t_ee_in_tcp_.norm() < 1e-9) return;
  const Eigen::Vector3d r = ee_rotation_in_base_ * t_ee_in_tcp_;
  // skew(r) 对应的反对称矩阵
  const Eigen::Matrix3d skew_r = (Eigen::Matrix3d() <<
      0.0,    -r.z(),   r.y(),
      r.z(),   0.0,   -r.x(),
     -r.y(),   r.x(),   0.0).finished();
  // 保存 J_w 的副本 (避免 .noalias() 中 RHS 与 LHS 别名)
  const Eigen::MatrixXd J_w = J.bottomRows<3>();
  J.topRows<3>().noalias() -= skew_r * J_w;
  // J.bottomRows(3) 不变
}

void CartesianImpedanceController::updateFilteredReference() {
  // 末端位姿滤波 (位置: 1 阶低通; 姿态: 球面线性插值 slerp)
  // 移植自参考实现 updateFilteredPose()
  if (filter_params_pose_ >= 0.999999) {
    position_d_filtered_ = position_d_target_;
    orientation_d_filtered_ = orientation_d_target_;
    desired_twist_filtered_ = desired_twist_target_;
    desired_pose_accel_filtered_ = desired_pose_accel_target_;
  } else {
    const double step = computeFilterStep(filter_params_pose_, update_frequency_);

    position_d_filtered_.x() =
        filteredUpdate(position_d_target_.x(), position_d_filtered_.x(), filter_params_pose_/*step*/);
    position_d_filtered_.y() =
        filteredUpdate(position_d_target_.y(), position_d_filtered_.y(), filter_params_pose_/*step*/);
    position_d_filtered_.z() =
        filteredUpdate(position_d_target_.z(), position_d_filtered_.z(), filter_params_pose_/*step*/);
    // slerp: q_filtered = q_filtered.slerp(step, q_target)
    orientation_d_filtered_ =
        orientation_d_filtered_.slerp(step, orientation_d_target_);
    // 末端速度/加速度参考: 与位置采用同一滤波系数 (1 阶低通, 逐分量)
    //   避免 twist/accel 跳变引起阻抗力矩突变, 与位姿滤波保持时间一致
    for (int i = 0; i < kCartesianDim; ++i) {
      desired_twist_filtered_(i) = filteredUpdate(
          desired_twist_target_(i), desired_twist_filtered_(i), filter_params_pose_/*step*/);
      desired_pose_accel_filtered_(i) = filteredUpdate(
          desired_pose_accel_target_(i), desired_pose_accel_filtered_(i), filter_params_pose_/*step*/);
    }
  }

  // 零空间期望构型滤波 (1 阶低通, 逐关节)
  if (filter_params_nullspace_ >= 0.999999) {
    q_d_nullspace_filtered_ = q_d_nullspace_target_;
  } else {
    const double step =
        computeFilterStep(filter_params_nullspace_, update_frequency_);
    for (size_t i = 0; i < n_dof_; ++i) {
      q_d_nullspace_filtered_(i) = filteredUpdate(
          q_d_nullspace_target_(i), q_d_nullspace_filtered_(i), filter_params_pose_/*step*/);
    }
  }
}

}  // namespace snowbot_impedance

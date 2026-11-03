// Simulation-only stdio bridge: actual controller, inert ROS, no device API.
#include "impedance/cartesian_impedance_controller.h"
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace snowbot_impedance;

int main(int argc, char** argv) try {
  if (argc != 12) throw std::runtime_error("URDF YAML SCALE LEFT_FRAME RIGHT_FRAME LEFT_XYZ RIGHT_XYZ required");
  const double scale = std::stod(argv[3]);
  if (!std::isfinite(scale) || scale <= 0.) throw std::runtime_error("invalid stiffness scale");
  std::array<CartesianImpedanceController, 2> controllers;
  for (int side = 0; side < 2; ++side) {
    auto cfg = ImpedanceConfig::fromYaml(argv[2], side ? "right" : "left", argv[1]);
    cfg.session_reference_only = true;
    cfg.filter_params_pose = cfg.filter_params_nullspace = 1.;
    cfg.integral_gain_joint.assign(7, 0.); cfg.force_feedback_gain.assign(6, 0.);
    // Virtual sensors and grasp TCPs are expressed in the selected model frames.
    cfg.ee_link = argv[4 + side];
    cfg.sensor_to_ee = {0., 0., 0., 0., 0., 0., 1.};
    cfg.tcp_to_ee = {std::stod(argv[6+3*side]), std::stod(argv[7+3*side]),
                     std::stod(argv[8+3*side]), 0., 0., 0., 1.};
    // PhysX uses the same URDF mass and Nm units; no hardware calibration/friction.
    cfg.kt_gravity.assign(7, 1.); cfg.lugre_friction.enable.assign(7, 0.);
    cfg.tool_mass = 0.;
    for (auto& k : cfg.stiffness) k *= scale;
    for (auto& d : cfg.damping) d *= std::sqrt(scale);
    if (!controllers[side].init(std::make_shared<rclcpp::Node>(), cfg))
      throw std::runtime_error("controller initialization failed");
  }
  std::array<std::string, 20> names;
  int n = 0;
  for (const std::string side : {"l", "r"})
    for (int j = 1; j <= 8; ++j) names[n++] = "arm_"+side+"_joint_"+std::to_string(j);
  for (const std::string group : {"head", "body"})
    for (int j = 1; j <= 2; ++j) names[n++] = group+"_joint_"+std::to_string(j);
  auto& model = controllers[0];
  VectorXd nominal = VectorXd::Zero(model.getNqFull()), q = nominal, dq = nominal, zero = nominal;
  if (model.getNqFull() != 20) throw std::runtime_error("model layout");
  auto read = [](double& value) {
    if (!(std::cin >> value) || !std::isfinite(value)) throw std::runtime_error("invalid measured state");
  };
  for (const auto& name : names) read(nominal[model.getJointQIdx(name)]);
  std::cout << "READY" << std::endl;
  bool enabled = false;
  double dt;
  while (std::cin >> dt) {
    if (!std::isfinite(dt) || dt <= 0. || dt > .01) throw std::runtime_error("invalid physics dt");
    for (const auto& name : names) read(q[model.getJointQIdx(name)]);
    for (const auto& name : names) read(dq[model.getJointVIdx(name)]);
    std::array<Vector6d, 2> wrench;
    for (auto& w : wrench) for (int j = 0; j < 6; ++j) read(w[j]);
    for (int side = 0; side < 2; ++side) {
      auto& ctl = controllers[side];
      if (!enabled) ctl.enable(q, dq, wrench[side]);
      if (!ctl.setNominalReference(nominal, zero)) throw std::runtime_error("invalid nominal reference");
      ctl.update(q, dq, zero, wrench[side], dt);
      if (!ctl.getEffortCommands().allFinite()) throw std::runtime_error("nonfinite effort");
    }
    enabled = true;
    std::cout << std::setprecision(17) << "TORQUE";
    for (auto& ctl : controllers) for (int j = 0; j < 7; ++j) std::cout << ' ' << ctl.getEffortCommands()[j];
    for (auto& ctl : controllers) for (int j = 0; j < 6; ++j) std::cout << ' ' << ctl.getPoseDeviation()[j];
    for (auto& ctl : controllers) for (int j = 0; j < 3; ++j) std::cout << ' ' << ctl.getTcpPoseInBase().translation()[j];
    std::cout << std::endl;
  }
  return 0;
} catch (const std::exception& error) {
  std::cerr << "IMPEDANCE_PHYSICS: " << error.what() << '\n'; return 1;
}

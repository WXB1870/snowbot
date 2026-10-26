# 阻抗控制综合参考消息.
#
# 同时承载末端位姿参考 (与 kinematic_pose_msgs/KinematicPose 一致)
# 和零空间期望构型 (7 维关节角), 保证上位机下发时两者时间一致.
#
# 末端位姿参考 (与 KinematicPose 字段同名同序, 便于复用):
#   pose       末端位姿 (位置 + 姿态)
#   pose_twist 末端速度参考 (线速度 + 角速度)
#   pose_accel 末端加速度参考 (线加速度 + 角加速度)
#
# 零空间期望构型:
#   q_d_nullspace  7 维关节角 [rad] (与 controlled_joint_names 一一对应)

geometry_msgs/Pose pose
geometry_msgs/Twist pose_twist
geometry_msgs/Accel pose_accel

float64[7] q_d_nullspace

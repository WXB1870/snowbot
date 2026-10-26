#中层节点向着上层节点发送信息的代码

#时间戳头
std_msgs/Header header

#----------控制反馈量----------

int32 if_first_temp_in_base     #底层控制器中是否为第一轮插补
int32 if_first_temp_in_middle   #中层控制器中是否为第一轮插补

#----------反馈值----------

float64[7] l_arm_pos_fdback     #左臂关节位置反馈
float64[7] l_arm_spd_fdback     #左臂关节速度反馈
float64[7] l_arm_iq_fdback      #左臂电流iq反馈

float64[7] r_arm_pos_fdback     #右臂的位置反馈
float64[7] r_arm_spd_fdback     #右臂的速度反馈
float64[7] r_arm_iq_fdback      #右臂的电流反馈

#----------计算状态反馈----------

#状态反馈值 -- 是否到达目标位置的判别
bool[7] l_arm_if_jnt_reach_despos  #左臂关节是否到达目标位置
bool[7] r_arm_if_jnt_reach_despos  #右臂关节是否到达目标位置

#状态反馈值 -- 本轮控制的目标位置、速度反馈
float64[7] l_arm_jntpos_des
float64[7] l_arm_jntspd_des

float64[7] r_arm_jntpos_des
float64[7] r_arm_jntspd_des

#状态反馈值 -- 本轮控制的重力、惯性力、前馈力补偿反馈
float64[7] l_arm_jnttor_gravcomp
float64[7] l_arm_jnttor_interiacomp
float64[7] l_arm_jnttor_feedforward

float64[7] r_arm_jnttor_gravcomp
float64[7] r_arm_jnttor_interiacomp
float64[7] r_arm_jnttor_feedforward
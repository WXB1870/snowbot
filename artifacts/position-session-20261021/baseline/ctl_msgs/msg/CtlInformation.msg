# CtlInformation

#时间戳头
std_msgs/Header header

#----------控制代码----------

#控制周期
float64 temp_time

#插补模式代码
int32 runin_mod

#Topic 点位模式标志：用于底层区分 Topic 点位初始化与 Demo/VR
bool if_topic_point_position

#判断是否刷新底层插补器
int32 if_runin_renew

#判断是否使用返回值作为第一轮插补输入值
int32 if_useret_to_start_runin

#判断是否使用调试模式
int32 if_use_debugmod

#下发的初始值（位置量，其他的均为0，Kp、Kd不参与插补）
float64[7] l_position_at_runin_st
float64[7] r_position_at_runin_st

#位置保持模式控制代码：0--终止位置保持模式；1--开启位置保持模式
int32 place_hold

#电机使能：0--抱闸(关闭使能), 1--使能(正常运控)
int32 motor_enable
#单关节使能 (0=抱闸, 1=使能), 仅motor_enable=1时有效
int32[7] l_motor_enable
int32[7] r_motor_enable

#底层控制模式选择代码：
int32[7] base_rigidity_ctl_mode_l       #左臂各个关节底层控制模式选择
int32[7] base_rigidity_ctl_mode_r       #右臂各个关节底层控制模式选择

#----------数据传输代码----------

float64[7] l_position    # 目标位置 (rad)
float64[7] l_kp          # 比例增益
float64[7] l_kd          # 微分增益
float64[7] l_speed       # 目标速度 (rad/s)
float64[7] l_torque_grav_comp       # 目标重力补偿力矩 (Nm)
float64[7] l_torque_interia_comp    # 目标惯性补偿力矩 (Nm)
float64[7] l_torque_feedforward     # 目标前馈力矩 (Nm)

float64[7] r_position    # 目标位置 (rad)
float64[7] r_kp          # 比例增益
float64[7] r_kd          # 微分增益
float64[7] r_speed       # 目标速度 (rad/s)
float64[7] r_torque_grav_comp        # 目标重力补偿力矩 (Nm)
float64[7] r_torque_interia_comp    # 目标惯性补偿力矩 (Nm)
float64[7] r_torque_feedforward     # 目标前馈力矩 (Nm)

#----------reserve----------

#保留量
float64[7] float_reserve
int32[7] int_reserve
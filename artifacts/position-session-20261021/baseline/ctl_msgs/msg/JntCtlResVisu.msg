#关节测试结果综合可视化函数

#原始目标位置
float64[7] l_des_pos_ori
float64[7] r_des_pos_ori

#目标位置
float64[7] l_des_pos
float64[7] r_des_pos

#原始目标速度
float64[7] l_des_vel_ori
float64[7] r_des_vel_ori

#目标速度
float64[7] l_des_vel
float64[7] r_des_vel

#目标加速度
float64[7] l_des_acc
float64[7] r_des_acc

#目标PD参数
float64[7] l_des_kp
float64[7] r_des_kp

float64[7] l_des_kd
float64[7] r_des_kd

#实际位置
float64[7] l_pos_ret
float64[7] r_pos_ret

#实际速度
float64[7] l_vel_ret
float64[7] r_vel_ret

#前馈力
float64[7] l_tor_feedforward
float64[7] r_tor_feedforward

#摩擦补偿
float64[7] l_fric_comp
float64[7] r_fric_comp

#实际电流
float64[7] l_current_ret
float64[7] r_current_ret

#位置偏差
float64[7] l_pos_err
float64[7] r_pos_err

#速度偏差
float64[7] l_vel_err
float64[7] r_vel_err

#时间戳
float64 time_spot

#时间间隔
float64 derta_time
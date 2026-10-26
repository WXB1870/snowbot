#RetInformation

#时间戳头
std_msgs/Header header

float64[8] l_position    # 反馈位置 (rad), J1~J7 + 左夹爪
float64[8] l_speed       # 反馈速度 (rad/s), J1~J7 + 左夹爪
float64[8] l_iq      # 反馈电流, J1~J7 + 左夹爪

float64[8] r_position    # 反馈位置 (rad), J1~J7 + 右夹爪
float64[8] r_speed       # 反馈速度 (rad/s), J1~J7 + 右夹爪
float64[8] r_iq      # 反馈电流, J1~J7 + 右夹爪

int32 if_first_time   #标志量：是否是第一轮 1--是 0--否

float64[7] float_reserve
int32[7] int_reserve

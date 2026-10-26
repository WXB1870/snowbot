#补偿量可视化

#重力综合补偿可视化
float64[7] l_grav_comp
float64[7] r_grav_comp

float64[7] l_interia_comp
float64[7] r_interia_comp

float64[7] l_feedforward_comp
float64[7] r_feedforward_comp

#刚度补偿可视化
float64[7] l_k_comp
float64[7] r_k_comp

#摩擦补偿可视化
float64[7] l_fric_comp
float64[7] r_fric_comp

#有待于进一步添加的控制量
#模糊控制输出
float64[7] l_fuzzy_kd
float64[7] r_fuzzy_kd
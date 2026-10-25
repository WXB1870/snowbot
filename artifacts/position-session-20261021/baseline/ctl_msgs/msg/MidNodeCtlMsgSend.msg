#上层节点与中层节点通讯代码
#上层节点向中层节点下发消息的代码
#该信息字段较长，有可能会出现通讯延时过久的bug

#----------控制消息发送代码----------

#时间戳头
std_msgs/Header header

#-----数据传输与插补模式选择-----

int32 ctl_trans_mod         #数据传输模式
int32 runin_mod             #插补模式

#-----求解模式信息-----

int32 if_use_ret            #是否使用返回值
bool if_use_temp            #是否使用循环输出
bool if_use_ret_to_start_runin  #下层插补是否从当前反馈位置开始
bool if_streaming_joint_position #是否为连续关节位置流
int32[7] rigidity_ctl_mod_l     #左臂各个关节底层控制模式选择
int32[7] rigidity_ctl_mod_r     #右臂各个关节底层控制模式选择

#搭配自定义刚度模式的控制代码
float64[7] kp_trans_l       #左臂关节Kp
float64[7] kd_trans_l       #左臂关节Kd
float64[7] kp_trans_r       #右臂关节Kp
float64[7] kd_trans_r       #右臂关节Kd

#-----重力补偿控制-----

int32 if_use_grav_comp      #是否启用重力补偿
int32 if_use_fric_comp      #是否启用自带摩擦补偿
int32 if_use_inp_ftr        #是否使用输入滤波器
int32 if_use_outp_ftr       #是否使用输出滤波器
int32 grav_comp_inp_mod     #重力补偿输入模式

int32 st_jnt_id_l_arm
int32 st_jnt_id_r_arm
int32 urdf_choose

#-----插补信息包装-----

float64 upper_monitor_time   #上位机迭代时间

float64[7] traj_time_l_arm  #左臂轨迹时间
float64[7] traj_time_r_arm  #右臂轨迹时间

float64[7] acc_rate_base_l_arm  #左臂加速阶段占比
float64[7] acc_rate_base_r_arm  #右臂加速阶段占比

bool[7] l_arm_if_use_stab  #左臂插补过程中是否使用静止阶段
bool[7] r_arm_if_use_stab  #右臂插补过程中是否使用静止阶段

int32[7] l_arm_start_stab_times     #左臂起始阶段静止轮数
int32[7] r_arm_start_stab_times     #右臂起始阶段静止轮数

int32[7] l_arm_des_stab_times       #左臂静止阶段静止轮数
int32[7] r_arm_des_stab_times       #右臂静止阶段静止轮数

#-----关节控制恒定位置输出开关-----

int32[7] l_arm_if_jnt_stab      #左臂关节是否开启恒定位置模式
int32[7] r_arm_if_jnt_stab      #右臂关节是否开启恒定位置模式

bool if_redefine_stab_jnt_ang   #判断量 -- 是否重新定义静止关节的角度

#-----关节限制信息-----

bool l_arm_if_use_jntlim       #左臂是否限制关节信息
int32 l_arm_jntlim_num          #左臂受限关节数量
string[7] l_arm_limjnt_label    #左臂受限关节标签
float64[7] l_arm_limjnt_ang        #左臂受限关节角度

bool r_arm_if_use_jntlim       #右臂是否限制关节信息
int32 r_arm_jntlim_num          #右臂受限关节数量
string[7] r_arm_limjnt_label    #右臂受限关节标签
float64[7] r_arm_limjnt_ang        #右臂受限关节角度

bool if_update_lim             #是否更新限制信息

#-----关节初始化信息-----

#是否初始化或者重新初始化判断量
bool if_reinit

#初始化左右臂关节位置
float64[7] l_arm_st_pos
float64[7] r_arm_st_pos

#信息打印开关量
bool if_disp_jnt_info
bool if_use_visualization
bool if_disp_msgs

#优化器参数
float64[5] opti_weight      #优化器权重
int32 solver_template_times   #优化器迭代次数
float64 err_tol             #优化器容许误差

int32 if_print_level        #是否打印优化日志
bool if_print_optidata      #是否打印计算过程数据


#-----目标控制数据-----

#左右臂笛卡尔空间

float64[7] l_arm_des_pos_descartes
float64[7] r_arm_des_pos_descartes

float64[7] l_arm_des_tor_descartes
float64[7] r_arm_des_tor_descartes

#左右臂关节空间

float64[7] l_arm_des_pos_jntspace
float64[7] r_arm_des_pos_jntspace

float64[7] l_arm_des_tor_jntspace
float64[7] r_arm_des_tor_jntspace
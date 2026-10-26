#include "dual_arm_control/msgPublish.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

// 全局变量（定义，不在此初始化 —— 初始化在 rosloop_var_init() 中完成）

//---------------中层节点与下位机通讯全局变量---------------

//控制量发送：
int runin_mod;
int if_runin_renew;
int ifUseRet_toStRunin;
int ifUseDebugMod;
int base_rigidityCtl_mod_l[SINGLEARMDOF];
int base_rigidityCtl_mod_r[SINGLEARMDOF];

//左右臂初始值刷新
std::vector<double> lArm_pos_atRuninSt;
std::vector<double> rArm_pos_atRuninSt;

//目标参量发送：
std::vector<double> lArm_joint_pos;
std::vector<double> lArm_joint_kp;
std::vector<double> lArm_joint_kd;
std::vector<double> lArm_joint_spd;

std::vector<double> lArm_joint_tor_grav;
std::vector<double> lArm_joint_tor_interia;
std::vector<double> lArm_joint_tor_feedforward;

std::vector<double> rArm_joint_pos;
std::vector<double> rArm_joint_kp;
std::vector<double> rArm_joint_kd;
std::vector<double> rArm_joint_spd;

std::vector<double> rArm_joint_tor_grav;
std::vector<double> rArm_joint_tor_interia;
std::vector<double> rArm_joint_tor_feedforward;

std::vector<double> lArm_torFeedForward_assist;
std::vector<double> rArm_torFeedForward_assist;

//控制状态向上位机反馈：
std::vector<bool> lArm_ifReachEnd;
std::vector<bool> rArm_ifReachEnd;

//---------------中层节点与上位机通讯全局变量---------------

//-----控制信息-----

//综合控制信息
modCtl modCtlMsg_get;

//程序打印控制参数
optiPrintMsgCtl printMsgPkg_get;

//优化器参数打包
optimizerCtl optimizerPara_get;

//关节限制信息打包
JntLimMsg lArm_LimMsg_get;
JntLimMsg rArm_LimMsg_get;
JntLimMsg allArm_LimMsg_get;

//控制过程信息打包
bool ifReInit_get;
bool ifUpdateLim_get;
bool ifRedefineJntTrajStab_get;

//左右臂关节初始位置
std::vector<double> lArm_stPos_get;
std::vector<double> rArm_stPos_get;
bool ifStartCal;

//-----目标位置等信息-----
std::vector<double> lArm_desPos_descartes;
std::vector<double> lArm_desPos_JntSpace;

std::vector<double> lArm_desTor_descartes;
std::vector<double> lArm_desTor_JntSpace;

std::vector<double> rArm_desPos_descartes;
std::vector<double> rArm_desPos_JntSpace;

std::vector<double> rArm_desTor_descartes;
std::vector<double> rArm_desTor_JntSpace;

//搭配自定义PD参数模式使用 -- PD参数接收模块
std::vector<double> lArm_Kp_;
std::vector<double> lArm_Kd_;
std::vector<double> rArm_Kp_;
std::vector<double> rArm_Kd_;

//-----------------------------------------------------

//反馈接收：
std::vector<double> lArm_pos_fdback;
std::vector<double> lArm_spd_fdback;
std::vector<double> lArm_iq_fdback;

std::vector<double> rArm_pos_fdback;
std::vector<double> rArm_spd_fdback;
std::vector<double> rArm_iq_fdback;

int if_lower_firstTime;
int if_middle_firstTime;

//debug用的辅助变量
std::vector<double> vec_dbg_1;
std::vector<double> vec_dbg_2;

//===================================================================================
void signalHandler(int sig) {
    shutdown_flag = 1;
}
bool is_key_pressed(int timeout_ms) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval timeout = {0, timeout_ms * 1000};
    return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout) > 0;
}

//============================================================
//调试用辅助函数
std::vector<double> JntPDoutpDebug(double posDes, double velDes, double posRet, double Kp, double Kd)
{
    std::vector<double> mRes;
    double mRes_all = (posDes - posRet) * Kp + velDes * Kd;
    double mRes_kp = (posDes - posRet) * Kp;
    double mRes_kd = velDes * Kd;
    mRes.push_back(mRes_all);
    mRes.push_back(mRes_kp);
    mRes.push_back(mRes_kd);

    return mRes;
}

//===================================================================================

//构造函数
ArmCtlNode::ArmCtlNode() : Node("dual_arm_control_node") , 
    has_received_command_(false),
    has_received_upperMonitor_command_(false)
{
    //加载urdf文件
    try {
        std::string urdf_dir = get_packages_path("dual_arm", "urdf");
        std::filesystem::path p(urdf_dir);
        path_options[0] = (p / "left_arm.urdf").string();
        path_options[1] = (p / "right_arm.urdf").string();
        path_options[2] = (p / "snowbot.urdf").string();
        RCLCPP_INFO(this->get_logger(), "URDF directory found: %s", urdf_dir.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error finding URDF directory: %s", e.what());
        return;
    }

    std::string urdf_path = path_options[URDFCHOOSE];
    std::ifstream urdf_file(urdf_path);


    //读取urdf文件内容
    if (!urdf_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", urdf_path.c_str());
        //rclcpp::shutdown();
        return;
    }

    robot_description_ = std::string((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
    if (robot_description_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "URDF file is empty!");
        //rclcpp::shutdown();
        return;
    }

    //加载机器人模型
    robot_model_ = std::make_shared<urdf::Model>();
    if (!robot_model_->initString(robot_description_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF model");
        //rclcpp::shutdown();
        return;
    }

    //初始化KDL树
    if (!kdl_parser::treeFromString(robot_description_, kdl_tree_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
        return;
    }

    //提取KDL链 (到link_7，不含夹爪joint_8)
    if (!kdl_tree_.getChain(base_link[0], "arm_l_link_7", kdl_lArm_chain_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to extract Left Arm KDL chain");
        return;
    }
    if(!kdl_tree_.getChain(base_link[1], "arm_r_link_7", kdl_rArm_chain_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to extract Right Arm KDL chain");
        return;
    }

    // 订阅 /joint_states 缓存 body/head/gripper 角度 (由各自控制节点发布)
    js_cache_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&ArmCtlNode::js_cache_callback, this, std::placeholders::_1));

    // 初始化 RobotStatePublisher
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"robot_description", robot_description_},
        {"publish_frequency", 1000.0}  // 与循环频率匹配
    });
    robot_state_publisher_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(options);


    // 初始化TF广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 加载发布者与服务：joint state publisher
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    // 服务：允许外部修改关节角度
    set_joint_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_joint_angles",
        std::bind(&ArmCtlNode::set_joint_angles, this, std::placeholders::_1, std::placeholders::_2));

    //初始化发布与接受的消息类型

    mCtlData_pub = this->create_publisher<snowbot_ctl_msgs::msg::CtlInformation>(
        "ctl_data_pub",
        10
    );
    mRetData = this->create_subscription<snowbot_ctl_msgs::msg::RetInformation>(
        "dualArmCtl_fdback",			// 订阅的话题（必须与发布者一致)
        10,								// QoS历史深度
        std::bind(&ArmCtlNode::message_callback, this, std::placeholders::_1)
    );

    upperMonitorMsg_get = this->create_subscription<snowbot_ctl_msgs::msg::MidNodeCtlMsgSend>(
        "upperMonitor_msg_send",
        10,
        std::bind(&ArmCtlNode::upperMonitor_msg_callback, this, std::placeholders::_1)
    );
    upperMonitorMsg_send = this->create_publisher<snowbot_ctl_msgs::msg::MidNodeCtlMsgRet>(
        "midMonitor_msg_send",
        10
    );

    mDbgData_pub = this->create_publisher<snowbot_ctl_msgs::msg::DbgInformation>(
        "dbg_data_pub",
        10
    );

    //初始化画图代码
    endPosTrajPlot = new EndPos_plot(this);

    //初始化机器人配置代码
    yaml_path_zeroPos = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") + "/config/zeropos.yaml";
    yaml_path_desc_zeroPos = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") + "/config/desc_zeropos.yaml";
    yaml_path_initialPD = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") + "/config/initialPD.yaml";
    std::cout << " ===== 初始化关节层零位配置 ===== " << std::endl;
    loadZeroPosFromYaml(yaml_path_zeroPos);
    std::cout << " ===== 初始化笛卡尔空间零位配置 ===== " << std::endl;
    loadDescZeroPosFromYaml(yaml_path_desc_zeroPos);
    std::cout << " ===== 初始化PD参数配置 ===== " << std::endl;
    loadInitialPDFromYaml(yaml_path_initialPD);

}


//------------------------------
//控制参量发送模块
void ArmCtlNode::msg_publish()
{
    auto tempCtlData_pub = snowbot_ctl_msgs::msg::CtlInformation();
    tempCtlData_pub.header.stamp = this->now();  // 添加当前时间戳


    tempCtlData_pub.temp_time = TEMPTIME;   //发送迭代时间

    //发送控制量
    // Topic 点位轨迹交给伺服层做线性 1kHz 细分；Demo/VR 保留原有下发模式。
    tempCtlData_pub.runin_mod = runin_mod;
    tempCtlData_pub.if_topic_point_position = modCtlMsg_get.ifStreamingJointPosition;
    tempCtlData_pub.if_runin_renew = if_runin_renew;
    tempCtlData_pub.if_useret_to_start_runin = ifUseRet_toStRunin;
    tempCtlData_pub.if_use_debugmod = ifUseDebugMod;
    tempCtlData_pub.motor_enable = motor_enable_;
    // 单关节使能: ifJntStab[i]=0(动)→1(使能), =1(锁位)→0(抱闸)
    for(int i=0;i<SINGLEARMDOF;i++){
        int l_stab = modCtlMsg_get.ifJntStab.size()>0 ? modCtlMsg_get.ifJntStab[0][i] : 0;
        int r_stab = modCtlMsg_get.ifJntStab.size()>0 ? modCtlMsg_get.ifJntStab[1][i] : 0;
        tempCtlData_pub.l_motor_enable[i] = motor_enable_ ? (l_stab != 1) : 0;
        tempCtlData_pub.r_motor_enable[i] = motor_enable_ ? (r_stab != 1) : 0;
    }

    for(int i = 0; i < SINGLEARMDOF; i++){
        tempCtlData_pub.base_rigidity_ctl_mode_l[i] = base_rigidityCtl_mod_l[i];
        tempCtlData_pub.base_rigidity_ctl_mode_r[i] = base_rigidityCtl_mod_r[i];
    }

    //发送左右臂初始值刷新
    for(int i = 0; i < SINGLEARMDOF; i++){
        tempCtlData_pub.l_position_at_runin_st[i] = lArm_pos_atRuninSt[i];
        tempCtlData_pub.r_position_at_runin_st[i] = rArm_pos_atRuninSt[i];
    }

    for(int i = 0; i < SINGLEARMDOF; i++){
        /**/
        tempCtlData_pub.l_position[i] = lArm_joint_pos[i];
        tempCtlData_pub.l_speed[i] = lArm_joint_spd[i];

        tempCtlData_pub.l_torque_grav_comp[i] = lArm_joint_tor_grav[i];
        tempCtlData_pub.l_torque_interia_comp[i] = lArm_joint_tor_interia[i];
        tempCtlData_pub.l_torque_feedforward[i] = lArm_joint_tor_feedforward[i];

        tempCtlData_pub.l_kp[i] = lArm_joint_kp[i];
        tempCtlData_pub.l_kd[i] = lArm_joint_kd[i];

        tempCtlData_pub.r_position[i] = rArm_joint_pos[i];
        tempCtlData_pub.r_speed[i] = rArm_joint_spd[i];

        tempCtlData_pub.r_torque_grav_comp[i] = rArm_joint_tor_grav[i];
        tempCtlData_pub.r_torque_interia_comp[i] = rArm_joint_tor_interia[i];
        tempCtlData_pub.r_torque_feedforward[i] = rArm_joint_tor_feedforward[i];

        tempCtlData_pub.r_kp[i] = rArm_joint_kp[i];
        tempCtlData_pub.r_kd[i] = rArm_joint_kd[i];

        tempCtlData_pub.float_reserve[i] = 0;
        tempCtlData_pub.int_reserve[i] = 0;
    }
    mCtlData_pub->publish(tempCtlData_pub);
}

//上位机指令发送模块
void ArmCtlNode::midMonitor_msg_publish(){

    auto tempCtlData_pub = snowbot_ctl_msgs::msg::MidNodeCtlMsgRet();
    tempCtlData_pub.header.stamp = this->now();     //添加当前时间戳

    //发送控制反馈量

    tempCtlData_pub.if_first_temp_in_base = if_lower_firstTime;
    tempCtlData_pub.if_first_temp_in_middle = if_middle_firstTime;

    //发送反馈值
    for(int i = 0; i < SINGLEARMDOF; i++){
        tempCtlData_pub.l_arm_pos_fdback[i] = lArm_pos_fdback[i];
        tempCtlData_pub.l_arm_spd_fdback[i] = lArm_spd_fdback[i];
        tempCtlData_pub.l_arm_iq_fdback[i] = lArm_iq_fdback[i];
    }
    for(int i = 0; i < SINGLEARMDOF; i++){
        tempCtlData_pub.r_arm_pos_fdback[i] = rArm_pos_fdback[i];
        tempCtlData_pub.r_arm_spd_fdback[i] = rArm_spd_fdback[i];
        tempCtlData_pub.r_arm_iq_fdback[i] = rArm_iq_fdback[i];
    }

    //发送计算状态反馈

    for(int i = 0; i < SINGLEARMDOF; i++){
        tempCtlData_pub.l_arm_if_jnt_reach_despos[i] = lArm_ifReachEnd[i];
        tempCtlData_pub.r_arm_if_jnt_reach_despos[i] = rArm_ifReachEnd[i];
    }

    for(int i = 0; i < SINGLEARMDOF; i++){
        tempCtlData_pub.l_arm_jntpos_des[i] = lArm_joint_pos[i];
        tempCtlData_pub.l_arm_jntspd_des[i] = lArm_joint_spd[i];
        
        tempCtlData_pub.l_arm_jnttor_gravcomp[i] = lArm_joint_tor_grav[i];
        tempCtlData_pub.l_arm_jnttor_interiacomp[i] = lArm_joint_tor_interia[i];
        tempCtlData_pub.l_arm_jnttor_feedforward[i] = lArm_joint_tor_feedforward[i];
    }
    for(int i = 0; i < SINGLEARMDOF; i++){
        tempCtlData_pub.r_arm_jntpos_des[i] = rArm_joint_pos[i];
        tempCtlData_pub.r_arm_jntspd_des[i] = rArm_joint_spd[i];

        tempCtlData_pub.r_arm_jnttor_gravcomp[i] = rArm_joint_tor_grav[i];
        tempCtlData_pub.r_arm_jnttor_interiacomp[i] = rArm_joint_tor_interia[i];
        tempCtlData_pub.r_arm_jnttor_feedforward[i] = rArm_joint_tor_feedforward[i];
    }

    upperMonitorMsg_send->publish(tempCtlData_pub);

}

//反馈值接受模块
void ArmCtlNode::fdback_get() {
    rclcpp::spin_some(this->get_node_base_interface());  // 非阻塞检查回调
}

//------------------------------
//仿真信息与debug信息发送模块

// 发布身体/头部锁关节TF (关节角=0, 保证完整TF链)
void ArmCtlNode::publish_body_head_tf() {
    auto half_sin = [](double a) { return std::sin(a * 0.5); };
    auto half_cos = [](double a) { return std::cos(a * 0.5); };

    // 读取 body_head_control / force_sensor 发布的关节角度
    double bj1=0, bj2=0, hj1=0, hj2=0, gl=0, gr=0;
    {
        std::lock_guard<std::mutex> lock(js_cache_mutex_);
        bj1 = cached_body_j1_; bj2 = cached_body_j2_;
        hj1 = cached_head_j1_; hj2 = cached_head_j2_;
        gl  = cached_grip_l_;  gr  = cached_grip_r_;
    }

    auto send_tf = [this](const std::string& parent, const std::string& child,
                          double x, double y, double z,
                          double qx, double qy, double qz, double qw) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
        t.transform.rotation.x = qx;
        t.transform.rotation.y = qy;
        t.transform.rotation.z = qz;
        t.transform.rotation.w = qw;
        tf_broadcaster_->sendTransform(t);
    };

    // body_joint_1: Z轴
    send_tf("body_base_link", "body_link_1", 0, 0, 0.1645, 0, 0, half_sin(bj1), half_cos(bj1));
    // body_joint_2: Y轴
    send_tf("body_link_1", "body_link_2", 0.09, 0, 0.2, 0, half_sin(bj2), 0, half_cos(bj2));
    // head_joint_1: -Y轴
    send_tf("body_link_2", "head_link_1", 0, 0, 0.4, 0, half_sin(-hj1), 0, half_cos(-hj1));
    // head_joint_2: Z轴
    send_tf("head_link_1", "head_link_2", 0, 0, 0, 0, 0, half_sin(hj2), half_cos(hj2));
    // gripper: Z轴
    send_tf("arm_l_link_7", "arm_l_link_8", 0.122, 0, 0, 0, 0, half_sin(gl), half_cos(gl));
    send_tf("arm_r_link_7", "arm_r_link_8", 0.122, 0, 0, 0, 0, half_sin(gr), half_cos(gr));
}

// 发送关节角度到RViz
// 使用KDL计算正向运动学并发布TF
void ArmCtlNode::js_cache_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(js_cache_mutex_);
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++) {
        if (msg->name[i] == "body_joint_1")   cached_body_j1_ = msg->position[i];
        if (msg->name[i] == "body_joint_2")   cached_body_j2_ = msg->position[i];
        if (msg->name[i] == "head_joint_1")   cached_head_j1_ = msg->position[i];
        if (msg->name[i] == "head_joint_2")   cached_head_j2_ = msg->position[i];
        if (msg->name[i] == "arm_l_joint_8")  cached_grip_l_  = msg->position[i];
        if (msg->name[i] == "arm_r_joint_8")  cached_grip_r_  = msg->position[i];
    }
}

void ArmCtlNode::publish_tf_from_joint_states(KDL::Chain mkdl_chain_, std::vector<double> mjoint_pos) {
    // 创建KDL关节数组
    KDL::JntArray jnt_array(mkdl_chain_.getNrOfJoints());
    
    // 填充当前关节位置
    for (size_t i = 0; i < mjoint_pos.size() && i < static_cast<size_t>(mkdl_chain_.getNrOfJoints()); ++i) {
        jnt_array(i) = mjoint_pos[i];
    }

    // 创建KDL帧数组来存储各连杆的位姿
    std::vector<KDL::Frame> frames(mkdl_chain_.getNrOfSegments() + 1);
    frames[0] = KDL::Frame::Identity(); // 基座坐标系

    // 计算各连杆的位姿
    int joint_id = 0;
    KDL::ChainFkSolverPos_recursive fk_solver(mkdl_chain_);
    for (size_t i = 0; i < mkdl_chain_.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = mkdl_chain_.getSegment(i);
        const KDL::Joint& joint = segment.getJoint();
        
        // 如果是关节（不是固定连接）
        if (joint.getType() != KDL::Joint::None) {
            // 获取当前关节索引
            int joint_idx = mkdl_chain_.getNrOfJoints() - 1; // 默认最后一个关节
            for (int j = 0; j < mkdl_chain_.getNrOfJoints(); ++j) {
                if (mkdl_chain_.getSegment(j).getJoint().getName() == joint.getName()) {
                    joint_idx = j;
                    break;
                }
            }

            // 计算前一个位姿到当前位姿的变换
            //frames[i + 1] = frames[i] * segment.getFrameToTip() * joint_frame;
            //已知关节角度，计算父连杆末端到关节坐标系的变换矩阵
            double joint_angle = jnt_array(joint_id);
            KDL::Frame joint_frame;
            switch (joint.getType())
            {
            case KDL::Joint::RotAxis:
                //旋转轴关节 -- 需要获取关节旋转方向    
                joint_frame = KDL::Frame(KDL::Rotation::Rot(joint.JointAxis(), joint_angle));
                break;
            case KDL::Joint::RotX:
                joint_frame = KDL::Frame(KDL::Rotation::RotX(joint_angle));
                break;
            case KDL::Joint::RotY:
                joint_frame = KDL::Frame(KDL::Rotation::RotY(joint_angle));
                break;
            case KDL::Joint::RotZ:
                joint_frame = KDL::Frame(KDL::Rotation::RotZ(joint_angle));
                break;
            default:
                break;
            }

            KDL::Frame tempFrame = segment.getFrameToTip();     //计算关节坐标系到连杆末端的变换矩阵
            frames[i + 1] = tempFrame * joint_frame;
            joint_id = joint_id + 1;

        } else {
            // 固定连接，直接应用segment的变换
            frames[i + 1] = segment.getFrameToTip();
        }
    }

    // 发布所有连杆的TF
    auto now = global_timestamp_;
    for (size_t i = 0; i < frames.size(); ++i) {
        if (i == 0) continue; // 跳过基座
        
        const KDL::Segment& segment = mkdl_chain_.getSegment(i - 1);
        std::string parent_frame = (i == 1) ? base_link[URDFCHOOSE] : mkdl_chain_.getSegment(i - 2).getName();
        std::string child_frame = segment.getName();

        // 创建TF变换消息
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now;
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        
        // 从KDL Frame转换
        transform.transform.translation.x = frames[i].p.x();
        transform.transform.translation.y = frames[i].p.y();
        transform.transform.translation.z = frames[i].p.z();
        
        double roll, pitch, yaw;
        frames[i].M.GetRPY(roll, pitch, yaw);
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();
        
        // 发布TF
        tf_broadcaster_->sendTransform(transform);
    }
}

// 发布关节状态
void ArmCtlNode::publish_joint_states(
    KDL::Chain mkdl_chain_, std::vector<double> mjoint_pos, 
    std::vector<double> mjoint_spd, std::vector<double> mjoint_tor
){
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = global_timestamp_;
    joint_msg.header.frame_id = base_link[URDFCHOOSE];

    // 获取URDF中所有关节名称
    std::vector<std::string> joint_names;
    for (const auto& segment : mkdl_chain_.segments) {
        const KDL::Joint& joint = segment.getJoint();
        if (joint.getType() != KDL::Joint::None) {
            joint_names.push_back(joint.getName());
        }
    }
    joint_msg.name = joint_names;
    joint_msg.position = mjoint_pos;
    joint_msg.velocity = mjoint_spd;
    joint_msg.effort = mjoint_tor;


    joint_state_pub_->publish(joint_msg);
}

//debug画图信息发送模块
void ArmCtlNode::dbg_publish(std::vector<double> mInp_1, int len){
    auto tempDbgData_pub = snowbot_ctl_msgs::msg::DbgInformation();
    
    int temp_len = len;
    if(len > 7){
        temp_len = 7;
    }

    for(int i = 0; i < temp_len; i++){
        tempDbgData_pub.float_reserve[i] = mInp_1[i];
    }

    mDbgData_pub->publish(tempDbgData_pub);
}

//第二种debug画图信息发送模块——一次发送两个轨迹的信息
void ArmCtlNode::dbg_publish_2(std::vector<double> mInp_1, std::vector<double> mInp_2){
    auto tempDbgData_pub = snowbot_ctl_msgs::msg::DbgInformation();

    int temp_len_1 = mInp_1.size();
    if(temp_len_1 > 7){
        temp_len_1 = 7;
    }
    for(int i = 0; i < temp_len_1; i++){
        tempDbgData_pub.float_reserve[i] = mInp_1[i];
    }

    int temp_len_2 = mInp_2.size();
    if(temp_len_2 > 7){
        temp_len_2 = 7;
    }
    for(int i = 0; i < temp_len_2; i++){
        tempDbgData_pub.float_reserve_b[i] = mInp_2[i];
    }

    mDbgData_pub->publish(tempDbgData_pub);
}

//------------------------------
//ArmInterface接口交互模块

//初始化模块
void ArmCtlNode::InterfaceInit(){
    //初始化模块，采用返回值进行初始化

    //初始化量解包
    KDL::JntArray lArm_stPos, rArm_stPos;
    lArm_stPos = vec_to_JntArray(lArm_stPos_get);
    rArm_stPos = vec_to_JntArray(rArm_stPos_get);

    //求解器初始化
    solverInterface.init(
        lArm_stPos, rArm_stPos,
        lArm_LimMsg_get, rArm_LimMsg_get,
        modCtlMsg_get,
        printMsgPkg_get,
        optimizerPara_get
    );
    std::cout << "初始化完成" <<std::endl;
}

//进行一轮控制计算的模块
void ArmCtlNode::controlCal(){
    KDL::Frame lArm_desPos_Frame, rArm_desPos_Frame;
    KDL::JntArray lArm_desPos_Jnt, rArm_desPos_Jnt;
    KDL::JntArray lArm_posRet_Jnt, rArm_posRet_Jnt;
    KDL::JntArray lArm_velRet_Jnt, rArm_velRet_Jnt;

    lArm_desPos_Frame = vec_to_Frame(lArm_desPos_descartes);
    rArm_desPos_Frame = vec_to_Frame(rArm_desPos_descartes);
    
    lArm_desPos_Jnt = vec_to_JntArray(lArm_desPos_JntSpace);
    rArm_desPos_Jnt = vec_to_JntArray(rArm_desPos_JntSpace);

    lArm_posRet_Jnt = vec_to_JntArray(lArm_pos_fdback);
    rArm_posRet_Jnt = vec_to_JntArray(rArm_pos_fdback);
    lArm_velRet_Jnt = vec_to_JntArray(lArm_spd_fdback);
    rArm_velRet_Jnt = vec_to_JntArray(rArm_spd_fdback);

    //进行一轮计算
    solverInterface.Traj_cal(
        ifUpdateLim_get,
        allArm_LimMsg_get,

        lArm_desPos_Frame, rArm_desPos_Frame,
        lArm_desTor_descartes, rArm_desTor_descartes,

        lArm_desPos_Jnt, rArm_desPos_Jnt,
        lArm_desTor_JntSpace, rArm_desTor_JntSpace,

        lArm_posRet_Jnt, rArm_posRet_Jnt,
        lArm_velRet_Jnt, rArm_velRet_Jnt,
        lArm_iq_fdback, rArm_iq_fdback,

        modCtlMsg_get,
        ifRedefineJntTrajStab_get
    );
}

// 回零控制初始化计算代码
void ArmCtlNode::retZeroControlInit(){
    // ---------- 刷新控制参量 ---------- 
    allArm_LimMsg_get.ifUseJntLim = false;

    modCtl temp_modMsg = modCtlMsg_get;
    temp_modMsg.transMod = 1;
    temp_modMsg.runinMod = 3;
    temp_modMsg.ifUseTemp = false;
    for(int i = 0; i < SINGLEARMDOF; i++){
        temp_modMsg.base_rigiditiCtl_mod_l[i] = 1;
        temp_modMsg.base_rigiditiCtl_mod_r[i] = 1;
    }
    temp_modMsg.gravComp_ctl.ifUseGravComp = 1;
    temp_modMsg.gravComp_ctl.mIfUseFricComp = 0;
    temp_modMsg.gravComp_ctl.mIfUseInpFtr = 1;
    temp_modMsg.gravComp_ctl.mIfUseOutpFtr = 0;
    temp_modMsg.gravComp_ctl.GravCompInpMod = 1;

    std::vector<std::vector<float>> TrajTime, accRateBase;
    TrajTime.resize(2);
    TrajTime[0].assign(LARMDOF, 2.0f);
    TrajTime[1].assign(RARMDOF, 2.0f);
    accRateBase.resize(2);
    accRateBase[0].assign(LARMDOF, 1.2f);
    accRateBase[1].assign(RARMDOF, 1.2f);
    temp_modMsg.TrajTime = TrajTime;
    temp_modMsg.accRateBase = accRateBase;
    for(int i = 0; i < LARMDOF; i++){
        temp_modMsg.m_stabMsg[0][i].ifUseStab = true;
        temp_modMsg.m_stabMsg[0][i].start_stabTimes = 50;
        temp_modMsg.m_stabMsg[0][i].des_stabTimes = 30;
    }
    for(int i = 0; i < RARMDOF; i++){
        temp_modMsg.m_stabMsg[1][i].ifUseStab = true;
        temp_modMsg.m_stabMsg[1][i].start_stabTimes = 50;
        temp_modMsg.m_stabMsg[1][i].des_stabTimes = 50;
    }

    std::vector<std::vector<int>> temp_ifJntStab;
    temp_ifJntStab.resize(2);
    temp_ifJntStab[0].assign(LARMDOF, 0);
    temp_ifJntStab[1].assign(RARMDOF, 0);
    temp_modMsg.ifJntStab = temp_ifJntStab;

    // ---------- 执行计算任务 ----------
    KDL::Frame lArm_desPos_Frame, rArm_desPos_Frame;
    KDL::JntArray lArm_desPos_Jnt, rArm_desPos_Jnt;
    KDL::JntArray lArm_posRet_Jnt, rArm_posRet_Jnt;
    KDL::JntArray lArm_velRet_Jnt, rArm_velRet_Jnt;

    lArm_desPos_Frame = vec_to_Frame(l_descZeroPos);
    rArm_desPos_Frame = vec_to_Frame(r_descZeroPos);

    lArm_desPos_Jnt = vec_to_JntArray(lArm_zeroPos);
    rArm_desPos_Jnt = vec_to_JntArray(rArm_zeroPos);

    lArm_posRet_Jnt = vec_to_JntArray(lArm_pos_fdback);
    rArm_posRet_Jnt = vec_to_JntArray(rArm_pos_fdback);
    lArm_velRet_Jnt = vec_to_JntArray(lArm_spd_fdback);
    rArm_velRet_Jnt = vec_to_JntArray(rArm_spd_fdback);

    // 进行一轮计算
    solverInterface.Traj_cal(
        false,
        allArm_LimMsg_get,

        lArm_desPos_Frame, rArm_desPos_Frame,
        lArm_desTor_descartes, rArm_desTor_descartes,

        lArm_desPos_Jnt, rArm_desPos_Jnt,
        lArm_desTor_JntSpace, rArm_desTor_JntSpace,

        lArm_posRet_Jnt, rArm_posRet_Jnt,
        lArm_velRet_Jnt, rArm_velRet_Jnt,
        lArm_iq_fdback, rArm_iq_fdback,

        temp_modMsg,
        false
    );
}

//控制封装代码
void ArmCtlNode::control_seal(){

    //如果没有接收到下位机的反馈值，强制不使用返回值，并强制各个初始化过程需要注入的参数刷零
    if(!has_received_command_){
        modCtlMsg_get.ifUseRet = 0;
    }

    //先进行一轮初始化 -- 注意执行第一轮要确保强制刷新
    if(ifFirstTemp || ifReInit_get){
        InterfaceInit();
        ifIKsolverInited = true;
    }

    //再进行一轮控制计算 -- 注意满足条件或第一轮情况下计算
    bool all_lArmEnd = std::all_of(lArm_ifReachEnd.begin(), lArm_ifReachEnd.end(), [](bool val) { return val; });
    bool all_rArmEnd = std::all_of(rArm_ifReachEnd.begin(), rArm_ifReachEnd.end(), [](bool val) { return val; });
    bool ifAllEnd = all_lArmEnd && all_rArmEnd;

    if(modCtlMsg_get.transMod != 0){
        //其他控制模式
        if(ifFirstTemp || ifAllEnd){
            controlCal();
        }
        ifFirstTemp = false;

        //进行输出
        controlOutp();

        //向着上位机输出反馈值
        midMonitor_msg_publish();

        has_received_upperMonitor_command_ = false;
    }else{
        //回零位模式
        if(ifAllEnd){
            retZeroControlInit();
        }

        //进行输出
        controlOutp();

        //向着上位机输出反馈值
        midMonitor_msg_publish();

        has_received_upperMonitor_command_ = false;
    }
}

//注意修改执行逻辑，有的时候这个ifReachEnd输出的不正确，例子：轨迹2,另外要添加
//上层规划器令中层规划器刷新的机制。（强制令中层规划器重新初始化）

//进行一轮输出
void ArmCtlNode::controlOutp(){
    KDL::JntArray lArm_posRet, rArm_posRet;
    KDL::JntArray lArm_velRet, rArm_velRet;
    
    lArm_posRet = vec_to_JntArray(lArm_pos_fdback);
    rArm_posRet = vec_to_JntArray(rArm_pos_fdback);

    lArm_velRet = vec_to_JntArray(lArm_spd_fdback);
    rArm_velRet = vec_to_JntArray(rArm_spd_fdback);

    std::vector<outpWrap> mRes;
    mRes = solverInterface.Traj_temp_outp(
        lArm_posRet, rArm_posRet,
        lArm_velRet, rArm_velRet,
        lArm_iq_fdback, rArm_iq_fdback
    );

    //输出解包
    lArm_joint_pos = mRes[0].posDes;
    lArm_joint_spd = mRes[0].velDes;

    lArm_joint_tor_grav = mRes[0].gravComp_TorDes;
    lArm_joint_tor_interia = mRes[0].InteriaComp_TorDes;
    lArm_joint_tor_feedforward = mRes[0].FeedForward_TorDes;
    
    lArm_joint_kp = mRes[0].KpDes;
    lArm_joint_kd = mRes[0].KdDes;

    rArm_joint_pos = mRes[1].posDes;
    rArm_joint_spd = mRes[1].velDes;

    rArm_joint_tor_grav = mRes[1].gravComp_TorDes;
    rArm_joint_tor_interia = mRes[1].InteriaComp_TorDes;
    rArm_joint_tor_feedforward = mRes[1].FeedForward_TorDes;

    rArm_joint_kp = mRes[1].KpDes;
    rArm_joint_kd = mRes[1].KdDes;

    lArm_ifReachEnd = mRes[0].ifComToEnd;
    rArm_ifReachEnd = mRes[1].ifComToEnd;

    //若使用了自定义的PD参数，则需要覆盖原PD参数
    for(int i = 0; i < SINGLEARMDOF; i++){
        if(modCtlMsg_get.base_rigiditiCtl_mod_l[i] == -1){
            lArm_joint_kp[i] = lArm_Kp_[i];
            lArm_joint_kd[i] = lArm_Kd_[i];
        }

        if(modCtlMsg_get.base_rigiditiCtl_mod_r[i] == -1){
            rArm_joint_kp[i] = rArm_Kp_[i];
            rArm_joint_kd[i] = rArm_Kd_[i];
        }
    }

}

//------------------------------
//ros传输变量初始化模块

void ArmCtlNode::rosloop_var_init(){
    //---------------中层节点与下位机通讯全局变量---------------

    //控制量发送：
    runin_mod = -1;
    if_runin_renew = 0;
    ifUseRet_toStRunin = 0;
    ifUseDebugMod = 0;
    for(int i = 0; i < SINGLEARMDOF; i++){
        base_rigidityCtl_mod_l[i] = 1;
        base_rigidityCtl_mod_r[i] = 1;
    }

    //左右臂初始值刷新
    lArm_pos_atRuninSt = lArm_zeroPos;
    rArm_pos_atRuninSt = rArm_zeroPos;

    //目标参量发送：
    lArm_joint_pos = lArm_zeroPos;
    lArm_joint_kp = lArm_defaultKp_low;
    lArm_joint_kd = lArm_defaultKd_low;
    lArm_joint_spd.assign(SINGLEARMDOF, 0.0);
    lArm_joint_tor_grav.assign(SINGLEARMDOF, 0.0);
    lArm_joint_tor_interia.assign(SINGLEARMDOF, 0.0);
    lArm_joint_tor_feedforward.assign(SINGLEARMDOF, 0.0);

    rArm_joint_pos = rArm_zeroPos;
    rArm_joint_kp = rArm_defaultKp_low;
    rArm_joint_kd = rArm_defaultKd_low;
    rArm_joint_spd.assign(SINGLEARMDOF, 0.0);
    rArm_joint_tor_grav.assign(SINGLEARMDOF, 0.0);
    rArm_joint_tor_interia.assign(SINGLEARMDOF, 0.0);
    rArm_joint_tor_feedforward.assign(SINGLEARMDOF, 0.0);

    lArm_torFeedForward_assist.assign(SINGLEARMDOF, 0.0);
    rArm_torFeedForward_assist.assign(SINGLEARMDOF, 0.0);

    //反馈接收：
    lArm_pos_fdback = lArm_zeroPos;
    lArm_spd_fdback.assign(SINGLEARMDOF, 0.0);
    lArm_iq_fdback.assign(SINGLEARMDOF, 0.0);
    rArm_pos_fdback = rArm_zeroPos;
    rArm_spd_fdback.assign(SINGLEARMDOF, 0.0);
    rArm_iq_fdback.assign(SINGLEARMDOF, 0.0);

    if_lower_firstTime = true;
    if_middle_firstTime = true;

    //控制状态向上位机反馈：
    lArm_ifReachEnd.assign(SINGLEARMDOF, true);
    rArm_ifReachEnd.assign(SINGLEARMDOF, true);

    //---------------中层节点与上位机通讯全局变量---------------

    //控制过程信息打包
    ifReInit_get = false;
    ifUpdateLim_get = false;
    ifRedefineJntTrajStab_get = false;

    //左右臂关节初始位置
    lArm_stPos_get = lArm_zeroPos;
    rArm_stPos_get = rArm_zeroPos;
    ifStartCal = false;

    //目标位置等信息
    lArm_desPos_descartes = l_descZeroPos;
    lArm_desPos_JntSpace = lArm_zeroPos;
    lArm_desTor_descartes.assign(SINGLEARMDOF, 0.0);
    lArm_desTor_JntSpace.assign(SINGLEARMDOF, 0.0);
    rArm_desPos_descartes = r_descZeroPos;
    rArm_desPos_JntSpace = rArm_zeroPos;
    rArm_desTor_descartes.assign(6, 0.0);
    rArm_desTor_JntSpace.assign(SINGLEARMDOF, 0.0);

    //搭配自定义PD参数模式使用
    lArm_Kp_ = lArm_defaultKp_low;
    lArm_Kd_ = lArm_defaultKd_low;
    rArm_Kp_ = rArm_defaultKp_low;
    rArm_Kd_ = rArm_defaultKd_low;

    //debug用的辅助变量
    vec_dbg_1.assign(SINGLEARMDOF, 0.0);
    vec_dbg_2.assign(SINGLEARMDOF, 0.0);

    std::cout << "全局变量初始化完毕 (rosloop_var_init)" << std::endl;
}

//------------------------------
//ros2 循环函数

void ArmCtlNode::run(){
    //用户交互
    std::cout << "开始执行中层节点封装程序" << std::endl;

    //初始化全局变量
    rosloop_var_init();

    //初始化python解释器：
    PyGILState_STATE gstate = PyGILState_Ensure();

    //初始化执行频率
    float loopRate_unRound = 1 / TEMPTIME;
    int loopRate = std::round(loopRate_unRound);
    std::cout << "轨迹执行频率为：" << loopRate <<" Hz。" << std::endl;
    rclcpp::Rate loop_rate(loopRate); // 50Hz

    //开始ros2循环
    int i = 0;
    int j = 0;
    std::vector<double> topic_point_target_l;
    std::vector<double> topic_point_target_r;
    while(rclcpp::ok() && !shutdown_flag){
        //更新时间戳
        global_timestamp_ = this->now();
        //获取一次反馈值
        fdback_get();

        if(checkTemp){
            //检查轮 -- 注意中层节点第一轮是不能接收到上层节点的反馈值的
            if(has_received_upperMonitor_command_){
                std::cout << "警告：第一轮控制中中层节点需要在上层节点之前打开，退出程序以保证安全" << std::endl;
                break;
            }
            checkTemp = false;
        }

        //第一轮执行时，自动检测位置
        if(ifInitTemp){
            for(int i = 0; i < 10; i++){
                fdback_get();
            }
            for(int i = 0; i < SINGLEARMDOF; i++){
                std::cout << "lr pos in " << i << " = " << lArm_pos_fdback[i] << " || " << rArm_pos_fdback[i] << std::endl;
            }
            ifInitTemp = false;
        }

        // 电机使能: 收到上位机命令→使能, 空闲超时→抱闸
        if(has_received_upperMonitor_command_){
            motor_enable_ = 1;
            motor_idle_count_ = 0;
        } else {
            motor_idle_count_++;
            if(motor_idle_count_ > MOTOR_IDLE_TIMEOUT){
                motor_enable_ = 0;
            }
        }

        //若能接受到上位机的反馈值时，开始计算
        if(has_received_upperMonitor_command_ ){

            //如果没有接收到下位机的反馈值，强制不使用返回值，并强制各个初始化过程需要注入的参数刷零
            if(!has_received_command_){
                modCtlMsg_get.ifUseRet = 0;
            }

            //先进行一轮初始化 -- 注意执行第一轮要确保强制刷新
            if(ifFirstTemp || ifReInit_get){
                InterfaceInit();
                ifIKsolverInited = true;
            }

            //再进行一轮控制计算 -- 注意满足条件或第一轮情况下计算
            bool all_lArmEnd = std::all_of(lArm_ifReachEnd.begin(), lArm_ifReachEnd.end(), [](bool val) { return val; });
            bool all_rArmEnd = std::all_of(rArm_ifReachEnd.begin(), rArm_ifReachEnd.end(), [](bool val) { return val; });
            bool ifAllEnd = all_lArmEnd && all_rArmEnd;

            // Demo/VR 保持原有的到位刷新逻辑。Topic 单点位只在目标变化且
            // 上一段完成后重规划；已到位且目标不变时保持终点，不再反复生成微小轨迹。
            const bool topic_point_mode = modCtlMsg_get.ifStreamingJointPosition;
            const auto same_target = [](const std::vector<double>& a,
                                        const std::vector<double>& b) {
                return a.size() == b.size() &&
                    std::equal(a.begin(), a.end(), b.begin(),
                        [](double x, double y) { return std::abs(x - y) < 1e-9; });
            };
            const bool topic_target_changed =
                !same_target(lArm_desPos_JntSpace, topic_point_target_l) ||
                !same_target(rArm_desPos_JntSpace, topic_point_target_r);
            // Topic 单点命令是一次新的目标，不应被上一段轨迹的到位状态挡住；
            // Demo/VR 仍保持原来的“到位后刷新”逻辑。
            const bool recalculate = ifFirstTemp ||
                (topic_point_mode ? topic_target_changed : ifAllEnd);
            if(recalculate){
                controlCal();
                if(topic_point_mode){
                    topic_point_target_l = lArm_desPos_JntSpace;
                    topic_point_target_r = rArm_desPos_JntSpace;
                }
            }
            ifFirstTemp = false;

            //进行输出
            controlOutp();

            //向着上位机输出反馈值
            midMonitor_msg_publish();
            has_received_upperMonitor_command_ = false;
            

        }
        else{
            /*
            此时应当进入位置保持模式，因此位置、Kp、Kd、重力补偿不刷新，
            前馈力矩刷新成上一轮的控制器下发目标力矩
            */

            //进行输出
            if(ifIKsolverInited){
                controlOutp();
            }

            // 轨迹结束后(所有关节到达终点)才归零速度, 运行中保留插补速度
            bool all_lArmEnd = std::all_of(lArm_ifReachEnd.begin(), lArm_ifReachEnd.end(), [](bool v) { return v; });
            bool all_rArmEnd = std::all_of(rArm_ifReachEnd.begin(), rArm_ifReachEnd.end(), [](bool v) { return v; });
            if (all_lArmEnd && all_rArmEnd && !modCtlMsg_get.ifStreamingJointPosition) {
                lArm_joint_spd.assign(SINGLEARMDOF, 0.0);
                rArm_joint_spd.assign(SINGLEARMDOF, 0.0);
            }
            lArm_joint_tor_interia.assign(SINGLEARMDOF, 0.0);
            rArm_joint_tor_interia.assign(SINGLEARMDOF, 0.0);

            lArm_joint_tor_feedforward = lArm_torFeedForward_assist;
            rArm_joint_tor_feedforward = rArm_torFeedForward_assist;

            //向着上位机输出反馈值
            midMonitor_msg_publish();
        }

        if(ifIKsolverInited){
            //----------执行末端轨迹画图任务----------
            //先获取返回值
            KDL::JntArray lArm_JntPosFdback_kdl, rArm_JntPosFdback_kdl;
            std::vector<KDL::Frame> Arm_descPosFdback_kdl;
            std::vector<double> lArm_descPos_fdback, rArm_descPos_fdback;

            lArm_JntPosFdback_kdl.resize(SINGLEARMDOF);
            rArm_JntPosFdback_kdl.resize(SINGLEARMDOF);
            lArm_descPos_fdback.assign(6, 0.0);
            rArm_descPos_fdback.assign(6, 0.0);

            for(int i = 0; i < SINGLEARMDOF; i++){
                lArm_JntPosFdback_kdl(i) = lArm_pos_fdback[i];
                rArm_JntPosFdback_kdl(i) = rArm_pos_fdback[i];
            }
            Arm_descPosFdback_kdl = solverInterface.FKsolver(
                lArm_JntPosFdback_kdl, rArm_JntPosFdback_kdl
            );

            lArm_descPos_fdback[0] = Arm_descPosFdback_kdl[0].p[0];
            lArm_descPos_fdback[1] = Arm_descPosFdback_kdl[0].p[1];
            lArm_descPos_fdback[2] = Arm_descPosFdback_kdl[0].p[2];
            Arm_descPosFdback_kdl[0].M.GetRPY(lArm_descPos_fdback[3], lArm_descPos_fdback[4], lArm_descPos_fdback[5]);

            rArm_descPos_fdback[0] = Arm_descPosFdback_kdl[1].p[0];
            rArm_descPos_fdback[1] = Arm_descPosFdback_kdl[1].p[1];
            rArm_descPos_fdback[2] = Arm_descPosFdback_kdl[1].p[2];
            Arm_descPosFdback_kdl[1].M.GetRPY(rArm_descPos_fdback[3], rArm_descPos_fdback[4], rArm_descPos_fdback[5]);

            //再发布
            endPosTrajPlot->end_pos_plot(
                lArm_desPos_descartes, rArm_desPos_descartes,
                lArm_descPos_fdback, rArm_descPos_fdback
            );
        }

        //向下位机发送数据，同步给rviz2发送数据
        msg_publish();

        publish_body_head_tf();
        publish_joint_states(kdl_lArm_chain_, lArm_joint_pos, lArm_joint_spd, lArm_joint_tor_grav);
        publish_joint_states(kdl_rArm_chain_, rArm_joint_pos, rArm_joint_spd, rArm_joint_tor_grav);
        publish_tf_from_joint_states(kdl_lArm_chain_, lArm_joint_pos);
        publish_tf_from_joint_states(kdl_rArm_chain_, rArm_joint_pos);

        has_received_command_ = false;

        //末处理：
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }

    PyGILState_Release(gstate);
    RCLCPP_INFO(this->get_logger(), "程序已安全退出");
}

//------------------------------
//回调函数

void ArmCtlNode::set_joint_angles(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        // 这里可以添加设置关节角度的逻辑
        RCLCPP_INFO(this->get_logger(), "Received request to set joint angles");
        response->success = true;
        response->message = "Joint angles set successfully";
    } else {
        response->success = false;
        response->message = "Request not confirmed";
    }
}

void ArmCtlNode::message_callback(const snowbot_ctl_msgs::msg::RetInformation::SharedPtr msg) {

    // 检查时间戳是否合理
    auto now = this->now();
    auto latency = now - msg->header.stamp;
    if (latency.seconds() > 0.05) {
        RCLCPP_WARN(this->get_logger(), "High latency detected: %f seconds", latency.seconds());
    }

    for(int i = 0; i < LARMDOF; i++){
        lArm_pos_fdback[i] = msg->l_position[i];
        lArm_spd_fdback[i] = msg->l_speed[i];
        lArm_iq_fdback[i] = msg->l_iq[i];
    }

    for(int i = 0; i < RARMDOF; i++){
        rArm_pos_fdback[i] = msg->r_position[i];
        rArm_spd_fdback[i] = msg->r_speed[i];
        rArm_iq_fdback[i] = msg->r_iq[i];
    }

    this->has_received_command_ = true;
}

//从上位机获取的控制数据
void ArmCtlNode::upperMonitor_msg_callback(const snowbot_ctl_msgs::msg::MidNodeCtlMsgSend::SharedPtr msg)
{
    //----------综合模式控制信息解包----------

    std::cout << "upper monitor data get success" << std::endl;

    //数据传输模式
    modCtlMsg_get.transMod = msg->ctl_trans_mod;
    modCtlMsg_get.runinMod = msg->runin_mod;

    //求解模式
    modCtlMsg_get.ifUseRet = msg->if_use_ret;
    modCtlMsg_get.ifUseTemp = msg->if_use_temp;
    modCtlMsg_get.ifUseRetToStartRunin = msg->if_use_ret_to_start_runin;
    modCtlMsg_get.ifStreamingJointPosition = msg->if_streaming_joint_position;
    ifUseRet_toStRunin = msg->if_use_ret_to_start_runin ? 1 : 0;
    // Topic 点位模式也使用 Demo 的样条细分；“streaming”字段这里只作为
    // Topic 点位来源标识，不再触发每帧重规划。
    if (msg->if_streaming_joint_position) {
        runin_mod = 2;
    }

    for(int i = 0; i < SINGLEARMDOF; i++){
        modCtlMsg_get.base_rigiditiCtl_mod_l[i] = msg->rigidity_ctl_mod_l[i];
        modCtlMsg_get.base_rigiditiCtl_mod_r[i] = msg->rigidity_ctl_mod_r[i];

        base_rigidityCtl_mod_l[i] = msg->rigidity_ctl_mod_l[i];
        base_rigidityCtl_mod_r[i] = msg->rigidity_ctl_mod_r[i];
    }
    //std::cout << "base_rigidityCtl_mod_l[0] = " <<  << std::endl;

    //重力补偿控制
    modCtlMsg_get.gravComp_ctl.ifUseGravComp = msg->if_use_grav_comp;
    modCtlMsg_get.gravComp_ctl.mIfUseFricComp = msg->if_use_fric_comp;
    modCtlMsg_get.gravComp_ctl.mIfUseInpFtr = msg->if_use_inp_ftr;
    modCtlMsg_get.gravComp_ctl.mIfUseOutpFtr = msg->if_use_outp_ftr;
    modCtlMsg_get.gravComp_ctl.GravCompInpMod = msg->grav_comp_inp_mod;

    modCtlMsg_get.gravComp_ctl.stJntID_lArm = msg->st_jnt_id_l_arm;
    modCtlMsg_get.gravComp_ctl.stJntID_rArm = msg->st_jnt_id_r_arm;
    modCtlMsg_get.gravComp_ctl.urdf_choose = msg->urdf_choose;

    //插补信息包装
    modCtlMsg_get.upperMonitor_time = msg->upper_monitor_time;
    modCtlMsg_get.TrajTime.resize(2);
    modCtlMsg_get.accRateBase.resize(2);
    modCtlMsg_get.m_stabMsg.resize(2);

    modCtlMsg_get.TrajTime[0].assign(SINGLEARMDOF, 0.0);
    modCtlMsg_get.accRateBase[0].assign(SINGLEARMDOF, 0.0);
    modCtlMsg_get.m_stabMsg[0].resize(SINGLEARMDOF);

    modCtlMsg_get.TrajTime[1].assign(SINGLEARMDOF, 0.0);
    modCtlMsg_get.accRateBase[1].assign(SINGLEARMDOF, 0.0);
    modCtlMsg_get.m_stabMsg[1].resize(SINGLEARMDOF);

    for(int i = 0; i < SINGLEARMDOF; i++){
        modCtlMsg_get.TrajTime[0][i] = msg->traj_time_l_arm[i];
        modCtlMsg_get.accRateBase[0][i] = msg->acc_rate_base_l_arm[i];

        modCtlMsg_get.m_stabMsg[0][i].ifUseStab = msg->l_arm_if_use_stab[i];
        modCtlMsg_get.m_stabMsg[0][i].des_stabTimes = msg->l_arm_des_stab_times[i];
        modCtlMsg_get.m_stabMsg[0][i].start_stabTimes = msg->l_arm_start_stab_times[i];
    }

    for(int i = 0; i < SINGLEARMDOF; i++){
        modCtlMsg_get.TrajTime[1][i] = msg->traj_time_r_arm[i];
        modCtlMsg_get.accRateBase[1][i] = msg->acc_rate_base_r_arm[i];

        modCtlMsg_get.m_stabMsg[1][i].ifUseStab = msg->r_arm_if_use_stab[i];
        modCtlMsg_get.m_stabMsg[1][i].start_stabTimes = msg->r_arm_start_stab_times[i];
        modCtlMsg_get.m_stabMsg[1][i].des_stabTimes = msg->r_arm_des_stab_times[i];
    }

    //控制量包装 -- 各个关节是否使用静止输出模式
    modCtlMsg_get.ifJntStab.resize(2);
    modCtlMsg_get.ifJntStab[0].assign(SINGLEARMDOF, 0);
    modCtlMsg_get.ifJntStab[1].assign(SINGLEARMDOF, 0);
    for(int i = 0; i < SINGLEARMDOF; i++){
        modCtlMsg_get.ifJntStab[0][i] = msg->l_arm_if_jnt_stab[i];
        modCtlMsg_get.ifJntStab[1][i] = msg->r_arm_if_jnt_stab[i];
    }

    ifUpdateLim_get = msg->if_update_lim;
    ifRedefineJntTrajStab_get = msg->if_redefine_stab_jnt_ang;

    //----------关节控制信息参数解包----------

    int lArmLimJntNumGet = msg->l_arm_jntlim_num;
    int rArmLimJntNumGet = msg->r_arm_jntlim_num;
    int allArmLimJntNumGet = lArmLimJntNumGet + rArmLimJntNumGet;

    //左臂
    lArm_LimMsg_get.ifUseJntLim = msg->l_arm_if_use_jntlim;
    lArm_LimMsg_get.limJntNum = msg->l_arm_jntlim_num;
    lArm_LimMsg_get.limJntLabel.resize(msg->l_arm_jntlim_num);
    lArm_LimMsg_get.limJntAng.resize(msg->l_arm_jntlim_num);
    for(int i = 0; i < lArmLimJntNumGet; i++){
        lArm_LimMsg_get.limJntLabel[i] = msg->l_arm_limjnt_label[i];
        lArm_LimMsg_get.limJntAng[i] = msg->l_arm_limjnt_ang[i];
    }

    //右臂
    rArm_LimMsg_get.ifUseJntLim = msg->r_arm_if_use_jntlim;
    rArm_LimMsg_get.limJntNum = msg->r_arm_jntlim_num;
    rArm_LimMsg_get.limJntLabel.resize(msg->r_arm_jntlim_num);
    rArm_LimMsg_get.limJntAng.resize(msg->r_arm_jntlim_num);
    for(int i = 0; i < rArmLimJntNumGet; i++){
        rArm_LimMsg_get.limJntLabel[i] = msg->r_arm_limjnt_label[i];
        rArm_LimMsg_get.limJntAng[i] = msg->r_arm_limjnt_ang[i];
    }

    //双臂 -- 将左右臂信息依次装入到双臂中
    allArm_LimMsg_get.ifUseJntLim = lArm_LimMsg_get.ifUseJntLim || rArm_LimMsg_get.ifUseJntLim;
    allArm_LimMsg_get.limJntNum = allArmLimJntNumGet;
    allArm_LimMsg_get.limJntLabel.resize(allArmLimJntNumGet);
    allArm_LimMsg_get.limJntAng.resize(allArmLimJntNumGet);
    int m_temp = 0;
    if(lArm_LimMsg_get.ifUseJntLim){
        for(int i = 0; i < lArmLimJntNumGet; i++){
            allArm_LimMsg_get.limJntLabel[m_temp] = lArm_LimMsg_get.limJntLabel[i];
            allArm_LimMsg_get.limJntAng[m_temp] = lArm_LimMsg_get.limJntAng[i];
            m_temp += 1;
        }
    }
    if(rArm_LimMsg_get.ifUseJntLim){
        for(int i = 0; i < rArmLimJntNumGet; i++){
            allArm_LimMsg_get.limJntLabel[m_temp] = rArm_LimMsg_get.limJntLabel[i];
            allArm_LimMsg_get.limJntAng[m_temp] = rArm_LimMsg_get.limJntAng[i];
            m_temp += 1;
        }
    }

    //----------关节初始化信息----------

    //判断是否重新初始化
    ifReInit_get = msg->if_reinit;

    //信息打印开关量初始化
    printMsgPkg_get.dispJntInfo = msg->if_disp_jnt_info;
    printMsgPkg_get.use_visualization = msg->if_use_visualization;
    printMsgPkg_get.disp_messages = msg->if_disp_msgs;

    //优化器参数初始化
    optimizerPara_get.errTol = msg->err_tol;
    optimizerPara_get.template_times = msg->solver_template_times;
    for(int i = 0; i < 5; i++){
        optimizerPara_get.OptiWeight[i] = msg->opti_weight[i];
    }
    optimizerPara_get.ifPrintLevel = msg->if_print_level;
    optimizerPara_get.ifPrintOptiTime = msg->if_print_optidata;

    //左右臂关节初始位置解包
    for(int i = 0; i < SINGLEARMDOF; i++){
        lArm_stPos_get[i] = msg->l_arm_st_pos[i];
    }
    for(int i = 0; i < SINGLEARMDOF; i++){
        rArm_stPos_get[i] = msg->r_arm_st_pos[i];
    }

    //----------关节目标位置、目标力矩等目标量信息----------
    
    for(int i = 0; i < 6; i++){
        lArm_desPos_descartes[i] = msg->l_arm_des_pos_descartes[i];
        lArm_desTor_descartes[i] = msg->l_arm_des_tor_descartes[i];
        
        rArm_desPos_descartes[i] = msg->r_arm_des_pos_descartes[i];
        rArm_desTor_descartes[i] = msg->r_arm_des_tor_descartes[i];
    }

    for(int i = 0; i < SINGLEARMDOF; i++){
        lArm_desPos_JntSpace[i] = msg->l_arm_des_pos_jntspace[i];
        lArm_desTor_JntSpace[i] = msg->l_arm_des_tor_jntspace[i];
    }
    for(int i = 0; i < SINGLEARMDOF; i++){
        rArm_desPos_JntSpace[i] = msg->r_arm_des_pos_jntspace[i];
        rArm_desTor_JntSpace[i] = msg->r_arm_des_tor_jntspace[i];
    }

    for(int i = 0; i < SINGLEARMDOF; i++){
        lArm_Kp_[i] = msg->kp_trans_l[i];
        lArm_Kd_[i] = msg->kd_trans_l[i];

        rArm_Kp_[i] = msg->kp_trans_r[i];
        rArm_Kd_[i] = msg->kd_trans_r[i];
    }

    has_received_upperMonitor_command_ = true;
}

//===================================================================================

EndPos_plot::EndPos_plot(rclcpp::Node* node): node_ptr_(node){
    //初始化消息发送变量：
    pvTrajPub = node_ptr_->create_publisher<snowbot_ctl_msgs::msg::EndDescPub>(
        "/feedback_get/end_desc_traj_pub",
        30
    );

    //初始化成员变量：
    lArm_DescPos_last.assign(SINGLEARMDOF, 0.0);
    rArm_DescPos_last.assign(SINGLEARMDOF, 0.0);
}

void EndPos_plot::end_pos_plot(
    std::vector<double> lArm_desDescPos, std::vector<double> rArm_desDescPos,
    std::vector<double> lArm_retDescPos, std::vector<double> rArm_retDescPos
){
    auto tempPV_pub = snowbot_ctl_msgs::msg::EndDescPub();

    //发送末端位置
    for(int i = 0; i < 6; i++){
        tempPV_pub.l_desc_pos_des[i] = lArm_desDescPos[i];
        tempPV_pub.r_desc_pos_des[i] = rArm_desDescPos[i];

        tempPV_pub.l_desc_pos_ret[i] = lArm_retDescPos[i];
        tempPV_pub.r_desc_pos_ret[i] = rArm_retDescPos[i];
    }

    //发送末端速度
    for(int i = 0; i < 6; i++){
        tempPV_pub.l_desc_vel_ret[i] = (lArm_retDescPos[i] - lArm_DescPos_last[i]) / temp_time;
        tempPV_pub.r_desc_vel_ret[i] = (rArm_retDescPos[i] - rArm_DescPos_last[i]) / temp_time;
    }

    //发送末端综合线速度大小
    double l_linerVel, r_linerVel;
    l_linerVel = std::sqrt(std::pow(tempPV_pub.l_desc_vel_ret[0], 2) + std::pow(tempPV_pub.l_desc_vel_ret[1], 2) + std::pow(tempPV_pub.l_desc_vel_ret[2], 2));
    r_linerVel = std::sqrt(std::pow(tempPV_pub.r_desc_vel_ret[0], 2) + std::pow(tempPV_pub.r_desc_vel_ret[1], 2) + std::pow(tempPV_pub.r_desc_vel_ret[2], 2));

    //更新上一轮位置
    lArm_DescPos_last = lArm_retDescPos;
    rArm_DescPos_last = rArm_retDescPos;

    tempPV_pub.l_desc_liner_vel = l_linerVel;
    tempPV_pub.r_desc_liner_vel = r_linerVel;

    pvTrajPub->publish(tempPV_pub);
}

#include "servo_control/zero_pos_config.h"
#include <iomanip>

std::string yaml_path_zeroPos;
std::string yaml_path_shutdownPos;
std::string yaml_path_initialPD;

//默认位置定义
std::vector<double> lArm_zeroPos;
std::vector<double> rArm_zeroPos;
double lGripper_zeroPos = 0.0;
double rGripper_zeroPos = 0.0;

//关机位置定义
std::vector<double> lArm_shutdownPos;
std::vector<double> rArm_shutdownPos;
double lGripper_shutdownPos = 0.0;
double rGripper_shutdownPos = 0.0;

//默认 PD 参数定义
std::vector<double> lArm_defaultKp;
std::vector<double> lArm_defaultKd;
std::vector<double> rArm_defaultKp;
std::vector<double> rArm_defaultKd;

//==============================================================================
// 构造函数
//==============================================================================

ZeroPosConfig::ZeroPosConfig()
    : lArm_zeroPos(LARMDOF, 0.0)
    , rArm_zeroPos(RARMDOF, 0.0)
    , yaml_path_("")
    , loaded_(false)
{
}

ZeroPosConfig::ZeroPosConfig(const std::string& yaml_path)
    : lArm_zeroPos(LARMDOF, 0.0)
    , rArm_zeroPos(RARMDOF, 0.0)
    , yaml_path_(yaml_path)
    , loaded_(false)
{
    loadFromYaml(yaml_path);
}

//==============================================================================
// 从 YAML 文件加载零位配置
//==============================================================================

bool ZeroPosConfig::loadFromYaml(const std::string& yaml_path){
    yaml_path_ = yaml_path;

    try{
        // 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(yaml_path_);

        // 读取左臂零位
        if(config["left_arm"]){
            const YAML::Node& left = config["left_arm"];
            lArm_zeroPos.clear();
            lArm_zeroPos.push_back(left["left_shoulder_pitch_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_shoulder_roll_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_shoulder_yaw_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_elbow_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_wrist_roll_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_wrist_pitch_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_wrist_yaw_joint"].as<double>(0.0));
        }else{
            std::cerr << "警告：zeropos.yaml 中未找到 left_arm 配置节点，"
                      << "使用默认零位 0.0" << std::endl;
            lArm_zeroPos.assign(LARMDOF, 0.0);
        }

        // 读取右臂零位
        if(config["right_arm"]){
            const YAML::Node& right = config["right_arm"];
            rArm_zeroPos.clear();
            rArm_zeroPos.push_back(right["right_shoulder_pitch_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_shoulder_roll_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_shoulder_yaw_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_elbow_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_wrist_roll_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_wrist_pitch_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_wrist_yaw_joint"].as<double>(0.0));
        }else{
            std::cerr << "警告：zeropos.yaml 中未找到 right_arm 配置节点，"
                      << "使用默认零位 0.0" << std::endl;
            rArm_zeroPos.assign(RARMDOF, 0.0);
        }

        // 读取夹爪零位
        lGripper_zeroPos = config["left_gripper"]["left_gripper_joint"].as<double>(0.0);
        rGripper_zeroPos = config["right_gripper"]["right_gripper_joint"].as<double>(0.0);

        // 检查读取的维度是否正确
        if(lArm_zeroPos.size() != LARMDOF){
            std::cerr << "错误：左臂零位配置维度 " << lArm_zeroPos.size()
                      << " 与 LARMDOF=" << LARMDOF << " 不匹配" << std::endl;
            lArm_zeroPos.assign(LARMDOF, 0.0);
            loaded_ = false;
            return false;
        }
        if(rArm_zeroPos.size() != RARMDOF){
            std::cerr << "错误：右臂零位配置维度 " << rArm_zeroPos.size()
                      << " 与 RARMDOF=" << RARMDOF << " 不匹配" << std::endl;
            rArm_zeroPos.assign(RARMDOF, 0.0);
            loaded_ = false;
            return false;
        }

        loaded_ = true;
        std::cout << "===============零位配置加载完毕===============" << std::endl;
        printDefaultPositions();

        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开零位配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析零位配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

//==============================================================================
// 获取零位位置
//==============================================================================

std::vector<double> ZeroPosConfig::getLeftArmZeroPos() const{
    return lArm_zeroPos;
}

std::vector<double> ZeroPosConfig::getRightArmZeroPos() const{
    return rArm_zeroPos;
}

//==============================================================================
// 打印默认位置
//==============================================================================

void ZeroPosConfig::printDefaultPositions() const{
    // 关节名称列表（与 YAML 顺序一致）
    const std::vector<std::string> left_joint_names = {
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint"
    };
    const std::vector<std::string> right_joint_names = {
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint"
    };

    std::cout << "========================================" << std::endl;
    std::cout << "  机械臂默认零位配置 (rad)" << std::endl;
    std::cout << "========================================" << std::endl;

    // 左臂
    std::cout << "--- 左臂 (Left Arm) ---" << std::endl;
    for(size_t i = 0; i < lArm_zeroPos.size() && i < left_joint_names.size(); i++){
        printf("  %-32s  %+8.4f\n",
               left_joint_names[i].c_str(), lArm_zeroPos[i]);
    }

    // 右臂
    std::cout << "--- 右臂 (Right Arm) ---" << std::endl;
    for(size_t i = 0; i < rArm_zeroPos.size() && i < right_joint_names.size(); i++){
        printf("  %-32s  %+8.4f\n",
               right_joint_names[i].c_str(), rArm_zeroPos[i]);
    }

    std::cout << "========================================" << std::endl;
}

//==============================================================================
// 检查是否已成功加载
//==============================================================================

bool ZeroPosConfig::isLoaded() const{
    return loaded_;
}

//将零位角度配置读取到全局变量中
bool loadZeroPosFromYaml(const std::string& yaml_path){
    ZeroPosConfig zeroCfg;
    if(!zeroCfg.loadFromYaml(yaml_path)){
        std::cerr << "警告：零位配置文件加载失败，使用默认零位 0.0" << std::endl;
        return false;
    }

    lArm_zeroPos = zeroCfg.getLeftArmZeroPos();
    rArm_zeroPos = zeroCfg.getRightArmZeroPos();
    lGripper_zeroPos = zeroCfg.getLeftGripperZeroPos();
    rGripper_zeroPos = zeroCfg.getRightGripperZeroPos();

    std::cout << "零位配置已同步到全局控制变量" << std::endl;
    return true;
}

//将关机角度配置读取到全局变量中
bool loadShutdownPosFromYaml(const std::string& yaml_path){
    ZeroPosConfig shutdownCfg;
    if(!shutdownCfg.loadFromYaml(yaml_path)){
        std::cerr << "警告：关机配置文件加载失败，使用默认零位 0.0" << std::endl;
        return false;
    }

    lArm_shutdownPos = shutdownCfg.getLeftArmZeroPos();
    rArm_shutdownPos = shutdownCfg.getRightArmZeroPos();
    lGripper_shutdownPos = shutdownCfg.getLeftGripperZeroPos();
    rGripper_shutdownPos = shutdownCfg.getRightGripperZeroPos();

    std::cout << "关机位置配置已同步到全局控制变量" << std::endl;
    return true;
}

//==============================================================================
//  InitialPDConfig 实现
//==============================================================================

// 构造函数
InitialPDConfig::InitialPDConfig()
    : lArm_Kp(LARMDOF, 0.0)
    , lArm_Kd(LARMDOF, 0.0)
    , rArm_Kp(RARMDOF, 0.0)
    , rArm_Kd(RARMDOF, 0.0)
    , yaml_path_("")
    , rigidity_level_("")
    , loaded_(false)
{
}

InitialPDConfig::InitialPDConfig(const std::string& yaml_path,
                                 const std::string& rigidity_level)
    : lArm_Kp(LARMDOF, 0.0)
    , lArm_Kd(LARMDOF, 0.0)
    , rArm_Kp(RARMDOF, 0.0)
    , rArm_Kd(RARMDOF, 0.0)
    , yaml_path_(yaml_path)
    , rigidity_level_(rigidity_level)
    , loaded_(false)
{
    loadFromYaml(yaml_path, rigidity_level);
}

//==============================================================================
// 从 YAML 文件加载指定刚度等级的 PD 配置
//==============================================================================

bool InitialPDConfig::loadFromYaml(const std::string& yaml_path,
                                   const std::string& rigidity_level){
    yaml_path_ = yaml_path;
    rigidity_level_ = rigidity_level;

    // 构建 YAML 节点名称: left_arm_Kp_<level>, left_arm_Kd_<level>, ...
    std::string left_Kp_key  = "left_arm_Kp_"  + rigidity_level_;
    std::string left_Kd_key  = "left_arm_Kd_"  + rigidity_level_;
    std::string right_Kp_key = "right_arm_Kp_" + rigidity_level_;
    std::string right_Kd_key = "right_arm_Kd_" + rigidity_level_;

    // 关节子键名称列表（与 YAML 顺序一致）
    const std::vector<std::string> left_joint_kp_keys = {
        "left_shoulder_pitch_joint_kp",
        "left_shoulder_roll_joint_kp",
        "left_shoulder_yaw_joint_kp",
        "left_elbow_joint_kp",
        "left_wrist_roll_joint_kp",
        "left_wrist_pitch_joint_kp",
        "left_wrist_yaw_joint_kp"
    };
    const std::vector<std::string> left_joint_kd_keys = {
        "left_shoulder_pitch_joint_kd",
        "left_shoulder_roll_joint_kd",
        "left_shoulder_yaw_joint_kd",
        "left_elbow_joint_kd",
        "left_wrist_roll_joint_kd",
        "left_wrist_pitch_joint_kd",
        "left_wrist_yaw_joint_kd"
    };
    const std::vector<std::string> right_joint_kp_keys = {
        "right_shoulder_pitch_joint_kp",
        "right_shoulder_roll_joint_kp",
        "right_shoulder_yaw_joint_kp",
        "right_elbow_joint_kp",
        "right_wrist_roll_joint_kp",
        "right_wrist_pitch_joint_kp",
        "right_wrist_yaw_joint_kp"
    };
    const std::vector<std::string> right_joint_kd_keys = {
        "right_shoulder_pitch_joint_kd",
        "right_shoulder_roll_joint_kd",
        "right_shoulder_yaw_joint_kd",
        "right_elbow_joint_kd",
        "right_wrist_roll_joint_kd",
        "right_wrist_pitch_joint_kd",
        "right_wrist_yaw_joint_kd"
    };

    try{
        // 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(yaml_path_);

        // ---- 读取左臂 Kp ----
        if(config[left_Kp_key]){
            const YAML::Node& node = config[left_Kp_key];
            lArm_Kp.clear();
            for(size_t i = 0; i < left_joint_kp_keys.size(); i++){
                lArm_Kp.push_back(node[left_joint_kp_keys[i]].as<double>(0.0));
            }
        }else{
            std::cerr << "警告：initialPD.yaml 中未找到 " << left_Kp_key
                      << " 配置节点，使用默认 Kp=0.0" << std::endl;
            lArm_Kp.assign(LARMDOF, 0.0);
        }

        // ---- 读取左臂 Kd ----
        if(config[left_Kd_key]){
            const YAML::Node& node = config[left_Kd_key];
            lArm_Kd.clear();
            for(size_t i = 0; i < left_joint_kd_keys.size(); i++){
                lArm_Kd.push_back(node[left_joint_kd_keys[i]].as<double>(0.0));
            }
        }else{
            std::cerr << "警告：initialPD.yaml 中未找到 " << left_Kd_key
                      << " 配置节点，使用默认 Kd=0.0" << std::endl;
            lArm_Kd.assign(LARMDOF, 0.0);
        }

        // ---- 读取右臂 Kp ----
        if(config[right_Kp_key]){
            const YAML::Node& node = config[right_Kp_key];
            rArm_Kp.clear();
            for(size_t i = 0; i < right_joint_kp_keys.size(); i++){
                rArm_Kp.push_back(node[right_joint_kp_keys[i]].as<double>(0.0));
            }
        }else{
            std::cerr << "警告：initialPD.yaml 中未找到 " << right_Kp_key
                      << " 配置节点，使用默认 Kp=0.0" << std::endl;
            rArm_Kp.assign(RARMDOF, 0.0);
        }

        // ---- 读取右臂 Kd ----
        if(config[right_Kd_key]){
            const YAML::Node& node = config[right_Kd_key];
            rArm_Kd.clear();
            for(size_t i = 0; i < right_joint_kd_keys.size(); i++){
                rArm_Kd.push_back(node[right_joint_kd_keys[i]].as<double>(0.0));
            }
        }else{
            std::cerr << "警告：initialPD.yaml 中未找到 " << right_Kd_key
                      << " 配置节点，使用默认 Kd=0.0" << std::endl;
            rArm_Kd.assign(RARMDOF, 0.0);
        }

        // 检查读取的维度是否正确
        if(lArm_Kp.size() != LARMDOF || lArm_Kd.size() != LARMDOF){
            std::cerr << "错误：左臂 PD 配置维度不匹配 (Kp=" << lArm_Kp.size()
                      << ", Kd=" << lArm_Kd.size()
                      << ")，期望 " << LARMDOF << std::endl;
            lArm_Kp.assign(LARMDOF, 0.0);
            lArm_Kd.assign(LARMDOF, 0.0);
            loaded_ = false;
            return false;
        }
        if(rArm_Kp.size() != RARMDOF || rArm_Kd.size() != RARMDOF){
            std::cerr << "错误：右臂 PD 配置维度不匹配 (Kp=" << rArm_Kp.size()
                      << ", Kd=" << rArm_Kd.size()
                      << ")，期望 " << RARMDOF << std::endl;
            rArm_Kp.assign(RARMDOF, 0.0);
            rArm_Kd.assign(RARMDOF, 0.0);
            loaded_ = false;
            return false;
        }

        loaded_ = true;
        std::cout << "===============PD 配置加载完毕===============" << std::endl;
        std::cout << "  刚度等级: " << rigidity_level_ << std::endl;
        printPDConfig();

        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开 PD 配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析 PD 配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

//==============================================================================
// 获取 PD 参数
//==============================================================================

std::vector<double> InitialPDConfig::getLeftArmKp() const{
    return lArm_Kp;
}

std::vector<double> InitialPDConfig::getLeftArmKd() const{
    return lArm_Kd;
}

std::vector<double> InitialPDConfig::getRightArmKp() const{
    return rArm_Kp;
}

std::vector<double> InitialPDConfig::getRightArmKd() const{
    return rArm_Kd;
}

//==============================================================================
// 获取当前刚度等级
//==============================================================================

std::string InitialPDConfig::getRigidityLevel() const{
    return rigidity_level_;
}

//==============================================================================
// 打印当前 PD 配置
//==============================================================================

void InitialPDConfig::printPDConfig() const{
    // 关节名称列表（与 YAML 顺序一致）
    const std::vector<std::string> left_joint_names = {
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint"
    };
    const std::vector<std::string> right_joint_names = {
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint"
    };

    std::cout << "========================================" << std::endl;
    std::cout << "  机械臂 PD 增益配置  [" << rigidity_level_ << "]"
              << std::endl;
    std::cout << "========================================" << std::endl;

    // 左臂
    std::cout << "--- 左臂 (Left Arm) ---" << std::endl;
    std::cout << "  " << std::left << std::setw(32) << "关节名称"
              << std::right << std::setw(10) << "Kp"
              << std::setw(10) << "Kd" << std::endl;
    std::cout << "  " << std::string(52, '-') << std::endl;
    for(size_t i = 0; i < lArm_Kp.size() && i < left_joint_names.size(); i++){
        printf("  %-32s  %8.0f  %8.0f\n",
               left_joint_names[i].c_str(), lArm_Kp[i], lArm_Kd[i]);
    }

    // 右臂
    std::cout << "--- 右臂 (Right Arm) ---" << std::endl;
    std::cout << "  " << std::left << std::setw(32) << "关节名称"
              << std::right << std::setw(10) << "Kp"
              << std::setw(10) << "Kd" << std::endl;
    std::cout << "  " << std::string(52, '-') << std::endl;
    for(size_t i = 0; i < rArm_Kp.size() && i < right_joint_names.size(); i++){
        printf("  %-32s  %8.0f  %8.0f\n",
               right_joint_names[i].c_str(), rArm_Kp[i], rArm_Kd[i]);
    }

    std::cout << "========================================" << std::endl;
}

//==============================================================================
// 检查是否已成功加载
//==============================================================================

bool InitialPDConfig::isLoaded() const{
    return loaded_;
}

//==============================================================================
// 封装函数 -- 将 PD 配置读取到全局变量中
//==============================================================================

bool loadInitialPDFromYaml(const std::string& yaml_path,
                           const std::string& rigidity_level){
    InitialPDConfig pdCfg;
    if(!pdCfg.loadFromYaml(yaml_path, rigidity_level)){
        std::cerr << "警告：PD 配置文件加载失败 (刚度等级: "
                  << rigidity_level << ")，使用默认 Kp=0, Kd=0" << std::endl;
        return false;
    }

    lArm_defaultKp = pdCfg.getLeftArmKp();
    lArm_defaultKd = pdCfg.getLeftArmKd();
    rArm_defaultKp = pdCfg.getRightArmKp();
    rArm_defaultKd = pdCfg.getRightArmKd();

    std::cout << "PD 配置已同步到全局控制变量 (刚度等级: "
              << rigidity_level << ")" << std::endl;
    return true;
}


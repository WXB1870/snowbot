#include "messageType/upper_msgType.h"

std::map<int, std::string> JntLabelDict = {
    {1, "l_joint_1"},
    {2, "l_joint_2"},
    {3, "l_joint_3"},
    {4, "l_joint_4"},
    {5, "l_joint_5"},
    {6, "l_joint_6"},
    {7, "l_joint_7"},

    {8, "r_joint_1"},
    {9, "r_joint_2"},
    {10, "r_joint_3"},
    {11, "r_joint_4"},
    {12, "r_joint_5"},
    {13, "r_joint_6"},
    {14, "r_joint_7"}
};

std::vector<std::string> JntID_to_label(std::vector<int> JntID){
    std::vector<std::string> mRes;
    mRes.resize(JntID.size());
    for(int i = 0; i < JntID.size(); i++){
        mRes[i] = JntLabelDict.at(JntID[i]);
    }
    return mRes;
}

// 角度转弧度
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 将位置和欧拉角（滚转、俯仰、偏航）转换为位置和四元数
Pose rpyToQuaternion(double x, double y, double z, double roll, double pitch, double yaw) {
    // 将角度转换为弧度（如果输入是度数，需要转换）
    // 如果输入已经是弧度，则直接使用，去掉 deg2rad 调用
    double cr = cos(yaw * 0.5);
    double sr = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(roll * 0.5);
    double sy = sin(roll * 0.5);
    
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return {x, y, z, q};
}
#include "robot_control/msgType.h"


/*  类型转换函数：
------------------
注意：Frame类型转化对应的vector容器应当是六维度向量，其数据含义依次为：
[x, y, z, row, pitch, yaw] -- 三个平移量+三个欧拉角
*/

// 将 std::vector<double> 转换为 KDL::JntArray
KDL::JntArray vec_2_JntArray(const std::vector<double>& mVecInp) {
    KDL::JntArray jntArray;
    if (!mVecInp.empty()) {
        jntArray.resize(mVecInp.size());
        for (size_t i = 0; i < mVecInp.size(); ++i) {
            jntArray(i) = mVecInp[i];
        }
    }
    return jntArray;
}

// 将 KDL::JntArray 转换为 std::vector<double>
std::vector<double> JntArray_2_vec(const KDL::JntArray& mArrInp) {
    std::vector<double> vec;
    if (mArrInp.rows() > 0) {
        vec.resize(mArrInp.rows());
        for (unsigned int i = 0; i < mArrInp.rows(); ++i) {
            vec[i] = mArrInp(i);
        }
    }
    return vec;
}

// 将 std::vector<double> 转换为 KDL::Frame（六维向量：[x, y, z, roll, pitch, yaw]）
KDL::Frame vec_2_Frame(const std::vector<double>& mVecInp) {
    if (mVecInp.size() != 6) {
        throw std::runtime_error("Input vector must have 6 elements for Frame conversion.");
    }

    // 提取平移部分
    KDL::Vector translation(mVecInp[0], mVecInp[1], mVecInp[2]);

    // 提取旋转部分（欧拉角转旋转矩阵）
    KDL::Rotation rotation = KDL::Rotation::RPY(mVecInp[3], mVecInp[4], mVecInp[5]);

    return KDL::Frame(rotation, translation);
}

// 将 KDL::Frame 转换为 std::vector<double>（六维向量：[x, y, z, roll, pitch, yaw]）
std::vector<double> Frame_2_vec(const KDL::Frame& mFrameInp) {
    std::vector<double> vec(6);

    // 提取平移部分
    vec[0] = mFrameInp.p.x();
    vec[1] = mFrameInp.p.y();
    vec[2] = mFrameInp.p.z();

    // 提取旋转部分（旋转矩阵转欧拉角）
    mFrameInp.M.GetRPY(vec[3], vec[4], vec[5]);

    return vec;
}
#ifndef __JntTeatCtl__H__
#define __JntTestCtl__H__

#include <iostream>
#include <vector>
#include <math.h>

#include "snowbot_ctl_msgs/msg/test_msg.hpp"
#include "snowbot_ctl_msgs/msg/jnt_comp_res_visu.hpp"
#include "snowbot_ctl_msgs/msg/jnt_ctl_res_visu.hpp"

#include "rclcpp/rclcpp.hpp"

//电流环跟踪测试
class currentTrackTest{
    private:
        rclcpp::Node* node_ptr_;  // 节点指针（使用原始指针，因为节点由外部管理）

        //电流常数（输入量除以这个数为参考电流）
        std::vector<double> Joint_Kt = {
            1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1
        };

        //-----成员变量-----
        std::vector<double> lArm_desCurrent;
        std::vector<double> rArm_desCurrent;

        std::vector<double> lArm_currentFdback;
        std::vector<double> rArm_currentFdback;

        //调试结果发布参量
        rclcpp::Publisher<snowbot_ctl_msgs::msg::TestMsg>::SharedPtr currentTestMsg;

        //-----功能函数-----

        //消息发布
        void publish();

        //-----补偿变量-----
        std::vector<int> l_retComp = {
            //1, -1, 1, -1, 1, -1, 1
            1, 1, 1, 1, 1, 1, 1
        };

        std::vector<int> r_retComp = {
            //    1, 1, 1, -1, 1, -1, -1
            1, 1, 1, 1, 1, 1, 1
        };

    public:
        //初始化
        currentTrackTest(rclcpp::Node* node);
        
        //绘图函数--PD+前馈模式
        void msgGetAndPlot_PVTmod(
            std::vector<double> lArm_posDes, std::vector<double> lArm_posRet, std::vector<double> lArm_Kp,
            std::vector<double> rArm_posDes, std::vector<double> rArm_posRet, std::vector<double> rArm_Kp,
            
            std::vector<double> lArm_velDes, std::vector<double> lArm_velRet, std::vector<double> lArm_Kd,
            std::vector<double> rArm_velDes, std::vector<double> rArm_velRet, std::vector<double> rArm_Kd,

            std::vector<double> lArm_TorFeedForward, std::vector<double> rArm_TorFeedForward,

            std::vector<double> l_current_fdback, std::vector<double> r_current_fdback
        );
};

//位置、速度跟踪测试
class pvTrackOutp{
    private:
        rclcpp::Node* node_ptr_;  // 节点指针（使用原始指针，因为节点由外部管理）
        
        //上一轮的位置、速度
        std::vector<double> lArm_posDesLst;
        std::vector<double> rArm_posDesLst;

        std::vector<double> lArm_velDesLst;
        std::vector<double> rArm_velDesLst;

        double tempTime = 0.001;

        //消息发送变量
        rclcpp::Publisher<snowbot_ctl_msgs::msg::TestMsg>::SharedPtr pvTrackPub;

        //-----补偿变量-----
        std::vector<int> l_retComp = {
        //    1, -1, 1, -1, 1, -1, 1
            1, 1, 1, 1, 1, 1, 1
        };

        std::vector<int> r_retComp = {
        //    1, 1, 1, -1, 1, -1, -1
            1, 1, 1, 1, 1, 1, 1
        };

    public:

        pvTrackOutp(rclcpp::Node* node);

        void msgGetAndPlot_PVTmod(
            std::vector<double> lArm_posDes, std::vector<double> lArm_posRet,
            std::vector<double> lArm_velDes, std::vector<double> lArm_velRet,

            std::vector<double> rArm_posDes, std::vector<double> rArm_posRet,
            std::vector<double> rArm_velDes, std::vector<double> rArm_velRet
        );

};

//力补偿结果可视化
class TorCompVisu{
    private:
        rclcpp::Node* node_ptr_;  // 节点指针（使用原始指针，因为节点由外部管理）

        //消息发送变量
        rclcpp::Publisher<snowbot_ctl_msgs::msg::TestMsg>::SharedPtr TorCompVisu_pub;

    public:
        
        TorCompVisu(rclcpp::Node* node);

        void msgGetAndPlot_PVTmod(
            std::vector<double> lArm_gravComp, std::vector<double> rArm_gravComp,
            std::vector<double> lArm_coriolisComp, std::vector<double> rArm_coriolisComp,
            std::vector<double> lArm_accTorComp, std::vector<double> rArm_accTorComp
        );
};

//刚度补偿结果可视化
class K_compVisu{
    private:
        rclcpp::Node* node_ptr_;

        //消息发送变量
        rclcpp::Publisher<snowbot_ctl_msgs::msg::TestMsg>::SharedPtr KcompPublisher;

    public:

        K_compVisu(rclcpp::Node* node);

        void msgGetAndPlot_PVTmod(
            std::vector<double> lArm_posComp, std::vector<double> rArm_posComp,
            std::vector<double> lArm_origin_pos, std::vector<double> rArm_origin_pos,
            std::vector<double> lArm_comped_pos, std::vector<double> rArm_comped_pos
        );

        //所有补偿的补偿结果可视化
        void allCompRes_view(
            std::vector<double> l_inp_1, std::vector<double> r_inp_1,
            std::vector<double> l_inp_2, std::vector<double> r_inp_2,
            std::vector<double> l_inp_3, std::vector<double> r_inp_3,
            std::vector<double> l_inp_4, std::vector<double> r_inp_4,
            std::vector<double> l_inp_5, std::vector<double> r_inp_5,
            std::vector<double> l_inp_6, std::vector<double> r_inp_6,
            std::vector<double> l_inp_7, std::vector<double> r_inp_7
        );

};

//摩擦补偿结果可视化
class fricCompRes_visu{
    private:
        rclcpp::Node* node_ptr_;

        //消息发送变量
        rclcpp::Publisher<snowbot_ctl_msgs::msg::TestMsg>::SharedPtr fricCompVisuPublisher;

    public:
        fricCompRes_visu(rclcpp::Node* node);

        void msgGetAndPlot_PVTmod(
            std::vector<double> lArm_vel, std::vector<double> rArm_vel,
            std::vector<double> lArm_tor, std::vector<double> rArm_tor,
            std::vector<double> lArm_fricCompRes, std::vector<double> rArm_fricCompRes,
            std::vector<double> lArm_torOri_feedForward, std::vector<double> rArm_torOri_feedForward,
            std::vector<double> lArm_tor_feedForward, std::vector<double> rArm_tor_feedForward
        );
};

//测试结果综合可视化函数
class Comprehensive_visu{
    private:
        //消息发送变量
        rclcpp::Publisher<snowbot_ctl_msgs::msg::JntCtlResVisu>::SharedPtr Jnt_ctl_res_pub;
        rclcpp::Publisher<snowbot_ctl_msgs::msg::JntCompResVisu>::SharedPtr Jnt_comp_res_pub;

        rclcpp::Node* node_ptr_;

        //辅助变量
        std::vector<double> lArm_velDes_last;   //左臂上一轮目标速度        
        std::vector<double> rArm_velDes_last;   //右臂上一轮目标速度
        double time_last;                       //上一轮时间戳

        double tempTime = 0.001;

    public:
        Comprehensive_visu(rclcpp::Node* node);

        //控制结果发送
        void ctl_res_pub_PVTmod(
            std::vector<double> l_posDes_origin, std::vector<double> r_posDes_origin,
            std::vector<double> l_posDes, std::vector<double> r_posDes,
            std::vector<double> l_velDes_origin, std::vector<double> r_velDes_origin,
            std::vector<double> l_velDes, std::vector<double> r_velDes,
            std::vector<double> l_torFeedforward, std::vector<double> r_torFeedforward,
            std::vector<double> l_Kp, std::vector<double> r_Kp,
            std::vector<double> l_Kd, std::vector<double> r_Kd,

            std::vector<double> l_posRet, std::vector<double> r_posRet,
            std::vector<double> l_velRet, std::vector<double> r_velRet,
            std::vector<double> l_currentRet, std::vector<double> r_currentRet,

            double time_spot
        );

        //补偿结果发送
        void comp_res_pub_PVTmod(
            std::vector<double> l_gravComp, std::vector<double> r_gravComp,
            std::vector<double> l_interiaComp, std::vector<double> r_interiaComp,
            std::vector<double> l_feedForwardComp, std::vector<double> r_feedForwardComp,

            std::vector<double> l_K_comp, std::vector<double> r_K_comp,
            std::vector<double> l_fricComp, std::vector<double> r_fricComp
        );
};

//机器人关节测试封装类 -- 将可视化函数包装
class performanceTestSeal {
private:
    rclcpp::Node* node_ptr_;

public:
    currentTrackTest* current_track;
    pvTrackOutp* pos_vel_track;
    TorCompVisu* torComp_track;
    K_compVisu* k_comp_visu;
    fricCompRes_visu* fric_comp_visu;

    Comprehensive_visu* comprehensive_msg_pub;
    
    performanceTestSeal(rclcpp::Node* node);
    ~performanceTestSeal();
};

//检查目标点的函数
class checkEmptyPoint{
    private:
        std::vector<double> all_errTol = {
            0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008
        };

        int storePointNUm = 10;

        std::vector<double> posLst;
        std::vector<std::vector<double>> posLst_store;

        //过程控制变量：
        bool ifShallInit = true;
        int tempTime = 0;

    public:
        void Init(){
            posLst.assign(7, 0.0);
            tempTime = 0;
        }
        void check_point(std::vector<double> posInp, int pointID,
            double realTime, double pointGetTime

        ){
        
            double posInp_check = posInp[pointID - 1];
            double errTol = all_errTol[pointID - 1];

            if(this->ifShallInit){
                Init();
                this->ifShallInit = false;
            }

            if(posInp_check - posLst[pointID - 1] > errTol){
                //执行打印函数
                std::cout << "=========================================================" << std::endl;
                std::cout << "第" << tempTime << "次异常输出" << std::endl;
                std::cout << "此时时刻：" << realTime << std::endl;
                std::cout << "获取上位机数据点的时刻：" << pointGetTime << std::endl;
                std::cout << "插补点输出位置信息：" << posInp[pointID - 1] << std::endl;

                std::cout << "=========================================================" << std::endl;
            }

            posLst = posInp;
            posLst_store.push_back(posInp);
            if(posLst_store.size() > storePointNUm){
                posLst_store.erase(posLst_store.begin());
            }

            tempTime += 1;
        }
};

#endif
#ifndef __ForceComp__H__
#define __ForceComp__H__

#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream>
#include <stdlib.h>

#include "JntController/runinController.h"

/*      by 霜云喵
------------------------
基于模糊控制的补偿器
------------------------

*/

//----------------------------------------------------------------------------
//简易模糊控制
/*  模糊依据 -- 速度偏差量，加速度
    模糊核心 -- 速度偏差量越大，加速度越大，阻尼

    补偿参数 -- 
*/
extern std::vector<std::vector<double>> Jnt_velErr_region;
extern std::vector<std::vector<double>> Jnt_acc_region;
class FuzzyKpComp_simp{
    private:
        //模糊区间划分
        std::vector<double> velErr_region;
        std::vector<double> acc_region;

        //模糊矩阵 -- 
        std::vector<std::vector<double>> fuzzy_matrix;

        //补偿参数输出：
        std::vector<double> KdCompInte;

    public:
        //依据关节序号选择全局变量中第几行
        //输入量：关节在机械臂上的序号
        FuzzyKpComp_simp(int JntID);

        //模糊补偿函数
        double KdComp(double JntVelErr, double JntAcc);

};

//----------------------------------------------------------------------------
//关节层面补偿：

class kalman_filter;

//简易刚度补偿
class K_comp{
    private:
        //关节扭矩系数
        std::vector<double> JointKt = {
            2.35, 2.8, 2.8, 2.8, 1.4, 1.4, 1.4
        };
        std::vector<double> Joint_K = {
            177.7, 110, 124, 124, 210, 210, 210
        };
        std::vector<double> cor = {
            0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
        };

        //----------滤波器----------
        std::vector<kalman_filter> dKalmanFilter;
        std::vector<double> all_Qgain = {
            12000, 12000, 12000, 12000, 12000, 12000, 12000
        };
        std::vector<double> all_Rgain = {
            200, 200, 200, 200, 200, 200, 200
        };
        bool ifStart = true;
        bool ifUseFtr = true;

    public:

        /*  输出函数
        */
        double posComp(double IqInp, int JointID);
};

//基于Stribeck的静摩擦补偿
class Stribeck_Fric
{
    private:
        //参数定义：
        //摩擦参数：
        float fs;       //静摩擦系数
        float fc;       //库伦摩擦系数
        float fstic;    //粘滞摩擦系数

        float vs;       //临界速度
        float gamma;    //Stribeck系数（取0.5~2.0）

        float mTo;      //空载摩擦损耗

        //过程控制量：
        int last_velDirection = 0;
    public:
        void StribeckModelInit(float mfs, float mfc, float mfstic, float vs, float gama, float T0); //初始化函数
        float StribeckFricCal(float Tinp, float vel, float posInp);   //计算Stribeck摩擦力的函数
};
class Stribeck_FricCompWrap{
    private:

        //摩擦参数
        std::vector<double> lArm_fs = {
            0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02
        };
        std::vector<double> rArm_fs = {
            0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02
        };

        std::vector<double> lArm_fc = {
            0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01
        };
        std::vector<double> rArm_fc = {
            0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01
        };
        std::vector<double> lArm_fstic = {
            0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001
        };
        std::vector<double> rArm_fstic = {
            0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001
        };

        std::vector<double> lArm_vs = {
            0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02
        };
        std::vector<double> rArm_vs = {
            0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02
        };
        std::vector<double> gamma = {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
        };

        std::vector<double> lArm_mT0 = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };
        std::vector<double> rArm_mT0 = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };

        //输入值滤波
        std::vector<double> all_Qgain_tor = {12000, 12000, 12000, 12000, 12000, 12000, 12000};
        std::vector<double> all_Rgain_tor = {300, 300, 300, 300, 300, 300, 300};
        std::vector<double> all_Qgain_vel = {12000, 12000, 12000, 12000, 12000, 12000, 12000};
        std::vector<double> all_Rgain_vel = {300, 300, 300, 300, 300, 300, 300};

        std::vector<kalman_filter> dKalmanFtr_TorInp_l;
        std::vector<kalman_filter> dKalmanFtr_TorInp_r;
        std::vector<kalman_filter> dKalmanFtr_velInp_l;
        std::vector<kalman_filter> dKalmanFtr_velInp_r;

        //静摩擦补偿
        std::vector<Stribeck_Fric> lArm_fricComp;
        std::vector<Stribeck_Fric> rArm_fricComp;

        //过程控制量
        bool ifFirst = true;
        bool ifUseInpFtr = true;

    public:

        //摩擦补偿
        std::vector<std::vector<double>> fricCompRes(
            std::vector<double> lArm_tor, std::vector<double> rArm_tor,
            std::vector<double> lArm_vel, std::vector<double> rArm_vel,
            std::vector<double> lArm_pos, std::vector<double> rArm_pos
        );
};

//积分补偿
class KiComp{
    private:
        std::vector<double> l_Ki = {
            3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0
        };

        std::vector<double> r_Ki = {
            3.0, 3.0, 10.0, 3.0, 3.0, 3.0, 3.0
        };

        std::vector<double> l_KiComp_max = {
            0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8
        };
        std::vector<double> r_KiComp_max = {
            0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8
        };

        std::vector<double> l_KiComp_min = {
            -0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8
        };
        std::vector<double> r_KiComp_min = {
            -0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8
        };

        std::vector<double> l_KiComp = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };
        std::vector<double> r_KiComp = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };

        double temp_T = 0.001;

    public:
        std::vector<std::vector<double>> ki_comp_outp(std::vector<double> l_posDes, std::vector<double> l_posRet, std::vector<double> r_posDes, std::vector<double> r_posRet){

            for(int i = 0; i < SINGLEARMJNTNUM; i++){
                l_KiComp[i] += (l_posDes[i] - l_posRet[i]) * temp_T;
                r_KiComp[i] += (r_posDes[i] - r_posRet[i]) * temp_T;

                if(l_KiComp[i] > l_KiComp_max[i]){
                    l_KiComp[i] = l_KiComp_max[i];
                }
                if(l_KiComp[i] < l_KiComp_min[i]){
                    l_KiComp[i] = l_KiComp_min[i];
                }

                if(r_KiComp[i] > r_KiComp_max[i]){
                    r_KiComp[i] = r_KiComp_max[i];
                }
                if(r_KiComp[i] < r_KiComp_min[i]){
                    r_KiComp[i] = r_KiComp_min[i];
                }
            }

            std::vector<std::vector<double>> mRes = {
                l_KiComp, r_KiComp
            };

            return mRes;
        }
};

//插补输出值过滤
class runinOutpFtr{
    private:
        std::vector<kalman_filter> dKalmanFilter_l;
        std::vector<kalman_filter> dKalmanFilter_r;

        //过程控制量
        bool ifFirst = true;

    public:
        std::vector<double> all_Qgain = {
            12000, 12000, 12000, 12000, 12000, 12000, 12000
        };
        std::vector<double> all_Rgain = {
            300, 300, 300, 300, 300, 300, 300
        };

        std::vector<std::vector<double>> Outp_ftr(std::vector<double> lArm_inp, std::vector<double> rArm_inp);
};


//----------------------------------------------------------------------------
//滤波器类

//一阶低通滤波器
class lowpassFilter_1d{

};

//卡尔曼滤波器
class kalman_filter{
    private:
        double outvar[2];   //卡尔曼滤波器输出
        double Pmtrx[2][2]; //状态估计误差协方差矩阵
        double Qgain;   //噪声增益（用于调整过程噪声协方差矩阵Q的影响，该项越大滤波越强）
        double Rgain;   //测量噪声增益（用于调整测量噪声协方差矩阵的影响）
        
        //初始化状态估计协方差矩阵
        double P0mtrx[2][2]={{1000000.0f, 0.0f}, {0.0f, 1000000.0f}};
        //状态转移矩阵及其转置
        double FAImtrx[2][2] = {{1.0, 0.001}, {0.0, 1.0}};
        double FAImtrxT[2][2] = {{1.0,0.0}, {0.001, 1.0}};
        //过程噪声协方差矩阵
        double Qmtrx[2][2] = {{0.001*0.001*0.001/3.0f, 0.001*0.001*0.5f}, {0.001*0.001*0.5f, 0.001}};

        //中间变量
        double midx[2];  //预测的状态值
        double tempMtrx[2][2]; //临时矩阵
        double FAIPFAIT[2][2]; // 预测的状态估计误差协方差矩阵
        double Pk_[2][2]; // 更新后的状态估计误差协方差矩阵
        double Kgain[2]; // 卡尔曼增益

        //---------------成员函数---------------
        //自定义矩阵乘法
        void mMatrixMul(double a[],double b[],int m,int n,int k,double c[]);

    public:
        void FilterInit(double inputVar, double mQgain, double mRgain);     //初始化函数
        double FilteredResCal(double inputvar);      //滤波结果计算函数

};



#endif
#ifndef __runinController__H__
#define __runinController__H__

#include<iostream>
#include<vector>
#include<math.h>
#include <deque>
#include <algorithm>
#include <numeric>

/*
        by 霜云喵
================================
插补函数
用于将上位机输入值插补
============修改日志============

==============协议==============

==============备注==============
**牛顿插补测试代码位置：/home/lhy/codes/Cana/debug01**

================================
*/

//定义关节单轮循环时间
#define JNTtempTime (0.001)
#define SINGLEARMJNTNUM (7)

//三次B——样条插补函数
class lowpassFtr{
    private:
        double lastOutp = 0;
        double k = 0.01;
    public:
        double outp(double mInp){
            double mRes = k*mInp + (1-k) * lastOutp;
            lastOutp = mRes;

            return mRes;
        }
};

class BSplineFitter{
    private:
        //控制点：
        std::vector<double> controlPoints;
        //节点向量
        std::vector<double> knotVector;

        //过程控制量：
        int degree = 3;     //三次样条插值
        int controlPoint_len = 4;   //控制点数组长度为4
        double yValue_last = 0.0;
        double base_tempTime = 0.001;

        //计算三次均匀B——样条基函数
        void calculateBasisFunctions(double u, 
                               double& N0, double& N1, 
                               double& N2, double& N3) const;

        //构建均匀节点向量
        void buildUniformKnotVector();

        //计算有效参数范围
        std::pair<double, double> getValidParameterRange() const;

        //使用节点向量找到参数u所在的段
        int findSpan(double u) const;
    
    public:

        // 一维点组的最后一段精细采样
        std::vector<double> sampleBSpline1D(double newPoint, int numSamples);

        // 清除所有数据
        void clear(){
            controlPoints.clear();
            knotVector.clear();
            yValue_last = 0.0;
        }
};

class CubicSpline{
    private:
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> a, b, c, d;

        // 计算样条系数
        void calculateSplineCoefficients();

        // 查找x_val所在的区间索引
        size_t findSegment(double x_val);

        //传入数据点，计算样条插值
        void SplineCal();

        //计算样条插值在指定点的值
        double interpolate(double x_val);

        bool ifFirstTemp = true;

        //假设每个区间长度为1
        double x_temp = 0;
        double der_x_temp = 1;
        int temp_num = 0;       //区间迭代计数器

    public:
        int windowLen = 50;

        //接口封装：计算x_i到x_{i+1}的拟合结果
        std::vector<double> cubicSplineCal(double y_lst, double y_des, int windowLen);
};

/*
    插补核心 -- 封装两类插补函数：线性插补与二次插补
------------------------------------------------
    封装两类函数：直线插补，B-样条曲线拟合法
    插补模式：
        1 -- 直线拟合
        2 -- 样条曲线拟合
*/

class lowpassFilter
{
    //传统的一阶低通滤波器
    private:
        double alpha;       // 滤波系数
        double lastOutput;  // 上一次输出值
        double sampleRate; // 采样频率（Hz）
        double timeConstant; // 时间常数（τ）(通过调节时间常数调节截止频率)
    public:

        void init(double sampleRate, double cutoffFreq);    //初始化函数
        double filter(double input);            //滤波处理函数
        void reset(double initValue = 0.0){     //重置函数
            this->lastOutput = initValue;
        }

};

//滑动平均滤波器
class smoother{
    private:
        std::vector<double> history_data;
        std::vector<double> weight = {
            0.2, 0.2, 0.2, 0.2, 0.2
        };

        double threshold = 0.1; // 异常检测阈值，可根据实际情况调整

    public:
        double output(double input){
        int window_len = weight.size();
        
        // 当有足够的历史数据时，进行异常检测
        if(history_data.size() >= window_len) {
            // 计算历史数据的均值
            double sum = 0.0;
            for(int i = 0; i < window_len; i++){
                sum += history_data[history_data.size() - window_len + i];
            }
            double mean = sum / window_len;
            
            // 检测输入数据是否偏离过大
            if(fabs(input - mean) > threshold) {
                // 使用历史数据的末位代替输入数据
                input = history_data.back();
            }
        }
        
        // 进行正常的滑动平均滤波
        if(history_data.size() >= window_len){
            history_data.push_back(input);
            history_data.erase(history_data.begin());

            double res = 0;
            for(int i = 0; i < window_len; i++){
                res += history_data[i] * weight[i];
            }
            return input;
        }else{
            history_data.push_back(input);
            return input;
        }
    }
};

class runin_core{
    private:
        double tempTime = JNTtempTime;

        CubicSpline m_cubicSpline;

        BSplineFitter m_splineFitter;
        //适配于B样条曲线插补的容器代码
        std::vector<double> mInp;
        bool ifFirstTemp = true;

        int winLen = 4;

        std::vector<lowpassFtr> m_ftr;
        bool ifInit = true;

    public:

        /*  直线插补函数：
        ------------------
        输入量：
            last_Inp -- 起始位置
            des_Inp -- 本轮插补目标位置
            pointNum -- 目标插补轨迹输出点数量
        */
        std::vector<double> liner_runin(double last_Inp, double des_Inp, int pointNum);

        //样条曲线插补函数
        std::vector<double> Spline_fit(double last_Inp, double des_Inp, int pointNum);

        //插补封装函数：
        std::vector<double> runin_seal(double last_Inp, double des_Inp, int pointNum, int runinMod);

};

//滤波器类
class WeightedMovingFtr{
    private:
        std::deque<double> data_buffer_;  // 数据缓冲区
        std::vector<double> weights_;     // 权重系数
        size_t window_size_;              // 窗口大小
        double sum_weights_;              // 权重总和（用于归一化）

        //计算加权平均值
        double calculateWeightedAverage();

    public:

        //构造函数
        WeightedMovingFtr(size_t window_size, const std::vector<double>& weights);

        //添加数据并返回滤波值
        double filter(double new_value);

        //-----------------------------------
        //辅助函数
        // 获取当前滤波器的状态（是否已填满窗口）
        bool isReady() const {
            return data_buffer_.size() == window_size_;
        }

        // 重置滤波器
        void reset() {
            data_buffer_.clear();
        }

        // 获取当前缓冲区大小
        size_t getCurrentSize() const {
            return data_buffer_.size();
        }

        // 获取窗口大小
        size_t getWindowSize() const {
            return window_size_;
        }
};


/*  插补控制类：
-------------------
包含五类控制量的插补：

    目标位置
    目标速度

    目标重力补偿力矩
    目标惯性力综合补偿力矩
    目标前馈力矩
插补过程中，注意目标位置与目标速度的协调

*/

//类定义——单轮插补数据输出
typedef struct{
    //每一轮插补输出的点数据集

    std::vector<double> lArm_posDot;
    std::vector<double> lArm_velDot;

    std::vector<double> lArm_gravCompDot;
    std::vector<double> lArm_InteriaCompDot;
    std::vector<double> lArm_FeedForwardCompDot;

    std::vector<double> rArm_posDot;
    std::vector<double> rArm_velDot;

    std::vector<double> rArm_gravCompDot;
    std::vector<double> rArm_InteriaCompDot;
    std::vector<double> rArm_FeedForwardCompDot;

}singleTempRuninData;

//类定义——插补输出轨迹包装类
typedef struct{
    std::vector<singleTempRuninData> runinDataTRaj;     //插补点轨迹
    int TrajLen;        //轨迹长度

    std::vector<double> lArmJnt_Kp;     //左臂关节Kp
    std::vector<double> lArmJnt_Kd;     //左臂关节Kd

    std::vector<double> rArmJnt_Kp;     //右臂关节Kp
    std::vector<double> rArmJnt_Kd;     //右臂关节Kd

    //插补信息(目标位置)：
    std::vector<double> lArm_tempDesPos;
    std::vector<double> rArm_tempDesPos;

}runinTrajWrap;

//类定义——插补数据输出类：
typedef struct{
    singleTempRuninData runinData;

    std::vector<double> lArmJnt_Kp;
    std::vector<double> lArmJnt_Kd;
    std::vector<double> rArmJnt_Kp;
    std::vector<double> rArmJnt_Kd;

}runinTrajOutp;

/*  插补控制器
----------------
    注意，插补控制器不会采用反馈值作为插补输入量，插补控制器就是针对目标位置研究的插补
*/
class runin_control{
    private:

        //插补求解核心
        std::vector<runin_core> posRunin_core_l;
        std::vector<runin_core> velRunin_core_l;
        std::vector<runin_core> gravCompRunin_core_l;
        std::vector<runin_core> InteriaCompRunin_core_l;
        std::vector<runin_core> FeedForwardRunin_core_l;

        std::vector<runin_core> posRunin_core_r;
        std::vector<runin_core> velRunin_core_r;
        std::vector<runin_core> gravCompRunin_core_r;
        std::vector<runin_core> InteriaCompRunin_core_r;
        std::vector<runin_core> FeedForwardRunin_core_r;

        //插补求解轨迹储存及其对应控制参数
        double tempTime = JNTtempTime;
        int singleWrapLen = SINGLEARMJNTNUM;
        int trajLen;        //插补轨迹长度
        bool ifFirstRunin;  //判断量 -- 是否为第一轮插补
        int loopTime = 0;       //控制量 -- 输出循环次数

        //滤波器
        std::vector<lowpassFilter> posFilter_l;
        std::vector<lowpassFilter> velFilter_l;

        std::vector<lowpassFilter> posFilter_r;
        std::vector<lowpassFilter> velFilter_r;

        float sampleRate = 1000;
        float cutoffFreq = 5;

        bool ifUseFtr = true;

        //上一轮目标插补信息
        singleTempRuninData lastStRuninDotWrap;

        //插补结果存储
        runinTrajWrap m_runin_traj_wrap;

        bool ifUseOutpFtr = false;   //是否使用滤波器
        std::vector<double> posFtr_weight = {
            1.0, 1.0, 1.0, 1.0, 1.0
        };
        std::vector<double> velFtr_weight = {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
        };
        WeightedMovingFtr posOutpWtr = WeightedMovingFtr(
            posFtr_weight.size(), posFtr_weight
        );
        WeightedMovingFtr velOutpWtr = WeightedMovingFtr(
            velFtr_weight.size(), velFtr_weight
        );
        std::vector<WeightedMovingFtr> allJntPosFilter_l;
        std::vector<WeightedMovingFtr> allJntVelFilter_l;
        std::vector<WeightedMovingFtr> allJntPosFilter_r;
        std::vector<WeightedMovingFtr> allJntVelFilter_r;

        //输出限幅相关参数
        bool ifFirstTime = true;
        runinTrajOutp last_outp;
        std::vector<double> derPos_upperLim = {
            0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
        };
        std::vector<double> derPos_opUpperLim = {
            -0.5, -0.05, -0.5, -0.5, -0.5, -0.5, -0.5
        };
        std::vector<double> derVel_upperLim = {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
        };
        std::vector<double> derVel_opUpperLim = {
            -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0
        };

    public:
        /*  初始化函数
        ------------------
        每次刷新插补求解器时，调用此函数即可
        函数输入值为左右臂第一轮输入位置，依据实际需求选取合适值即可
        速度、力矩输入值为0

        ------------------
        输入量：
            左右臂初始位置
            middle_tempTime -- 中层求解器插补频率
        */
        void init(
            std::vector<double> lArmPos_FirstTempInp, std::vector<double> rArmPos_FirstTempInp,
            int m_trajLen
        );

        /*  插补函数
        ---------------
            注意无论是哪种插补，首轮插补均使用线性插补，当不是首轮了才使用二次插补

            runinMod -- 插补模式
        */
        //插补计算函数
        void ModelRunin(
            std::vector<double> posDes_l, std::vector<double> velDes_l, 
            std::vector<double> gravComp_l, std::vector<double> InteriaComp_l, std::vector<double> FeedForward_l,
            std::vector<double> posDes_r, std::vector<double> velDes_r,
            std::vector<double> gravComp_r, std::vector<double> InteriaComp_r, std::vector<double> FeedForward_r,
            std::vector<double> Kp_l, std::vector<double> Kd_l,
            std::vector<double> Kp_r, std::vector<double> Kd_r,
            int runinMod,
            int m_trajLen
        );
        //插补结果输出函数
        runinTrajOutp dataOutp(bool topic_stream = false);

        //辅助变量：

};






#endif

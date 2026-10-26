#include"controller/JntController.h"
#include <stdlib.h>
#include <string.h>
//#include <yaml-cpp/yaml.h>
//#include <control_toolbox/filters.h>
#include <fstream>
#include <cmath>
#include <time.h>
#include <sys/time.h>

namespace Kernel
{
   //========================================== 

    //====================SingleJointController====================
    float sgn(float x)  //符号函数sgn(x)
    {
        if(std::fabs(x - 0) < 1e-3){
            return 0;
        }
        else{
            if(x < 0){
                return -1;
            }
            else{
                return 1;
            }
        }
    }
    void mMatrixMul(float a[],float b[],int m,int n,int k,float c[])
    { int i,j,l,u;
        for (i=0; i<=m-1; i++){
            for (j=0; j<=k-1; j++)
            { u=i*k+j; c[u]=0.0;
                for (l=0; l<=n-1; l++)
                c[u]=c[u]+a[i*n+l]*b[l*k+j];
            }
        }
        return;
    }

  //--------------------PIDcontrolBar--------------------

    //输入关键参量函数
    void PIDcontrolBar::CotrollerInit(float Kp, float Ki, float Kd, float KbInteigar, float mT)
    {
        //输入量：Kp; Ki; Kd; Kb系数; 单轮控制时间
        //该函数在初始化阶段调用
        this->mKp = Kp;
        this->mKi = Ki;
        this->mKd = Kd;
        this->mKb = KbInteigar*(Ki/Kp);

        this->IntegerErrAccu = 0;
        this->preErr = 0;
        this->dertaT = mT;
        this->temp = 0;
        this->preKbRet = 0;
    }

    //计算PID开环输出量的函数
    float PIDcontrolBar::ControlResultCal(float merr)
    {
        //输入量：第i轮迭代的偏差

        //Kp项
        float resKp = this->mKp * merr;
        //Ki项
        float resKi = this->IntegerErrAccu * this->mKi - this->preKbRet;
        //Kd项
        float resKd = this->mKd*(merr - this->preErr);
        float res = resKp + resKi + resKd;

        //迭代计算
        this->preKbRet = this->mKb * res;
        this->preErr = merr;
        this->IntegerErrAccu += merr * this->dertaT;

        //输出结果
        return res;
    }

    // PID controller update

    void PIDcontrolBar::FeedForwardInit(float Kf, float JMotor, float KtMotor)
    {
        this->preDesireNum = 0;
        this->preDesireNumDt = 0;
        this->mKf = Kf;
        this->mJMotor = JMotor;
        this->mKtMotor = KtMotor;
    }

    float PIDcontrolBar::ControlResultCalWithFeedForward(float errNum, float desireNum)
    {
        //输入量：第i轮迭代的偏差

        //Kp项
        float resKp = this->mKp * errNum;
        //Ki项
        float resKi = this->IntegerErrAccu * this->mKi - this->preKbRet;
        //Kd项
        float resKd = this->mKd*(errNum - this->preErr);
        float res = resKp + resKi + resKd;

        //迭代计算
        this->preKbRet = this->mKb * res;
        this->preErr = errNum;
        this->IntegerErrAccu += errNum * this->dertaT;

        //feed forward link
        float velNum = (desireNum - preDesireNum) / dertaT;
        float accNum = (velNum - preDesireNumDt) / dertaT;
        float ran = 0.0f;
        if(abs(velNum - 0) < 0.05){ran = 1.0f;}
        float resFeedForward = mJMotor * accNum / mKtMotor;
        //float resFeedForward = -(mJMotor * (errNum - preErr))/ mKtMotor;
        preDesireNum = desireNum;
        preDesireNumDt = velNum;

        //debug
        std::cout<<" fdfw= "<<resFeedForward<<std::endl;

        float mres = res + resFeedForward;
        return mres;
    }

    //--------------------filter--------------------

    void Filter::FilterInit(float inputVar, float mQgain, float mRgain)
    {
        //memcpy(Pmtrx, P0mtrx, sizeof(Pmtrx));

        //****** debug
        for(int i=0;i<2;i++){
            for(int j=0;j<2;j++)
            {
        
                Pmtrx[i][j]=P0mtrx[i][j];
            }
        }

        *outvar = inputVar;
        *(outvar+1) = 0;
        this->Qgain = mQgain;
        this->Rgain = mRgain;
        
    }

    float Filter::FilteredResCaloldVersion(float inputvar)
    {
        //预测阶段
        mMatrixMul(FAImtrx[0], outvar, 2, 2, 1, midx);

        //更新状态估计协方差矩阵
        mMatrixMul(FAImtrx[0], Pmtrx[0], 2, 2, 2, tempMtrx[0]);
        mMatrixMul(tempMtrx[0], FAImtrxT[0], 2, 2, 2, FAIPFAIT[0]);
        
        //更新阶段：
        for(int i=0; i<2; i++){
            for(int j=0; j<2; j++){
                Pk_[i][j] = FAIPFAIT[i][j] + Qgain*Qgain*Qmtrx[i][j];
            }
            Kgain[i] = Pk_[i][0]/(Pk_[0][0] + Rgain*Rgain);
            *(outvar+i) = midx[i] + Kgain[i]*(inputvar - midx[0]);
        }
        tempMtrx[0][0] = 1.0f-Kgain[0];
        tempMtrx[0][1] = 0.0f;
        tempMtrx[1][0] = -Kgain[1];
        tempMtrx[1][1] = 1.0f;
        mMatrixMul(tempMtrx[0],Pk_[0],2,2,2,FAIPFAIT[0]);
        for(int i=0;i<2;i++){
            for(int j=0;j<2;j++){
                Pmtrx[i][j] = FAIPFAIT[i][j];
            }
        }

        //返回结果
        float res = outvar[0];
        return res;
    }


    float Filter::FilteredResCal(float inputvar)
    {
        // 预测阶段
        mMatrixMul(FAImtrx[0], outvar, 2, 2, 1, midx); // 这里假设MatrixMul能正确处理这些尺寸
        mMatrixMul(FAImtrx[0], Pmtrx[0], 2, 2, 2, FAIPFAIT[0]); // 预测协方差矩阵
        // 加上过程噪声
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                FAIPFAIT[i][j] += Qgain * Qgain * Qmtrx[i][j];
            }
        }
        // 计算卡尔曼增益
        float K = FAIPFAIT[0][0] / (FAIPFAIT[0][0] + Rgain * Rgain);

        // 更新状态
        outvar[0] = midx[0] + K * (inputvar - midx[0]); // 对于一维状态，只更新第一个元素

        // 更新协方差矩阵（对于一维状态，只更新相关的元素）
        float I_minus_KH[2][2] = {{1.0f - K, 0.0f}, {-K, 1.0f}}; // 实际上，对于一维情况，这简化为[1-K, 0; -K, 1]（但保留2x2形式以备将来扩展）
        mMatrixMul(I_minus_KH[0], FAIPFAIT[0], 2, 2, 2, Pmtrx[0]); // 更新后的协方差矩阵

        // 返回结果
        return outvar[0];
    }

    //--------------------Low pass filter--------------------

    void lowpassFilter::init(double msampleRate, double cutoffFreq){
        //初始化函数
        this->sampleRate = msampleRate;
        this->timeConstant = 1.0 / (2*M_PI*cutoffFreq);
        this->lastOutput = 0;
        this->alpha = 1.0 - std::exp(-1.0 / (timeConstant * sampleRate));
    }

    double lowpassFilter::filter(double input)
    {
        // 一阶低通滤波公式：y(n) = α*y(n-1) + (1-α)*x(n)
        lastOutput = alpha * lastOutput + (1 - alpha) * input;
        return lastOutput;
    }

    //--------------------Stribeck Friction Model--------------------

    void StribeckFriction::StribeckModelInit(float mfs, float mfc, float mfstic, float mvs, float mgamma, float T0)
    {
        this->fs = mfs;
        this->fc = mfc;
        this->fstic = mfstic;
        this->vs = mvs;
        this->gamma = mgamma;
        this->mTo = T0;
    }
    float StribeckFriction::StribeckFricCal(float Tinp, float vel)
    {
        float mres = (this->fc*Tinp + ((this->fs*Tinp + this->mTo) - this->fc*Tinp)*std::exp(-1*std::fabs(vel/this->vs)*this->gamma))*sgn(vel) + vel*this->fstic;
        
        
        return mres;
    }


    //--------------------Impendence control--------------------

    void ImpendanceCtl::InitFunc(float B, float K, bool ifUseM, float T, float start_Theta, float M)
    {
        this->M_ImpCtl = M;
        this->B_ImpCtl = B;
        this->K_ImpCtl = K;
        this->IfUseM_Inp = ifUseM;
        this->dertaT = T;

        //偏移量初始化逻辑：先令理想目标位置等于理想实际位置, 负载为0
        //注意初始化后要紧跟着执行一段阻抗控制
        this->theta_0 = start_Theta;
        this->theta = start_Theta;
        this->derta_theta = 0;
        this->Tor = 0;

        //其他控制量初始化
        this->dv0 = 0;
        this->dx0 = 0;
    }

    void ImpendanceCtl::updateInte(float B, float K, float M)
    {
        this->M_ImpCtl = M;
        this->B_ImpCtl = B;
        this->K_ImpCtl = K;
    }

    bool ifAbleToOutp(float oldVar, float newVar, float derta, int temp_i, int maxTimes){
        
        if(std::abs(newVar - oldVar) <= derta){
            return true;
        }else{
            if(temp_i <= maxTimes){
                return false;
            }else{
                std::cout<< "**Warning: Failed to find a acceptable solution during the temps**" <<std::endl;
                return true;
            }
        }
    }
    void ImpendanceCtl::CalDertaByTor(float TorInp)
    {
        float old_x;
        float x = this->dx0;
        float v = this->dv0;
        float t = h;
        bool ifOutp;

        //采用四阶龙格-库塔法计算
        if(this->IfUseM_Inp)
        {
            for(int i = 0; i < this->steps; i++)
            {
                dervatives_Full(x, v, k1_x, k1_v, TorInp);
                dervatives_Full(x + h*(k1_x/2), v + h*(k1_v/2), k2_x, k2_v, TorInp);
                dervatives_Full(x + h*(k2_x/2), v + h*(k2_v/2), k3_x, k3_v, TorInp);
                dervatives_Full(x + h*k3_x, v + h*k3_v, k4_x, k4_v, TorInp);

                old_x = x;
                x = x + (h/6)*(k1_x + 2*k2_x + 2*k3_x + k4_x);
                v = v + (h/6)*(k1_v + 2*k2_v + 2*k3_v + k4_v);
                t += h;

                ifOutp = ifAbleToOutp(old_x, x, this->smallest_dertaRate*x, i, this->steps);
                if(ifOutp){
                    this->derta_theta = x;
                    break;
                }
            }
        }else{
            for(int i = 0; i <this->steps; i++)
            {
                dervatives_Part(x, k1_x, TorInp);
                dervatives_Part(x + h*(k1_x/2), k2_x, TorInp);
                dervatives_Part(x + h*(k2_x/2), k3_x, TorInp);
                dervatives_Part(x + h*k3_x, k4_x, TorInp);

                x = x + (h/6)*(k1_x + 2*k2_x + 2*k3_x + k4_x);
                t += h;

                ifOutp = ifAbleToOutp(old_x, x, this->smallest_dertaRate*x, i, this->steps);
                if(ifOutp){
                    this->derta_theta = x;
                    break;
                }
            }
        }
        this->derta_theta = x;
    }

    void ImpendanceCtl::CalTorByDerta(float derta_x, float derta_v, float derta_a)
    {
        float F = 0;
        if(this->IfUseM_Inp){
            F = this->M_ImpCtl*derta_a + this->B_ImpCtl*derta_v + this->K_ImpCtl*derta_x;
        }else{
            F = this->B_ImpCtl*derta_v + this->K_ImpCtl*derta_x;
        }
        this->Tor = F;
    }

    //--------------------Force Controller--------------------
    float ForceControl::ForceCtlOutp(float Inp)
    {
        float InpFiltered = Inp;
        if(this->ifNeedFilter == 1){
            InpFiltered = mForceInpFilter.FilteredResCal(Inp);
        }
        float outp = mForceCtl.ControlResultCal(InpFiltered);
        return outp;
    }

    //==========================================

    //--------------------力——阻抗控制器--------------------

    //初始化函数
    void JntCtl_TorImp::init(float B, float K, bool ifUseM, float Start_Theta,
                             float torKp, float torKi, float torKd, float torKbInte,
                             float mKt, float mKtInte, int mTorCompMod,
                             float mposLim[], float mvelLim[], float mOutpLim[],
                             float M
                             )
    {
        mForceControl.CotrollerInit(torKp, torKi, torKd, torKbInte, this->mT);
        mImpCtl.InitFunc(B, K, ifUseM, this->mT, Start_Theta, M);

        this->Kt = mKt;
        this->Kt_inte = mKtInte;
        this->TorCompMod = mTorCompMod;

        posLim[0] = mposLim[0];     posLim[1] = mposLim[1];
        velLim[0] = mvelLim[0];     velLim[1] = mvelLim[1];
        OutpLim[0] = mOutpLim[0];   OutpLim[1] = mOutpLim[1];

        this->velDesLst = 0;
    }

    float JntCtl_TorImp::ctlOutp(float posDes, float posRet, float velDes, float IqRet, float OtherTorComp)
    {
        float mOutp;

        if(ifStCtl){
            float TorDes;
            float dertaPos, accInp;
            //软限位
            if(posDes > posLim[0]){
                posDes = posLim[0];
            }
            if(posDes < posLim[1]){
                posDes = posLim[1];
            }
            if(velDes > velLim[0]){
                velDes = velLim[0];
            }
            if(velDes < velLim[1]){
                velDes = velLim[1];
            }

            dertaPos = posDes - posRet;
            accInp = (velDes - this->velDesLst)/this->mT;
            this->velDesLst = velDes;
            mImpCtl.CalTorByDerta(dertaPos, velDes, accInp);
            TorDes = mImpCtl.ResOutp(2);

            float TorRet;
            if(this->TorCompMod == 1){
                TorRet = IqRet * this->Kt + this->Kt_inte;
            }else{
                TorRet = IqRet;
            }
            float TorCtlOutp = mForceControl.ControlResultCal(TorDes - TorRet);

            mOutp = TorCtlOutp + OtherTorComp;
        }else{
            mOutp = OtherTorComp;
        }
        return mOutp;
    }

    //--------------------PD+前馈控制器--------------------

    //初始化函数：
    void JntCtl_PDfeedforward::init(float posKp, float posKi, float posKd, float posKbInte, 
                                    float velKp, float velKi, float velKd, float velKbInte,
                                    float mposLim[], float mvelLim[], float moutpLim[],
                                    int mTorCompMod,
                                    bool mIfUseTorComp, float mKt, float mKtInte)
    {
        mPosPDctl.CotrollerInit(posKp, posKi, posKd, posKbInte, this->mT);
        mVelPDctl.CotrollerInit(velKp, velKi, velKd, velKbInte, this->mT);
        posLim[0] = mposLim[0];     posLim[1] = mposLim[1];
        velLim[0] = mvelLim[0];     velLim[1] = mvelLim[1];
        outpLim[0] = moutpLim[0];     outpLim[1] = moutpLim[1];
        this->TorCompMod = mTorCompMod;
        this->ifUseTorComp = mIfUseTorComp;
        this->Kt = mKt;
        this->KtInte = mKtInte;
    }

    //输出计算函数：
    float JntCtl_PDfeedforward::CtlOutp(float posDes, float IqRet, float posRet, float velRet, float OtherTorComp)
    {
        //输入量：目标位置、电流反馈、位置反馈、速度反馈、其他额外力补偿（如重力补偿就可以在这里输入）
        //注意，在电流反馈模式下,IqRet指的是电流，若在力矩反馈模式下，IqRet指的是反馈力矩
        //输入量单位：角度
        float mPosRes, mVelRes;
        float TorComp;
        if(posDes > posLim[0]){
            posDes = posLim[0];
        }
        if(posDes < posLim[1]){
            posDes = posLim[1];
        }
        float posErr, velErr;
        if(this->ifFirstTemp){
            //第一轮控制，输出量为静止：
            posErr = 0;
            mPosRes = mPosPDctl.ControlResultCal(posErr);
            velErr = mPosRes - 0;
            mVelRes = mVelPDctl.ControlResultCal(velErr);

            TorComp = 0;

            this->ifFirstTemp = false;
        }else{
            posErr = posDes - posRet;
            mPosRes = mPosPDctl.ControlResultCal(posErr);
            velErr = mPosRes - velRet;
            mVelRes = mVelPDctl.ControlResultCal(velErr);

            if(this->TorCompMod == 1){
                //电流控制模式
                TorComp = IqRet * this->Kt + this->KtInte;
            }else{
                //力矩直通模式
                TorComp = IqRet;
            }
        }

        //输出限值
        float moutp;
        if(this->ifUseTorComp){
            moutp = mVelRes + TorComp + OtherTorComp;
        }else{
            moutp = mVelRes + OtherTorComp;
        }
        if(moutp > outpLim[0]){
            moutp = outpLim[0];
        }
        if(moutp < outpLim[1]){
            moutp = outpLim[1];
        }
        return moutp;
    }

    int16_t dataTrans(float outp_f)
    {
        //输出量转化
        //依据协议，输出量范围为-32768～32767，输入量的范围为-327.68~327.67
        //这里将输出量范围削减到-32760~32760，以保证安全
        int16_t outp_i;
        if(round(outp_f*100) > 32760){
            outp_i = 32760;
        }else if(round(outp_f*100) < -32760){
            outp_i = -32760;
        }else{
            outp_i = round(outp_f*100);
        }

        return outp_i;
    }

    //--------------------导纳+位置控制器--------------------

}
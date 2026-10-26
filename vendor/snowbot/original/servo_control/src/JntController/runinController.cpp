#include<JntController/runinController.h>



//====================================================================================================
//三次B——样条插补函数

//样条基函数
void BSplineFitter::calculateBasisFunctions(double u, 
                                          double& N0, double& N1, 
                                          double& N2, double& N3) const {
    // 三次均匀B样条基函数
    N0 = (1 - 3*u + 3*u*u - u*u*u) / 6.0;
    N1 = (4 - 6*u*u + 3*u*u*u) / 6.0;
    N2 = (1 + 3*u + 3*u*u - 3*u*u*u) / 6.0;
    N3 = (u*u*u) / 6.0;
}

//构建均匀节点向量
void BSplineFitter::buildUniformKnotVector() {
    knotVector.clear();
    
    int n = controlPoint_len - 1;  // 控制点最大索引
    int m = n + degree + 1;            // 节点向量长度-1
    
    // 均匀节点向量：u_i = i
    for (int i = 0; i <= m; i++) {
        knotVector.push_back(static_cast<double>(i));
    }
}

// 计算有效参数范围
std::pair<double, double> BSplineFitter::getValidParameterRange() const {
    if (controlPoints.size() < 4) {
        return {0.0, 1.0};  // 默认范围
    }
    
    // 对于均匀三次B样条：
    // 有效参数范围是 [degree, n+1] = [3, controlPoints.size()]
    double u_min = 3.0;
    double u_max = static_cast<double>(controlPoints.size());
    
    return {u_min, u_max};
}

// 使用节点向量找到参数u所在的段
int BSplineFitter::findSpan(double u) const {
    int n = controlPoints.size() - 1;
    
    // 特殊情况
    if (u >= knotVector[n+1]) return n;
    
    // 二分查找
    int low = degree;
    int high = n + 1;
    int mid = (low + high) / 2;
    
    while (u < knotVector[mid] || u >= knotVector[mid+1]) {
        if (u < knotVector[mid]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    
    return mid;
}

// 新增：一维点组的最后一段精细采样
std::vector<double> BSplineFitter::sampleBSpline1D(double newPoint, int numSamples) {
    std::vector<double> result;
    
    // 添加新控制点
    controlPoints.push_back(newPoint);
    
    // 需要至少degree+1个控制点（4个点才能画一段三次B样条）
    if (controlPoints.size() < 4) {
    //if (true){
        // 线性插值过渡
        double slope = (newPoint - yValue_last) / numSamples;
        for (int i = 1; i <= numSamples; i++) {
            result.push_back(yValue_last + slope * i);
        }
    } else {
        //维护控制点数组
        if(controlPoints.size() > controlPoint_len){
            controlPoints.erase(controlPoints.begin());
        }

        //获取四个控制点
        double Pn3 = controlPoints[0];
        double Pn2 = controlPoints[1];
        double Pn1 = controlPoints[2];
        double Pn = controlPoints[3];

        // 重建节点向量
        buildUniformKnotVector();

        //使用节点向量确定采样范围：
        int n = controlPoints.size() - 1;
        double u_min = knotVector[degree];      // u₃
        double u_max = knotVector[n+1];         // u_{n+1}

        // 采样最后一段
        double u_start = knotVector[n];         // u_n
        double u_end = knotVector[n+1];         // u_{n+1}


        //在本段曲线内均匀采样
        for(int i = 0; i < numSamples; i++){
        double u = u_start + (u_end - u_start) * i / (numSamples - 1);
            
            // 找到u所在的段
            int span = findSpan(u);
            
            // 计算局部参数
            double t = (u - knotVector[span]) / 
                        (knotVector[span+1] - knotVector[span]);
            
            // 计算基函数（需要节点向量！）
            double N0, N1, N2, N3;
            calculateBasisFunctions(t, N0, N1, N2, N3);
            
            // 计算曲线点
            double value = controlPoints[span-3] * N0 +
                            controlPoints[span-2] * N1 +
                            controlPoints[span-1] * N2 +
                            controlPoints[span] * N3;
            
            result.push_back(value);

        }
    }
    
    yValue_last = newPoint;
    return result;
}
//--------------------------------------------------------------------------
//三次样条插值

void CubicSpline::calculateSplineCoefficients(){
    size_t n = x.size() - 1;  // 区间数量
    
    // 初始化系数向量
    a.resize(n + 1);
    b.resize(n + 1);
    c.resize(n + 1);
    d.resize(n + 1);
    
    // 设置a系数为y值
    for (size_t i = 0; i <= n; i++) {
        a[i] = y[i];
    }
    
    // 计算h_i = x_{i+1} - x_i
    std::vector<double> h(n);
    for (size_t i = 0; i < n; i++) {
        h[i] = x[i+1] - x[i];
    }
    
    // 计算alpha_i
    std::vector<double> alpha(n);
    for (size_t i = 1; i < n; i++) {
        alpha[i] = 3.0 * (a[i+1] - a[i]) / h[i] - 3.0 * (a[i] - a[i-1]) / h[i-1];
    }
    
    // 三对角矩阵求解c系数
    std::vector<double> l(n+1), mu(n+1), z(n+1);
    
    // 自然边界条件：二阶导数为0
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;
    
    for (size_t i = 1; i < n; i++) {
        l[i] = 2.0 * (x[i+1] - x[i-1]) - h[i-1] * mu[i-1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
    }
    
    l[n] = 1.0;
    z[n] = 0.0;
    c[n] = 0.0;
    
    // 回代求解c系数
    for (int i = n-1; i >= 0; i--) {
        c[i] = z[i] - mu[i] * c[i+1];
    }
    
    // 计算b和d系数
    for (size_t i = 0; i < n; i++) {
        b[i] = (a[i+1] - a[i]) / h[i] - h[i] * (c[i+1] + 2.0 * c[i]) / 3.0;
        d[i] = (c[i+1] - c[i]) / (3.0 * h[i]);
    }
}

// 查找x_val所在的区间索引
size_t CubicSpline::findSegment(double x_val) {
    // 处理边界情况
    if (x_val <= x[0]) return 0;
    if (x_val >= x[x.size()-1]) return x.size() - 2;
    
    // 二分查找
    auto it = std::upper_bound(x.begin(), x.end(), x_val);
    return std::distance(x.begin(), it) - 1;
}

//传入数据点，计算样条插值
void CubicSpline::SplineCal(){

    //检查x,y尺寸
    if (x.size() != y.size() || x.size() < 2) {
        throw std::invalid_argument("x和y必须具有相同的大小且至少包含2个点");
    }
    
    // 检查x是否严格递增
    for (size_t i = 1; i < x.size(); i++) {
        if (x[i] <= x[i-1]) {
            throw std::invalid_argument("x坐标必须严格递增");
        }
    }

    calculateSplineCoefficients();
}

//计算样条插值在指定点的值
double CubicSpline::interpolate(double x_val){
    // 查找x_val所在的区间
    size_t idx = findSegment(x_val);
    
    double dx = x_val - x[idx];
    return a[idx] + b[idx] * dx + c[idx] * dx * dx + d[idx] * dx * dx * dx;
}

//接口封装
std::vector<double> CubicSpline::cubicSplineCal(double y_lst, double y_des, int dataNum){
    //数据装入：
    if(ifFirstTemp){
        ifFirstTemp = false;

        x.push_back(x_temp);
        y.push_back(y_lst);

        x_temp += der_x_temp;
        x.push_back(x_temp);
        y.push_back(y_des);
    }else{

        x_temp += der_x_temp;
        x.push_back(x_temp);
        y.push_back(y_des);
    }

    if(x.size() > windowLen){
        x.erase(x.begin());
    }
    if(y.size() > windowLen){
        y.erase(y.begin());
        a.erase(a.begin());
        b.erase(b.begin());
        c.erase(c.begin());
        d.erase(d.begin());
    }

    SplineCal();

    //计算输出
    double x_base = x.at(x.size() - 1);
    double der_x = 1/dataNum;
    //double der_x = 0.001;
    std::vector<double> y_outp;
    for(int i = 0; i < dataNum; i++){
        double temp_x = x_base + (i+1)*der_x;
        double temp_y = interpolate(temp_x);
        y_outp.push_back(temp_y);
    }

    return y_outp;
}

//====================================================================================================
//滤波器定义：

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

//====================================================================================================
//求解核心定义：

//------------------------------私有函数------------------------------


//------------------------------公有函数------------------------------
std::vector<double> runin_core::liner_runin(double last_Inp, double des_Inp, int pointNum)
{
    std::vector<double> mRes;
    double derta = (des_Inp - last_Inp)/pointNum;
    for(int i = 0; i < pointNum; i++){
        mRes.push_back((i+1)*derta + last_Inp);
    }
    return mRes;
}

//样条曲线插补函数
std::vector<double> runin_core::Spline_fit(double last_Inp, double des_Inp, int pointNum){
    std::vector<double> mRes;
    //mRes = m_cubicSpline.cubicSplineCal(last_Inp, des_Inp, pointNum);

    /**/
    //int sampRate = 10;
    int sampRate = pointNum;
    mRes = m_splineFitter.sampleBSpline1D(des_Inp, sampRate);
    /**/

    return mRes;
}

//插补封装函数：
std::vector<double> runin_core::runin_seal(double last_Inp, double des_Inp, int pointNum, int runinMod){
    std::vector<double> mRes;

    if(runinMod == 1){
        //线性插补
        mRes = liner_runin(last_Inp, des_Inp, pointNum);
    }else if(runinMod == 2){
        //牛顿插补
        mRes = Spline_fit(last_Inp, des_Inp, pointNum);
    }else{
        //不插补，直接输出目标位置
        mRes.assign(pointNum, des_Inp);
    }

    return mRes;
}


//滑动加权平均滤波器
double WeightedMovingFtr::calculateWeightedAverage(){
    double weighted_sum = 0.0;
        
    for (size_t i = 0; i < window_size_; ++i) {
        weighted_sum += data_buffer_[i] * weights_[i];
    }
    
    return weighted_sum / sum_weights_;
}

WeightedMovingFtr::WeightedMovingFtr(size_t window_size, const std::vector<double>& weights)
    : window_size_(window_size), weights_(weights) {
    
    if (window_size == 0) {
        throw std::invalid_argument("Window size must be greater than 0");
    }
    
    if (weights.size() != window_size) {
        throw std::invalid_argument("Weights size must match window size");
    }
    
    // 计算权重总和
    sum_weights_ = std::accumulate(weights.begin(), weights.end(), 0.0);
    
    if (sum_weights_ == 0.0) {
        throw std::invalid_argument("Sum of weights cannot be zero");
    }
}

// 添加新数据并返回滤波后的值
double WeightedMovingFtr::filter(double new_value) {
    // 添加新数据
    data_buffer_.push_back(new_value);
    
    // 如果缓冲区超过窗口大小，移除最旧的数据
    if (data_buffer_.size() > window_size_) {
        data_buffer_.pop_front();
    }
    
    // 如果数据不足，返回原始值
    if (data_buffer_.size() < window_size_) {
        return new_value;
    }
    
    // 计算加权平均值
    return calculateWeightedAverage();
}


//====================================================================================================
//求解函数定义：
void runin_control::init(
    std::vector<double> lArmPos_FirstTempInp, std::vector<double> rArmPos_FirstTempInp,
    int m_trajLen
){
    //------------------------------------------------------------
    //刷新控制量与求解器，以及上一轮目标求解信息

    lastStRuninDotWrap.lArm_posDot.clear();
    lastStRuninDotWrap.lArm_velDot.clear();
    
    lastStRuninDotWrap.lArm_gravCompDot.clear();
    lastStRuninDotWrap.lArm_InteriaCompDot.clear();
    lastStRuninDotWrap.lArm_FeedForwardCompDot.clear();

    lastStRuninDotWrap.rArm_posDot.clear();
    lastStRuninDotWrap.rArm_velDot.clear();
    
    lastStRuninDotWrap.rArm_gravCompDot.clear();
    lastStRuninDotWrap.rArm_InteriaCompDot.clear();
    lastStRuninDotWrap.rArm_FeedForwardCompDot.clear();

    posRunin_core_l.clear();
    velRunin_core_l.clear();
    gravCompRunin_core_l.clear();
    InteriaCompRunin_core_l.clear();
    FeedForwardRunin_core_l.clear();

    posRunin_core_r.clear();
    velRunin_core_r.clear();
    gravCompRunin_core_r.clear();
    InteriaCompRunin_core_r.clear();
    FeedForwardRunin_core_r.clear();

    lastStRuninDotWrap.lArm_posDot = lArmPos_FirstTempInp;
    lastStRuninDotWrap.lArm_velDot.assign(singleWrapLen, 0.0);

    lastStRuninDotWrap.lArm_gravCompDot.assign(singleWrapLen, 0.0);
    lastStRuninDotWrap.lArm_FeedForwardCompDot.assign(singleWrapLen, 0.0);
    lastStRuninDotWrap.lArm_InteriaCompDot.assign(singleWrapLen, 0.0);

    lastStRuninDotWrap.rArm_posDot = rArmPos_FirstTempInp;
    lastStRuninDotWrap.rArm_velDot.assign(singleWrapLen, 0.0);

    lastStRuninDotWrap.rArm_gravCompDot.assign(singleWrapLen, 0.0);
    lastStRuninDotWrap.rArm_FeedForwardCompDot.assign(singleWrapLen, 0.0);
    lastStRuninDotWrap.rArm_InteriaCompDot.assign(singleWrapLen, 0.0);

    posRunin_core_l.resize(singleWrapLen);
    velRunin_core_l.resize(singleWrapLen);
    gravCompRunin_core_l.resize(singleWrapLen);
    InteriaCompRunin_core_l.resize(singleWrapLen);
    FeedForwardRunin_core_l.resize(singleWrapLen);

    posRunin_core_r.resize(singleWrapLen);
    velRunin_core_r.resize(singleWrapLen);
    gravCompRunin_core_r.resize(singleWrapLen);
    InteriaCompRunin_core_r.resize(singleWrapLen);
    FeedForwardRunin_core_r.resize(singleWrapLen);

    posFilter_l.resize(singleWrapLen);
    velFilter_l.resize(singleWrapLen);

    posFilter_r.resize(singleWrapLen);
    velFilter_r.resize(singleWrapLen);

    for(int i = 0; i < singleWrapLen; i++){
        posFilter_l[i].init(sampleRate, cutoffFreq);
        velFilter_l[i].init(sampleRate, cutoffFreq);

        posFilter_r[i].init(sampleRate, cutoffFreq);
        velFilter_r[i].init(sampleRate, cutoffFreq);
    }

    //滤波器初始化
    allJntPosFilter_l.assign(singleWrapLen, posOutpWtr);
    allJntVelFilter_l.assign(singleWrapLen, velOutpWtr);
    allJntPosFilter_r.assign(singleWrapLen, posOutpWtr);
    allJntVelFilter_r.assign(singleWrapLen, velOutpWtr);

    //刷新控制量
    trajLen = m_trajLen;

    ifFirstRunin = true;
    ifFirstTime = true;

}

void runin_control::ModelRunin(
    std::vector<double> posDes_l, std::vector<double> velDes_l, 
    std::vector<double> gravComp_l, std::vector<double> InteriaComp_l, std::vector<double> FeedForward_l,
    std::vector<double> posDes_r, std::vector<double> velDes_r,
    std::vector<double> gravComp_r, std::vector<double> InteriaComp_r, std::vector<double> FeedForward_r,
    std::vector<double> Kp_l, std::vector<double> Kd_l,
    std::vector<double> Kp_r, std::vector<double> Kd_r,
    int runinMod,
    int m_trajLen
){

    //获取插补轨迹
    std::vector<std::vector<double>> l_posTraj, r_posTraj;
    std::vector<std::vector<double>> l_velTraj, r_velTraj;
    std::vector<std::vector<double>> lArm_gravCompTraj, rArm_gravCompTraj;
    std::vector<std::vector<double>> lArm_InteriaCompTraj, rArm_InteriaCompTraj;
    std::vector<std::vector<double>> lArm_FeedForwardTraj, rArm_FeedForwardTraj;

    int mRuninMod;
    this->trajLen = m_trajLen;

    //刷控制模式：
    if(ifFirstRunin){
        mRuninMod = 1;
        ifFirstRunin = false;
    }else{
        mRuninMod = runinMod;
    }

    //计算插补轨迹
    for(int i = 0; i < singleWrapLen; i++){
        l_posTraj.push_back(posRunin_core_l[i].runin_seal(lastStRuninDotWrap.lArm_posDot[i], posDes_l[i], this->trajLen, mRuninMod));
        l_velTraj.push_back(velRunin_core_l[i].runin_seal(lastStRuninDotWrap.lArm_velDot[i], velDes_l[i], this->trajLen, mRuninMod));
        lArm_gravCompTraj.push_back(gravCompRunin_core_l[i].runin_seal(lastStRuninDotWrap.lArm_gravCompDot[i], gravComp_l[i], this->trajLen, mRuninMod));
        lArm_InteriaCompTraj.push_back(InteriaCompRunin_core_l[i].runin_seal(lastStRuninDotWrap.lArm_InteriaCompDot[i], InteriaComp_l[i], this->trajLen, mRuninMod));
        lArm_FeedForwardTraj.push_back(FeedForwardRunin_core_l[i].runin_seal(lastStRuninDotWrap.lArm_FeedForwardCompDot[i], FeedForward_l[i], this->trajLen, mRuninMod));

        r_posTraj.push_back(posRunin_core_r[i].runin_seal(lastStRuninDotWrap.rArm_posDot[i], posDes_r[i], this->trajLen, mRuninMod));
        r_velTraj.push_back(velRunin_core_r[i].runin_seal(lastStRuninDotWrap.rArm_velDot[i], velDes_r[i], this->trajLen, mRuninMod));
        rArm_gravCompTraj.push_back(gravCompRunin_core_r[i].runin_seal(lastStRuninDotWrap.rArm_gravCompDot[i], gravComp_r[i], this->trajLen, mRuninMod));
        rArm_InteriaCompTraj.push_back(InteriaCompRunin_core_r[i].runin_seal(lastStRuninDotWrap.rArm_InteriaCompDot[i], InteriaComp_r[i], this->trajLen, mRuninMod));
        rArm_FeedForwardTraj.push_back(FeedForwardRunin_core_r[i].runin_seal(lastStRuninDotWrap.rArm_FeedForwardCompDot[i], FeedForward_r[i], this->trajLen, mRuninMod));
    }

    //上一轮插补轨迹清空
    m_runin_traj_wrap.runinDataTRaj.clear();
    m_runin_traj_wrap.lArmJnt_Kp.clear();
    m_runin_traj_wrap.lArmJnt_Kd.clear();
    m_runin_traj_wrap.rArmJnt_Kp.clear();
    m_runin_traj_wrap.rArmJnt_Kd.clear();

    //插补轨迹封装
    m_runin_traj_wrap.TrajLen = this->trajLen;
    for(int j = 0; j < this->trajLen; j++){
        singleTempRuninData temp_runinData;
        
        for(int i = 0; i < this->singleWrapLen; i++){

            temp_runinData.lArm_posDot.push_back(l_posTraj[i][j]);
            temp_runinData.lArm_velDot.push_back(l_velTraj[i][j]);
            temp_runinData.lArm_gravCompDot.push_back(lArm_gravCompTraj[i][j]);
            temp_runinData.lArm_InteriaCompDot.push_back(lArm_InteriaCompTraj[i][j]);
            temp_runinData.lArm_FeedForwardCompDot.push_back(lArm_FeedForwardTraj[i][j]);

            temp_runinData.rArm_posDot.push_back(r_posTraj[i][j]);
            temp_runinData.rArm_velDot.push_back(r_velTraj[i][j]);
            temp_runinData.rArm_gravCompDot.push_back(rArm_gravCompTraj[i][j]);
            temp_runinData.rArm_InteriaCompDot.push_back(rArm_InteriaCompTraj[i][j]);
            temp_runinData.rArm_FeedForwardCompDot.push_back(rArm_FeedForwardTraj[i][j]);
        }

        m_runin_traj_wrap.runinDataTRaj.push_back(temp_runinData);
    }

    m_runin_traj_wrap.lArmJnt_Kp = Kp_l;
    m_runin_traj_wrap.lArmJnt_Kd = Kd_l;
    m_runin_traj_wrap.rArmJnt_Kp = Kp_r;
    m_runin_traj_wrap.rArmJnt_Kd = Kd_r;

    m_runin_traj_wrap.lArm_tempDesPos = posDes_l;
    m_runin_traj_wrap.rArm_tempDesPos = posDes_r;

    //刷新上一轮控制结果
    lastStRuninDotWrap.lArm_posDot = posDes_l;
    lastStRuninDotWrap.lArm_velDot = velDes_l;
    lastStRuninDotWrap.lArm_gravCompDot = gravComp_l;
    lastStRuninDotWrap.lArm_InteriaCompDot = InteriaComp_l;
    lastStRuninDotWrap.lArm_FeedForwardCompDot = FeedForward_l;

    lastStRuninDotWrap.rArm_posDot = posDes_r;
    lastStRuninDotWrap.rArm_velDot = velDes_r;
    lastStRuninDotWrap.rArm_gravCompDot = gravComp_r;
    lastStRuninDotWrap.rArm_InteriaCompDot = InteriaComp_r;
    lastStRuninDotWrap.rArm_FeedForwardCompDot = FeedForward_r;

    //控制量刷新
    this->loopTime = 0;
}

runinTrajOutp runin_control::dataOutp(bool topic_stream)
{
    runinTrajOutp mOutp;

    if(loopTime < m_runin_traj_wrap.TrajLen){
        if(ifUseFtr){
            singleTempRuninData temp_runinData;
            temp_runinData.lArm_posDot.resize(m_runin_traj_wrap.runinDataTRaj[loopTime].lArm_posDot.size());
            temp_runinData.lArm_velDot.resize(m_runin_traj_wrap.runinDataTRaj[loopTime].lArm_velDot.size());

            temp_runinData.rArm_posDot.resize(m_runin_traj_wrap.runinDataTRaj[loopTime].rArm_posDot.size());
            temp_runinData.rArm_velDot.resize(m_runin_traj_wrap.runinDataTRaj[loopTime].rArm_velDot.size());

            for(int i = 0; i < temp_runinData.lArm_posDot.size(); i++){
                temp_runinData.lArm_posDot[i] = posFilter_l[i].filter(m_runin_traj_wrap.runinDataTRaj[loopTime].lArm_posDot[i]);
                temp_runinData.lArm_velDot[i] = velFilter_l[i].filter(m_runin_traj_wrap.runinDataTRaj[loopTime].lArm_velDot[i]);
            }
            for(int i = 0; i < temp_runinData.rArm_posDot.size(); i++){
                temp_runinData.rArm_posDot[i] = posFilter_r[i].filter(m_runin_traj_wrap.runinDataTRaj[loopTime].rArm_posDot[i]);
                temp_runinData.rArm_velDot[i] = velFilter_r[i].filter(m_runin_traj_wrap.runinDataTRaj[loopTime].rArm_velDot[i]);
            }

            temp_runinData.lArm_gravCompDot = m_runin_traj_wrap.runinDataTRaj[loopTime].lArm_gravCompDot;
            temp_runinData.lArm_InteriaCompDot = m_runin_traj_wrap.runinDataTRaj[loopTime].lArm_InteriaCompDot;
            temp_runinData.lArm_FeedForwardCompDot = m_runin_traj_wrap.runinDataTRaj[loopTime].lArm_FeedForwardCompDot;

            temp_runinData.rArm_gravCompDot = m_runin_traj_wrap.runinDataTRaj[loopTime].rArm_gravCompDot;
            temp_runinData.rArm_InteriaCompDot = m_runin_traj_wrap.runinDataTRaj[loopTime].rArm_InteriaCompDot;
            temp_runinData.rArm_FeedForwardCompDot = m_runin_traj_wrap.runinDataTRaj[loopTime].rArm_FeedForwardCompDot;

            mOutp.runinData = temp_runinData;
        }else{
            mOutp.runinData = m_runin_traj_wrap.runinDataTRaj[loopTime];
        }
        mOutp.lArmJnt_Kp = m_runin_traj_wrap.lArmJnt_Kp;
        mOutp.lArmJnt_Kd = m_runin_traj_wrap.lArmJnt_Kd;
        mOutp.rArmJnt_Kp = m_runin_traj_wrap.rArmJnt_Kp;
        mOutp.rArmJnt_Kd = m_runin_traj_wrap.rArmJnt_Kd;
    }else{
        //创建恒定位置输出，此时的输出仅有
        singleTempRuninData m_runinData;
        m_runinData.lArm_posDot = m_runin_traj_wrap.lArm_tempDesPos;
        m_runinData.rArm_posDot = m_runin_traj_wrap.rArm_tempDesPos;

        m_runinData.lArm_gravCompDot = m_runin_traj_wrap.runinDataTRaj[m_runin_traj_wrap.TrajLen - 1].lArm_gravCompDot;
        m_runinData.rArm_gravCompDot = m_runin_traj_wrap.runinDataTRaj[m_runin_traj_wrap.TrajLen - 1].rArm_gravCompDot;

        m_runinData.lArm_velDot.assign(singleWrapLen, 0.0);
        m_runinData.lArm_InteriaCompDot.assign(singleWrapLen, 0.0);
        m_runinData.lArm_FeedForwardCompDot.assign(singleWrapLen, 0.0);

        m_runinData.rArm_velDot.assign(singleWrapLen, 0.0);
        // Topic velocity is a feedforward command. Missing a single packet must
        // not switch it abruptly to zero; decay it while holding the position.
        if (topic_stream && !ifFirstTime) {
            const double decay = std::exp(-0.001 / 0.05);
            for (int i = 0; i < singleWrapLen; ++i) {
                m_runinData.lArm_velDot[i] = last_outp.runinData.lArm_velDot[i] * decay;
                m_runinData.rArm_velDot[i] = last_outp.runinData.rArm_velDot[i] * decay;
            }
        }
        m_runinData.rArm_InteriaCompDot.assign(singleWrapLen, 0.0);
        m_runinData.rArm_FeedForwardCompDot.assign(singleWrapLen, 0.0);

        //输出包装
        mOutp.runinData = m_runinData;
        mOutp.lArmJnt_Kp = m_runin_traj_wrap.lArmJnt_Kp;
        mOutp.lArmJnt_Kd = m_runin_traj_wrap.lArmJnt_Kd;
        mOutp.rArmJnt_Kp = m_runin_traj_wrap.rArmJnt_Kp;
        mOutp.rArmJnt_Kd = m_runin_traj_wrap.rArmJnt_Kd;
    }

    if(loopTime < 1e10){
        loopTime += 1;
    }

    //做输出限幅度
    if(!ifFirstTime){
        for(int i = 0; i < mOutp.runinData.lArm_posDot.size(); i++){
            if((mOutp.runinData.lArm_posDot[i] - last_outp.runinData.lArm_posDot[i]) > derPos_upperLim[i]){
                mOutp.runinData.lArm_posDot[i] = last_outp.runinData.lArm_posDot[i] + derPos_upperLim[i];
            }
            if((mOutp.runinData.lArm_posDot[i] - last_outp.runinData.lArm_posDot[i]) < derPos_opUpperLim[i]){
                mOutp.runinData.lArm_posDot[i] = last_outp.runinData.lArm_posDot[i] + derPos_opUpperLim[i];
            }

            if((mOutp.runinData.lArm_velDot[i] - last_outp.runinData.lArm_velDot[i]) > derVel_upperLim[i]){
                mOutp.runinData.lArm_velDot[i] = last_outp.runinData.lArm_velDot[i] + derVel_upperLim[i];
            }
            if((mOutp.runinData.lArm_velDot[i] - last_outp.runinData.lArm_velDot[i]) < derVel_opUpperLim[i]){
                mOutp.runinData.lArm_velDot[i] = last_outp.runinData.lArm_velDot[i] + derVel_opUpperLim[i];
            }

            if((mOutp.runinData.rArm_posDot[i] - last_outp.runinData.rArm_posDot[i]) > derPos_upperLim[i]){
                mOutp.runinData.rArm_posDot[i] = last_outp.runinData.rArm_posDot[i] + derPos_upperLim[i];
            }
            if((mOutp.runinData.rArm_posDot[i] - last_outp.runinData.rArm_posDot[i]) < derPos_opUpperLim[i]){
                 mOutp.runinData.rArm_posDot[i] = last_outp.runinData.rArm_posDot[i] + derPos_opUpperLim[i];
            }

            if((mOutp.runinData.rArm_velDot[i] - last_outp.runinData.rArm_velDot[i]) > derVel_upperLim[i]){
                mOutp.runinData.rArm_velDot[i] = last_outp.runinData.rArm_velDot[i] + derVel_upperLim[i];
            }
            if((mOutp.runinData.rArm_velDot[i] - last_outp.runinData.rArm_velDot[i]) < derVel_opUpperLim[i]){
                mOutp.runinData.rArm_velDot[i] = last_outp.runinData.rArm_velDot[i] + derVel_opUpperLim[i];
            }
        }

    }
    ifFirstTime = false;
    last_outp = mOutp;


    runinTrajOutp mOutp_ftred = mOutp;
    if(ifUseOutpFtr){
        for(int i = 0; i < mOutp.runinData.lArm_posDot.size(); i++){
            mOutp_ftred.runinData.lArm_posDot[i] = allJntPosFilter_l[i].filter(mOutp.runinData.lArm_posDot[i]);
            mOutp_ftred.runinData.lArm_velDot[i] = allJntVelFilter_l[i].filter(mOutp.runinData.lArm_velDot[i]);
        }
        for(int i = 0; i < mOutp.runinData.rArm_posDot.size(); i++){
            mOutp_ftred.runinData.rArm_posDot[i] = allJntPosFilter_r[i].filter(mOutp.runinData.rArm_posDot[i]);
            mOutp_ftred.runinData.rArm_velDot[i] = allJntVelFilter_r[i].filter(mOutp.runinData.rArm_velDot[i]);
        }
        return mOutp_ftred;
    }else{
        return mOutp;
    }
}

#include "controller/upper_Transmit.h"
#include "robot_control/config.h"

//--------------------------------------------------
//输入数据处理代码：

//-----三次B-样条曲线插补代码-----

void BSplineFitter_vr::calculateBasisFunctions(double u, 
                                          double& N0, double& N1, 
                                          double& N2, double& N3) const {
    // 三次均匀B样条基函数
    N0 = (1 - 3*u + 3*u*u - u*u*u) / 6.0;
    N1 = (4 - 6*u*u + 3*u*u*u) / 6.0;
    N2 = (1 + 3*u + 3*u*u - 3*u*u*u) / 6.0;
    N3 = (u*u*u) / 6.0;
}

std::vector<double> BSplineFitter_vr::evaluateCurvePoint(
    const std::vector<std::vector<double>>& controlPoints,
    double u){
    
    if (controlPoints.size() < 4) {
        throw std::runtime_error("控制点数量不足，需要至少4个点");
    }
    
    // 找到对应的曲线段
    int segment = static_cast<int>(u);
    if (segment < 0) segment = 0;
    if (segment > controlPoints.size() - 4) {
        segment = controlPoints.size() - 4;
    }
    
    double local_u = u - segment;
    
    // 计算基函数
    double N0, N1, N2, N3;
    calculateBasisFunctions(local_u, N0, N1, N2, N3);
    
    // 计算曲线点坐标
    double x = controlPoints[segment][0] * N0 +
               controlPoints[segment + 1][0] * N1 +
               controlPoints[segment + 2][0] * N2 +
               controlPoints[segment + 3][0] * N3;
    
    double y = controlPoints[segment][1] * N0 +
               controlPoints[segment + 1][1] * N1 +
               controlPoints[segment + 2][1] * N2 +
               controlPoints[segment + 3][1] * N3;
    
    return {x, y};
}

std::vector<std::vector<double>> BSplineFitter_vr::generateBSplineCurve(
    const std::vector<std::vector<double>>& controlPoints,
    int samplesPerSegment){
    
    if (controlPoints.size() < 4) {
        //返回原始值
        return controlPoints;
    }

    controlPoint_num = controlPoints.size() - 3;
    m_samplePerSegment = samplesPerSegment + 1;
    
    std::vector<std::vector<double>> curvePoints;
    int segmentCount = controlPoints.size() - 3;
    
    // 生成每个曲线段的点
    for (int i = 0; i < segmentCount; ++i) {
        for (int j = 0; j <= samplesPerSegment; ++j) {
            double u = i + static_cast<double>(j) / samplesPerSegment;
            auto point = evaluateCurvePoint(controlPoints, u);
            curvePoints.push_back(point);
        }
    }
    
    return curvePoints;
}

std::vector<std::vector<double>> BSplineFitter_vr::fit(
    const std::vector<std::vector<double>>& dataPoints,
    int samplesPerSegment){
    
    // 直接使用原始数据点作为控制点（非夹紧）
    return generateBSplineCurve(dataPoints, samplesPerSegment);
}

std::vector<double> BSplineFitter_vr::fit1D(
    const std::vector<double>& yValues,
    int samplesPerSegment,
    int windowLen
){
    
    // 将一维数据转换为二维点（x均匀分布）
    for (size_t i = 0; i < yValues.size(); ++i) {
        double x = static_cast<double>(i) / (yValues.size() - 1); // 归一化到[0,1]
        dataPoints.push_back({x, yValues[i]});
        if(dataPoints.size() > windowLen){
            dataPoints.erase(dataPoints.begin());
        }
    }
    
    auto curvePoints = fit(dataPoints, samplesPerSegment);
    
    // 提取y值作为拟合结果
    std::vector<double> result;
    for (const auto& point : curvePoints) {
        result.push_back(point[1]);
    }
    
    return result;
}

// 新增：一维点组的最后一段精细采样
std::vector<double> BSplineFitter_vr::getLastSegmentPoints1D(
    const std::vector<double>& yValues,
    int pointNum,
    int windowLen
) {
    
    // 将一维数据转换为二维点（x均匀分布）
    for (size_t i = 0; i < yValues.size(); ++i) {
        double x = static_cast<double>(i) / (yValues.size() - 1); // 归一化到[0,1]
        dataPoints.push_back({x, yValues[i]});
        if(dataPoints.size() > windowLen){
            dataPoints.erase(dataPoints.begin());
        }
    }

    std::vector<double> resultPoints;
    
    // 确定最后一段的索引
    int lastSegmentIndex = dataPoints.size() - 4;
    
    // 最后一段使用的控制点：最后4个控制点
    const auto& Pn3 = dataPoints[lastSegmentIndex];     // P_{n-3}
    const auto& Pn2 = dataPoints[lastSegmentIndex + 1]; // P_{n-2}  
    const auto& Pn1 = dataPoints[lastSegmentIndex + 2]; // P_{n-1}
    const auto& Pn  = dataPoints[lastSegmentIndex + 3]; // P_{n} (最后一个点)
    
    // 在最后一段 [lastSegmentIndex, lastSegmentIndex+1] 内均匀采样
    for (int i = 0; i < pointNum; ++i) {
        double u = lastSegmentIndex + static_cast<double>(i) / (pointNum - 1);
        
        // 计算局部参数 (确保在 [0,1] 范围内)
        double local_u = u - lastSegmentIndex;
        if (i == pointNum - 1) {
            local_u = 1.0; // 确保最后一个点精确对应最后一个控制点
        }
        
        // 计算基函数
        double N0, N1, N2, N3;
        calculateBasisFunctions(local_u, N0, N1, N2, N3);
        
        // 计算曲线点的y坐标（一维输出，只关心y值）
        double y = Pn3[1] * N0 + Pn2[1] * N1 + Pn1[1] * N2 + Pn[1] * N3;
        
        resultPoints.push_back(y);
    }
    
    return resultPoints;
}

//参数输出 -- 拟合点的个数
int BSplineFitter_vr::pointNumInCurve(){
    return m_samplePerSegment * controlPoint_num;
}

//-----一般的三次样条曲线拟合-----

void CubicSpline_vr::calculateSplineCoefficients(){
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
size_t CubicSpline_vr::findSegment(double x_val) {
    // 处理边界情况
    if (x_val <= x[0]) return 0;
    if (x_val >= x[x.size()-1]) return x.size() - 2;
    
    // 二分查找
    auto it = std::upper_bound(x.begin(), x.end(), x_val);
    return std::distance(x.begin(), it) - 1;
}

//传入数据点，计算样条插值
void CubicSpline_vr::SplineCal(){

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
double CubicSpline_vr::interpolate(double x_val){
    // 查找x_val所在的区间
    size_t idx = findSegment(x_val);
    
    double dx = x_val - x[idx];
    return a[idx] + b[idx] * dx + c[idx] * dx * dx + d[idx] * dx * dx * dx;
}

//接口封装
std::vector<double> CubicSpline_vr::cubicSplineCal(double y_lst, double y_des, int dataNum){
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
    std::vector<double> y_outp;
    for(int i = 0; i < dataNum; i++){
        double temp_x = x_base + (i+1)*der_x;
        double temp_y = interpolate(temp_x);
        y_outp.push_back(temp_y);
    }

    return y_outp;
}

//-----卡尔曼滤波器-----
void kalman_filter_vr::mMatrixMul(double a[],double b[],int m,int n,int k,double c[])
{ int i,j,l,u;
    for (i=0; i<=m-1; i++){
        for (j=0; j<=k-1; j++)
        { u=i*k+j; c[u]=0.0;
            for (l=0; l<=n-1; l++)
            c[u]=c[u]+a[i*n+l]*b[l*k+j];
        }
    }
}

void kalman_filter_vr::FilterInit(double inputVar, double mQgain, double mRgain)
{
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

double kalman_filter_vr::FilteredResCal(double inputvar)
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
    double I_minus_KH[2][2] = {{1.0f - K, 0.0f}, {-K, 1.0f}}; // 实际上，对于一维情况，这简化为[1-K, 0; -K, 1]（但保留2x2形式以备将来扩展）
    mMatrixMul(I_minus_KH[0], FAIPFAIT[0], 2, 2, 2, Pmtrx[0]); // 更新后的协方差矩阵

    // 返回结果
    return outvar[0];
}

//--------------------------------------------------
//插补核心代码 -- 先进行三次样条曲线插补，再进行卡尔曼滤波

double Inp_DataProcess::Spline_fit(double des_Inp){
    std::vector<double> t_Inp = {des_Inp};
    std::vector<double> mRes;

    //样条曲线平滑
    if(splineFitter_choose == 1){
        Cubic_spFit.windowLen = Cubic_Fit_winLen;
        mRes = Cubic_spFit.cubicSplineCal(lastInp, des_Inp, 1);
    }else{
        std::vector<double> n_res = B_spFit.fit1D(t_Inp, B_Fit_sampRate, B_Fit_winLen);
        mRes = n_res.empty() ? std::vector<double>{des_Inp} : std::vector<double>{n_res.back()};
    }

    //滤波器初始化
    if(ifFirstTemp){
        ifFirstTemp  = false;
        dKalmanFtr.FilterInit(Qgain, Rgain, mRes.back());
    }

    //滤波器输出
    double res;
    res = dKalmanFtr.FilteredResCal(mRes.back());

    return res;
}

//--------------------------------------------------
//数据处理包装1 -- 笛卡尔空间数据处理

void arm_traj_ctl_cartesian::init(){
   
    l_arm_end_pos.assign(3, 0.0);
    r_arm_end_pos.assign(3, 0.0);
    l_arm_end_delta.assign(3, 0.0);
    r_arm_end_delta.assign(3, 0.0);
    lArm_end_pos_rpy.assign(6, 0.0);
    rArm_end_pos_rpy.assign(6, 0.0);

    //控制参量初始化
    stabtimecount = 0;
    ifUseStFtr = true;

    //初始信息包装
    //关节空间：
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间：
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;
    ctlStMsg.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_l_JointSpace.assign(SingleArmDof, 0.0);
    ctlStMsg.desTor_r_JointSpace.assign(SingleArmDof, 0.0);

    ctlDesMsg = ctlStMsg;
}


void arm_traj_ctl_cartesian::trans_and_outp_core(){
    // 数据转化 - 将ZYX顺序的欧拉角转换为四元数
    std::vector<double> lArm_quat_ori = eulerToQuaternionZYX(l_arm_end_pos);
    std::vector<double> rArm_quat_ori = eulerToQuaternionZYX(r_arm_end_pos);
    
    std::vector<double> base_quat_l = eulerToQuaternionZYX(gesture_trans_l);
    std::vector<double> base_quat_r = eulerToQuaternionZYX(gesture_trans_r);
    
    // 使用四元数乘法进行姿态组合
    std::vector<double> lArm_quat_world = multiplyQuaternions(base_quat_l, lArm_quat_ori);
    std::vector<double> rArm_quat_world = multiplyQuaternions(base_quat_r, rArm_quat_ori);
    
    // 将四元数转换回ZYX顺序的欧拉角
    std::vector<double> lArm_rpy_world = quaternionToEulerZYX(lArm_quat_world);
    std::vector<double> rArm_rpy_world = quaternionToEulerZYX(rArm_quat_world);
    
    // 数据点包装
    for(int i = 0; i < 3; i++){
        lArm_end_pos_rpy[i] = l_arm_end_delta[i] + pos_trans_l[i];
        rArm_end_pos_rpy[i] = r_arm_end_delta[i] + pos_trans_r[i];
        
        lArm_end_pos_rpy[i+3] = lArm_rpy_world[i];
        rArm_end_pos_rpy[i+3] = rArm_rpy_world[i];
    }
    
    ctlDesMsg = ctlStMsg;
    ctlDesMsg.desPos_l_Descartes = lArm_end_pos_rpy;
    ctlDesMsg.desPos_r_Descartes = rArm_end_pos_rpy;
}

//--------------位姿初始化部分--------------

/*变量转化与输出
--------------------
controlMod -- 控制模式： 0 —— 点位控制模式；1 —— 轨迹控制模式
*/
ctlDesMsgWrap arm_traj_ctl_cartesian::trans_and_outp(bool ifReachEnd, int controlMod){

    if(ifFirstOutp){
        ifFirstOutp = false;
        for(int i = 0; i < l_arm_end_pos.size(); i++){
            l_arm_end_pos[i] = 0.0;
        }
        for(int i = 0; i < l_arm_end_delta.size(); i++){
            l_arm_end_delta[i] = 0.0;
        }

        for(int i = 0; i < r_arm_end_pos.size(); i++){
            r_arm_end_pos[i] = 0.0;
        }
        for(int i = 0; i < r_arm_end_delta.size(); i++){
            r_arm_end_delta[i] = 0.0;
        }
        ifUseStFtr = true;
        stabtimecount = 0;
    }
    
    trans_and_outp_core();

    ///if(controlMod == 0){
    if(true){
        //点位控制
        return ctlDesMsg;
    }else{
        //轨迹控制
        ctlDesMsgWrap ctlDesMsg_ftr = ctlDesMsg;
        for(int i = 0; i < ctlDesMsg_ftr.desPos_l_Descartes.size(); i++){
            ctlDesMsg_ftr.desPos_l_Descartes[i] = l_ftr[i].Spline_fit(ctlDesMsg.desPos_l_Descartes[i]);
        }
        for(int i = 0; i < ctlDesMsg_ftr.desPos_r_Descartes.size(); i++){
            ctlDesMsg_ftr.desPos_r_Descartes[i] = r_ftr[i].Spline_fit(ctlDesMsg.desPos_r_Descartes[i]);
        }

        if(ifUseStFtr){
            if(stabtimecount >= ststabTimes){
                ifUseStFtr = false;
            }
            stabtimecount += 1;

            return ctlDesMsg;
        }else{
            return ctlDesMsg_ftr;      
        }
    }
}

//----------------------------------------------------------------------------------
//辅助函数新添加
// 四元数乘法（保持不变）
std::vector<double> multiplyQuaternions(const std::vector<double>& q1, const std::vector<double>& q2) {
    std::vector<double> result(4);
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]; // w
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]; // x
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]; // y
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]; // z
    return result;
}

// 欧拉角转四元数（ZYX顺序 - 先绕Z轴，再Y轴，最后X轴）
std::vector<double> eulerToQuaternionZYX(const std::vector<double>& euler) {
    double yaw = euler[0];   // Z轴旋转
    double pitch = euler[1]; // Y轴旋转  
    double roll = euler[2];  // X轴旋转
    
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    std::vector<double> q(4);
    q[0] = cr * cp * cy + sr * sp * sy; // w
    q[1] = sr * cp * cy - cr * sp * sy; // x
    q[2] = cr * sp * cy + sr * cp * sy; // y
    q[3] = cr * cp * sy - sr * sp * cy; // z
    
    return q;
}

// 四元数转欧拉角（ZYX顺序）
std::vector<double> quaternionToEulerZYX(const std::vector<double>& q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    
    std::vector<double> euler(3);
    
    // 绕Z轴旋转（yaw）
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler[0] = atan2(siny_cosp, cosy_cosp);
    
    // 绕Y轴旋转（pitch）
    double sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1.0) {
        euler[1] = copysign(M_PI / 2.0, sinp); // 使用90度如果超出范围
    } else {
        euler[1] = asin(sinp);
    }
    
    // 绕X轴旋转（roll）
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler[2] = atan2(sinr_cosp, cosr_cosp);
    
    return euler;
}
//----------------------------------------------------------------------------------
//数据处理包装2 -- 关节空间数据处理
void arm_traj_cal_JntSpace::init(){
    l_arm_joint_angular.assign(7, 0.0);
    r_arm_joint_angular.assign(7, 0.0);

    //控制参量初始化
    stabtimecount = 0;
    ifUseStFtr = true;

    //初始信息包装
    //关节空间：
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间：
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;
    ctlStMsg.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_l_JointSpace.assign(SingleArmDof, 0.0);
    ctlStMsg.desTor_r_JointSpace.assign(SingleArmDof, 0.0);

    ctlDesMsg = ctlStMsg;
}

void arm_traj_cal_JntSpace::trans_and_outp_core(){
    ctlDesMsg.desPos_l_Descartes.assign(6, 0.0);
    ctlDesMsg.desPos_r_Descartes.assign(6, 0.0);
    ctlDesMsg.desPos_l_JointSpace = l_arm_joint_angular;
    ctlDesMsg.desPos_r_JointSpace = r_arm_joint_angular;

    ctlDesMsg.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg.desTor_l_JointSpace.assign(6, 0.0);
    ctlDesMsg.desTor_r_JointSpace.assign(6, 0.0);
}

/*变量转化与输出
--------------------
controlMod -- 控制模式： 0 —— 点位控制模式；1 —— 轨迹控制模式
*/
ctlDesMsgWrap arm_traj_cal_JntSpace::trans_and_outp(bool ifReachEnd, int controlMod){

    trans_and_outp_core();

    if(controlMod == 0){
        //点位控制
        return ctlDesMsg;
    }else{
        //轨迹控制

        if(ifUseStFtr){
            if(stabtimecount >= ststabTimes){
                ifUseStFtr = false;
            }
            stabtimecount += 1;

            return ctlDesMsg;
        }else{
            // 先复制原始目标，避免空包装导致连续关节轨迹在预热阶段后变成空数组。
            ctlDesMsgWrap ctlDesMsg_ftr = ctlDesMsg;
            for(int i = 0; i < ctlDesMsg_ftr.desPos_l_JointSpace.size(); i++){
                ctlDesMsg_ftr.desPos_l_JointSpace[i] = l_ftr[i].Spline_fit(ctlDesMsg.desPos_l_JointSpace[i]);
            }
            for(int i = 0; i < ctlDesMsg_ftr.desPos_r_JointSpace.size(); i++){
                ctlDesMsg_ftr.desPos_r_JointSpace[i] = r_ftr[i].Spline_fit(ctlDesMsg.desPos_r_JointSpace[i]);
            }
            return ctlDesMsg_ftr;
        }
    }
}


//----------------------------------------------------------------------------------
//数据处理核心代码1 -- 机械臂顶层下发轨迹处理 -- 包含针对两种轨迹的处理

void arm_traj_ctl::init(){

    //--------------------初始化输出包装变量--------------------
    //初始化角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };
    rArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };

    //初始信息包装
    //关节空间：
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间：
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    //初始化目标位置
    ctlDesMsgWrap ctlStMsg;
    ctlStMsg.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_l_JointSpace.assign(SingleArmDof, 0.0);
    ctlStMsg.desTor_r_JointSpace.assign(SingleArmDof, 0.0);

    //控制值初始化
    //-----数据传输与插补模式-----
    mdataSend.ctl_transMod = 4;
    mdataSend.runinMod = 1;

    //-----求解模式信息-----
    mdataSend.ifUseRet = 0;
    mdataSend.ifUseTemp = 0;

    mdataSend.KpTrans_l.resize(SingleArmDof);
    mdataSend.KdTrans_l.resize(SingleArmDof);
    mdataSend.KpTrans_r.resize(SingleArmDof);
    mdataSend.KdTrans_r.resize(SingleArmDof);

    for(int i = 0; i < SingleArmDof; i++){
        mdataSend.rigidityCtl_mod_l[i] = this->rigidity_mod_l[i];
        mdataSend.rigidityCtl_mod_r[i] = this->rigidity_mod_r[i];
    }

    for(int i = 0; i < SingleArmDof; i++){
        mdataSend.KpTrans_l[i] = Kp_l_default[i];
        mdataSend.KdTrans_l[i] = Kd_l_default[i];

        mdataSend.KpTrans_r[i] = Kp_r_default[i];
        mdataSend.KdTrans_r[i] = Kd_r_default[i];
    }

    //-----重力补偿控制-----
    mdataSend.GravCompCtlMsg.ifUseGravComp = 1;
    mdataSend.GravCompCtlMsg.mIfUseFricComp = 0;
    mdataSend.GravCompCtlMsg.mIfUseInpFtr = 1;
    mdataSend.GravCompCtlMsg.mIfUseOutpFtr = 0;
    mdataSend.GravCompCtlMsg.GravCompInpMod = 1;

    mdataSend.stJntID_lArm = 0;
    mdataSend.stJntID_rArm = 7;
    mdataSend.urdf_choose = 2;

    //-----插补信息包装-----
    //发送变量初始化
    double trajTime = 2.0;
    double accRate = 1.2;
    runinStabMsgWrap stabMsg;
    stabMsg.ifUseStab = true;
    stabMsg.start_stabTimes = 50;
    stabMsg.des_stabTimes = 50;

    mdataSend.TrajTime.resize(2);
    mdataSend.TrajTime[0].assign(SingleArmDof, trajTime);
    mdataSend.TrajTime[1].assign(SingleArmDof, trajTime);

    mdataSend.accRateBase.resize(2);
    mdataSend.accRateBase[0].assign(SingleArmDof, accRate);
    mdataSend.accRateBase[1].assign(SingleArmDof, accRate);

    mdataSend.m_stabMsg.resize(2);
    mdataSend.m_stabMsg[0].assign(SingleArmDof, stabMsg);
    mdataSend.m_stabMsg[1].assign(SingleArmDof, stabMsg);

    //-----开关量包装-----
    mdataSend.ifJntStab.resize(2);
    mdataSend.ifJntStab[0] = lArm_ifJntStab;
    mdataSend.ifJntStab[1] = rArm_ifJntStab;

    mdataSend.ifRedefineJntTrajStab = false;

    //-----关节限位信息-----
    // 锁定身体/头部/夹爪关节 (仅解算手臂J1~J7)
    mdataSend.lArm_JntLim.ifUseJntLim = true;
    mdataSend.lArm_JntLim.limJntNum = 5;
    mdataSend.lArm_JntLim.limJntLabel.resize(mdataSend.lArm_JntLim.limJntNum);
    mdataSend.lArm_JntLim.limJntAng.resize(mdataSend.lArm_JntLim.limJntNum);
    mdataSend.lArm_JntLim.limJntLabel[0] = "body_joint_1";
    mdataSend.lArm_JntLim.limJntAng[0] = 0.0;
    mdataSend.lArm_JntLim.limJntLabel[1] = "body_joint_2";
    mdataSend.lArm_JntLim.limJntAng[1] = 0.0;
    mdataSend.lArm_JntLim.limJntLabel[2] = "head_joint_1";
    mdataSend.lArm_JntLim.limJntAng[2] = 0.0;
    mdataSend.lArm_JntLim.limJntLabel[3] = "head_joint_2";
    mdataSend.lArm_JntLim.limJntAng[3] = 0.0;
    mdataSend.lArm_JntLim.limJntLabel[4] = "arm_l_joint_8";
    mdataSend.lArm_JntLim.limJntAng[4] = 0.0;

    mdataSend.rArm_JntLim.ifUseJntLim = true;
    mdataSend.rArm_JntLim.limJntNum = 1;
    mdataSend.rArm_JntLim.limJntAng.resize(mdataSend.rArm_JntLim.limJntNum);
    mdataSend.rArm_JntLim.limJntLabel.resize(mdataSend.rArm_JntLim.limJntNum);
    mdataSend.rArm_JntLim.limJntLabel[0] = "arm_r_joint_8";
    mdataSend.rArm_JntLim.limJntAng[0] = 0.0;

    mdataSend.ifUpdateLim = false;

    //-----关节初始化信息-----
    mdataSend.ifReInit = false;
    mdataSend.lArm_stPos = lArm_stJntPos;
    mdataSend.rArm_stPos = rArm_stJntPos;

    mdataSend.ifDispJntInfo = false;
    mdataSend.ifUse_visualization = false;
    mdataSend.ifDisp_msgs = false;

    //求解器参数初始化

    mdataSend.optiWeight[0] = 50.0;
    mdataSend.optiWeight[1] = 1.0;
    mdataSend.optiWeight[2] = 0.002;
    mdataSend.optiWeight[3] = 0.01;
    mdataSend.optiWeight[4] = 0.0002;

    mdataSend.template_times = 50;
    mdataSend.errTol = 1e-6;

    mdataSend.ifPrintOptiLevel = 0;
    mdataSend.ifPrintOptiTime = false;

    mdataSend.ctlDesMsg = ctlStMsg;
    //-------------------------------------------------------
    // 轨迹数据处理包装初始化
    cartesian_traj_.init();
    JntSpace_traj_.init();
    //-------------------------------------------------------
}

void arm_traj_ctl::JntStabMod(std::vector<int> l_JntStab, std::vector<int> r_JntStab){
    for(int i = 0; i < SingleArmDof; i++){
        mdataSend.ifJntStab[0][i] = l_JntStab[i];
        mdataSend.ifJntStab[1][i] = r_JntStab[i];
    }
}

/*  数据处理与传输代码
--------------------
传输协议
输入内容：
    1. 重力补偿控制量 -- gravCompMod
        0 -- 无重力补偿；1 -- 有重力补偿，目标值作为输入量；2 -- 有重力补偿，反馈值作为输入量
    2. 关节限位控制量 -- JntStabMod
    3. 运动控制量 -- ctlTransMod
        0 -- 归零控制；1 -- 关节模式；2 -- 点位控制； 3 -- 关节轨迹控制； 4 -- 点位轨迹控制； 5 -- 零力拖动模式
    4. 目标力位 -- ctlDesMsg
    5. 力前馈开关量 -- ifUse_TorFeedForward
        默认值：0
        0 -- 不使用；1 -- 使用
*/
dataSendWrap arm_traj_ctl::trans_and_outp(
    int gravCompMod, 
    JntLimMsg_trans JntStabMod_l, 
    JntLimMsg_trans JntStabMod_r, 
    int ctlTransMod, 
    ctlDesMsg_trans ctlDesMsg, 
    bool ifReachEnd, 
    int ifUseTorFeedForward,
    int DescTrajMod
){
    
    //--------------------
    // 控制量赋值
    mdataSend.ctl_transMod = ctlTransMod;
    int runinMod;
    if(ctlTransMod == 3 || ctlTransMod == 4){
        runinMod = 1;
    }else{
        runinMod = 3;
    }
    mdataSend.runinMod = runinMod;
    
    //重力补偿：
    gravComCtlWrap temp_gravCompCtl;
    temp_gravCompCtl = mdataSend.GravCompCtlMsg;
    if(gravCompMod == 0){
        temp_gravCompCtl.ifUseGravComp = 0;
        temp_gravCompCtl.mIfUseFricComp = 0;
        temp_gravCompCtl.mIfUseInpFtr = 0;
        temp_gravCompCtl.mIfUseOutpFtr = 0;
        temp_gravCompCtl.GravCompInpMod = 0;
    }else if(gravCompMod == 2){
        temp_gravCompCtl.ifUseGravComp = 1;
        temp_gravCompCtl.mIfUseFricComp = 0;
        temp_gravCompCtl.mIfUseInpFtr = 1;
        temp_gravCompCtl.mIfUseOutpFtr = 0;
        temp_gravCompCtl.GravCompInpMod = 2;
    }else{
        temp_gravCompCtl.ifUseGravComp = 1;
        temp_gravCompCtl.mIfUseFricComp = 0;
        temp_gravCompCtl.mIfUseInpFtr = 1;
        temp_gravCompCtl.mIfUseOutpFtr = 0;
        temp_gravCompCtl.GravCompInpMod = 1;
    }

    //关节限位控制量
    JntLimMsgWrap temp_JntLimMsg_l = mdataSend.lArm_JntLim;
    JntLimMsgWrap temp_JntLimMsg_r = mdataSend.rArm_JntLim;

    temp_JntLimMsg_l.ifUseJntLim = JntStabMod_l.ifUseJntLim;
    temp_JntLimMsg_l.limJntNum = JntStabMod_l.limJntNum;
    temp_JntLimMsg_l.limJntAng = JntStabMod_l.limJntAng;
    temp_JntLimMsg_l.limJntLabel = JntID_to_label(JntStabMod_l.limJntID);

    temp_JntLimMsg_r.ifUseJntLim = JntStabMod_r.ifUseJntLim;
    temp_JntLimMsg_r.limJntNum = JntStabMod_r.limJntNum;
    temp_JntLimMsg_r.limJntAng = JntStabMod_r.limJntAng;
    temp_JntLimMsg_r.limJntLabel = JntID_to_label(JntStabMod_r.limJntID);

    //是否采用反馈值作为初始化
    mdataSend.ifUseRet = ctlDesMsg.ifInitUseRet;

    //单点控制指令刷新
    mdataSend.ifUseTemp = ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp;
    mdataSend.TrajTime.resize(2);
    mdataSend.TrajTime[0] = ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L;
    mdataSend.TrajTime[1] = ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R;
    mdataSend.accRateBase.resize(2);
    mdataSend.accRateBase[0] = ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L;
    mdataSend.accRateBase[1] = ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R;

    mdataSend.ifRedefineJntTrajStab = ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt;
    mdataSend.m_stabMsg.resize(2);
    mdataSend.m_stabMsg[0] = ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg;
    mdataSend.m_stabMsg[1] = ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg;
    
    //控制模式更新
    mdataSend.GravCompCtlMsg = temp_gravCompCtl;
    mdataSend.lArm_JntLim = temp_JntLimMsg_l;
    mdataSend.rArm_JntLim = temp_JntLimMsg_r;
    mdataSend.ifUpdateLim = JntStabMod_l.ifUseJntLim || JntStabMod_r.ifUseJntLim;
    //** 力控模式刷新尚未开放 **//

    //针对目标量进行处理
    ctlDesMsgWrap ctlDesMsg_processed;

    ctlDesMsg_processed.desPos_l_Descartes.assign(6, 0.0);
    ctlDesMsg_processed.desPos_r_Descartes.assign(6, 0.0);
    ctlDesMsg_processed.desPos_l_JointSpace.assign(7, 0.0);
    ctlDesMsg_processed.desPos_r_JointSpace.assign(7, 0.0);

    ctlDesMsg_processed.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_processed.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_processed.desTor_l_JointSpace.assign(7, 0.0);
    ctlDesMsg_processed.desTor_r_JointSpace.assign(7, 0.0);

    if(ctlTransMod == 2 || ctlTransMod == 4){
        //传输笛卡尔空间轨迹

        if(DescTrajMod == 2){

            for(int i = 0; i < 3; i++){
                cartesian_traj_.l_arm_end_delta[i] = ctlDesMsg.lArm_desMsg[i];
                cartesian_traj_.r_arm_end_delta[i] = ctlDesMsg.rArm_desMsg[i];
            }
            for(int i = 0; i < 4; i++){
                cartesian_traj_.l_arm_end_pos[i] = ctlDesMsg.lArm_desMsg[i + 3];
                cartesian_traj_.r_arm_end_pos[i] = ctlDesMsg.rArm_desMsg[i + 3];
            }

            int ctlMod = 0;
            if(ctlTransMod == 2){
                ctlMod = 0;
            }else{
                ctlMod = 1;
            }

            ctlDesMsg_processed = cartesian_traj_.trans_and_outp(ifReachEnd, ctlMod);
        }else{
            //输入量姿态采用欧拉角描述 -- 直接传输
            ctlDesMsg_processed.desPos_l_Descartes.assign(6, 0.0);
            ctlDesMsg_processed.desPos_r_Descartes.assign(6, 0.0);
            ctlDesMsg_processed.desPos_l_JointSpace.assign(SingleArmDof, 0.0);
            ctlDesMsg_processed.desPos_r_JointSpace.assign(SingleArmDof, 0.0);

            ctlDesMsg_processed.desTor_l_Descartes.assign(6, 0.0);
            ctlDesMsg_processed.desTor_r_Descartes.assign(6, 0.0);
            ctlDesMsg_processed.desTor_l_JointSpace.assign(SingleArmDof, 0.0);
            ctlDesMsg_processed.desTor_r_JointSpace.assign(SingleArmDof, 0.0);

            for(int i = 0; i < 6; i++){
                ctlDesMsg_processed.desPos_l_Descartes[i] = ctlDesMsg.lArm_desMsg[i];
                ctlDesMsg_processed.desPos_r_Descartes[i] = ctlDesMsg.rArm_desMsg[i];
            }
        }
    }else if(ctlTransMod == 1 || ctlTransMod == 3){
        for(int i = 0; i < 7; i++){
            JntSpace_traj_.l_arm_joint_angular[i] = ctlDesMsg.lArm_desMsg[i];
            JntSpace_traj_.r_arm_joint_angular[i] = ctlDesMsg.rArm_desMsg[i];
        }

        int ctlMod = 0;
        if(ctlTransMod == 1){
            ctlMod = 0;
        }else{
            ctlMod = 1;
        }

        ctlDesMsg_processed = JntSpace_traj_.trans_and_outp(ifReachEnd, ctlMod);

    }else{

    }

    mdataSend.ctlDesMsg = ctlDesMsg_processed;

    //更新关节刚度模式
    for(int i = 0; i < SingleArmDof; i++){
        mdataSend.rigidityCtl_mod_l[i] = ctlDesMsg.lArm_rigidity_mod[i];
        mdataSend.rigidityCtl_mod_r[i] = ctlDesMsg.rArm_rigidity_mod[i];
    }
    //更新PD参数
    for(int i = 0; i < SingleArmDof; i++){
        mdataSend.KpTrans_l[i] = ctlDesMsg.lArm_JntKp[i];
        mdataSend.KdTrans_l[i] = ctlDesMsg.lArm_JntKd[i];

        mdataSend.KpTrans_r[i] = ctlDesMsg.rArm_JntKp[i];
        mdataSend.KdTrans_r[i] = ctlDesMsg.rArm_JntKd[i];
    }

    return mdataSend;
}

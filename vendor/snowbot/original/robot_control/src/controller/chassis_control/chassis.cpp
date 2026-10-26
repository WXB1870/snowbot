#include "controller/chassis_control/chassis.h"



//----------辅助函数----------
int sgn(double x){
    if(x > 1e-4){
        return 1;
    }else if(x < -1 * 1e-4){
        return -1;
    }else{
        return 0;
    }
}

//----------底盘通讯代码----------

chassis_car::chassis_car(){

    //运动限幅参数配置
    max_linear_x_ = 0.5;      // 前后最大线速度 (m/s)
    max_linear_y_ = 0.5;      // 左右最大线速度 (m/s)
    max_angular_z_ = 1.0;     // 最大角速度 (rad/s)

    std::cout << " === 底盘通讯初始化完成 === " << std::endl;
}

void chassis_car::get_and_publish_(double linear_x_, double linear_y_, double angualr_z_){
    chassis_ctlMsg temp_derta_pos;

    if(std::abs(linear_x_) > max_linear_x_){
        temp_derta_pos.linear_x_ = sgn(linear_x_) * max_linear_x_;
    }else{
        temp_derta_pos.linear_x_ = linear_x_;
    }

    if(std::abs(linear_y_) > max_linear_y_){
        temp_derta_pos.linear_y_ = sgn(linear_y_) * max_linear_y_;
    }else{
        temp_derta_pos.linear_y_ = linear_y_;
    }

    if(std::abs(angualr_z_) > max_angular_z_){
        temp_derta_pos.angular_z_ = sgn(angualr_z_) * max_angular_z_;
    }else{
        temp_derta_pos.angular_z_ = angualr_z_;
    }


}
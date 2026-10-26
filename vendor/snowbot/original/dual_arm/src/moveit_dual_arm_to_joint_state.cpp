#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <dual_arm/JointCommand.h>
#include <vector>
#include <map>

class JointStateProcesser {
public:
    JointStateProcesser() {
        nh_ = ros::NodeHandle();
        sub_ = nh_.subscribe("/joint_states", 10, &JointStateProcesser::jointStateCallback, this);
        pub_ = nh_.advertise<dual_arm::JointCommand>("joint_commands", 10);
        
        // 初始化关节ID映射
        joint_id_map_ = {
            {"l_joint_1", 0},
            {"l_joint_2", 1},
            {"l_joint_3", 2},
            {"l_joint_4", 3},
            {"l_joint_5", 4},
            {"l_joint_6", 5},
            {"l_joint_7", 6}
             // 右臂关节
            {"r_joint_1", 7},
            {"r_joint_2", 8},
            {"r_joint_3", 9},
            {"r_joint_4", 10},
            {"r_joint_5", 11},
            {"r_joint_6", 12},
            {"r_joint_7", 13}

            // {"r_joint_1", 0},
            // {"r_joint_2", 1},
            // {"r_joint_3", 2},
            // {"r_joint_4", 3},
            // {"r_joint_5", 4},
            // {"r_joint_6", 5},
            // {"r_joint_7", 6}
            //  // 左臂关节
            // {"l_joint_1", 7},
            // {"l_joint_2", 8},
            // {"l_joint_3", 9},
            // {"l_joint_4", 10},
            // {"l_joint_5", 11},
            // {"l_joint_6", 12},
            // {"l_joint_7", 13}
        };
        
        // 初始化位置数组
        joint_positions_.resize(14, 0.0);
         // 初始化默认控制参数
        default_kp_ = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};  // 默认kp
        default_kd_ = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};                // 默认kd
        default_speed_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};             // 默认速度 (rad/s)
        default_torque_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};     // 默认力矩 (Nm)
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if(msg->name.empty() || msg->position.empty()) {
            ROS_WARN("No date");
            return;
        }
        dual_arm::JointCommand cmd_msg;

        // 初始化所有数据
        cmd_msg.position.resize(14);
        cmd_msg.kp.resize(14);
        cmd_msg.kd.resize(14);
        cmd_msg.speed.resize(14);
        cmd_msg.torque.resize(14);

        for (int i = 0; i < 14; i++) {
            cmd_msg.position[i] = 0.0f;
            cmd_msg.kp[i] = default_kp_[i];
            cmd_msg.kd[i] = default_kd_[i];
            cmd_msg.speed[i] = default_speed_[i];
            cmd_msg.torque[i] = default_torque_[i];
        }

        //   更新位置数据
         for (size_t i = 0; i < msg->name.size(); i++) {
             auto it = joint_id_map_.find(msg->name[i]);
             if (it != joint_id_map_.end()) {
                 int joint_idx = it->second;
                 if (joint_idx >= 0 && joint_idx < 14) {
                     cmd_msg.position[joint_idx] = msg->position[i];
                 }
             }
         }

        ROS_INFO("Pub Command:");
        // 打印右臂关节信息
        ROS_INFO("Right Arm:");
        for (int i = 7; i < 14; i++) {
            ROS_INFO("MotorID1%d: pos=%.3f, kp=%.1f, kd=%.1f, speed=%.2f, torque=%.2f",
                    i + 1, cmd_msg.position[i], cmd_msg.kp[i], cmd_msg.kd[i],
                    cmd_msg.speed[i], cmd_msg.torque[i]);
        }
        // 打印左臂关节信息
        ROS_INFO("Left Arm:");
        for (int i = 0; i < 7; i++) {
            ROS_INFO("MotorID2%d: pos=%.3f, kp=%.1f, kd=%.1f, speed=%.2f, torque=%.2f",
                    i + 1, cmd_msg.position[i], cmd_msg.kp[i], cmd_msg.kd[i],
                    cmd_msg.speed[i], cmd_msg.torque[i]);
        }
        pub_.publish(cmd_msg);
      } 
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::vector<double> joint_positions_; 
    std::map<std::string, int> joint_id_map_;
    std::vector<double> default_kp_;
    std::vector<double> default_kd_;
    std::vector<double> default_speed_;
    std::vector<double> default_torque_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_reader");
    JointStateProcesser node;
    ROS_INFO("Dual Arm Joint State Processor started");
    ros::Rate rate(1000); // 1kHz
    ros::spin();
    return 0;
}
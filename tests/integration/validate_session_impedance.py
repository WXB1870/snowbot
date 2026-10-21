#!/usr/bin/env python3
"""Compile actual impedance/controller/RT gate code with inert ROS and memory PDOs.

Requires a coherent Pinocchio/Eigen/urdfdom/yaml-cpp prefix. Never launches ROS,
opens a network interface or accesses a device. This is not a full ROS build.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


def write_shims(shim):
    def put(name,s):
     p=shim/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s)
    put('rclcpp/rclcpp.hpp',r'''#pragma once
    #include <memory>
    #include <string>
    #include <vector>
    #define RCLCPP_INFO(...) ((void)0)
    #define RCLCPP_WARN(...) ((void)0)
    #define RCLCPP_ERROR(...) ((void)0)
    #define RCLCPP_INFO_STREAM(...) ((void)0)
    #define RMW_QOS_POLICY_LIVELINESS_AUTOMATIC 1
    namespace rclcpp {
    struct QoS {explicit QoS(int){} QoS& best_effort(){return *this;}QoS& durability_volatile(){return *this;}QoS& liveliness(int){return *this;}};
    template<class T>struct Subscription {using SharedPtr=std::shared_ptr<Subscription<T>>;};
    template<class T>struct Publisher {using SharedPtr=std::shared_ptr<Publisher<T>>;T last{};void publish(const T& v){last=v;}};
    struct Node {using SharedPtr=std::shared_ptr<Node>;unsigned subscriptions=0;
    int get_logger(){return 0;}
    template<class T>typename Publisher<T>::SharedPtr create_publisher(const std::string&,QoS){return std::make_shared<Publisher<T>>();}
    template<class T,class F>typename Subscription<T>::SharedPtr create_subscription(const std::string&,QoS,F){++subscriptions;return std::make_shared<Subscription<T>>();}
    };}
    ''')
    put('rclcpp/subscription.hpp','#include "rclcpp/rclcpp.hpp"\n')
    put('ament_index_cpp/get_package_share_directory.hpp','#pragma once\n#include <string>\nnamespace ament_index_cpp {inline std::string get_package_share_directory(const std::string&){return "";}}\n')
    put('snowbot_ctl_msgs/msg/cartesian_impedance.hpp',r'''#pragma once
    #include <array>
    #include <memory>
    namespace snowbot_ctl_msgs::msg {
    struct Vector {double x=0,y=0,z=0;};struct Quaternion {double x=0,y=0,z=0,w=1;};
    struct Motion {Vector linear,angular;};}
    namespace geometry_msgs::msg {using Twist=snowbot_ctl_msgs::msg::Motion;using Accel=Twist;}
    namespace snowbot_ctl_msgs::msg {
    struct CartesianImpedance {using SharedPtr=std::shared_ptr<CartesianImpedance>;struct {Vector position;Quaternion orientation;}pose;Motion pose_twist,pose_accel;std::array<double,7>q_d_nullspace{};};}
    ''')
    put('geometry_msgs/msg/wrench.hpp','#pragma once\nnamespace geometry_msgs::msg {struct Wrench {}; }\n')
    put('std_msgs/msg/float64_multi_array.hpp','#pragma once\n#include <vector>\nnamespace std_msgs::msg {struct Float64MultiArray {std::vector<double> data;};}\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--prefix',type=Path,required=True)
    args=parser.parse_args()
    source=args.source.resolve();prefix=args.prefix.resolve()
    tests=Path(__file__).resolve().parents[1]/'native'
    compiler=shutil.which('c++')
    if not compiler:raise RuntimeError('C++ compiler unavailable')
    with tempfile.TemporaryDirectory(prefix='impedance-offline-') as temporary:
        work=Path(temporary);shim=work/'shim';write_shims(shim)
        flags=[compiler,'-std=c++17','-O0','-DPINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION',
            '-DPINOCCHIO_WITH_URDFDOM','-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS',
            '-DBOOST_MPL_LIMIT_LIST_SIZE=30','-DBOOST_MPL_LIMIT_VECTOR_SIZE=30',
            '-DBOOST_FUSION_INVOKE_MAX_ARITY=12']
        for path in (shim,source/'servo_control/include',prefix/'include',prefix/'include/eigen3',
                     prefix/'include/pinocchio/deprecated',prefix/'include/urdfdom_headers'):
            flags+=['-I',str(path)]
        controller=source/'servo_control/src/impedance/cartesian_impedance_controller.cpp'
        obj=work/'controller.o'
        def run(command):
            result=subprocess.run(command,capture_output=True,text=True,timeout=120)
            if result.returncode:raise RuntimeError(result.stdout+result.stderr)
            return result.stdout
        driver=(source/'servo_control/src/servo_control/ethercat_driver.cpp').read_text()
        start=driver.index('    obs.sensor_valid_mask=0;')
        sensor=driver[start:driver.index('    obs.session=session_watchdog_.session;',start)]
        start=driver.index('    uint32_t position_written=0;')
        output=driver[start:driver.index('    // --- PDO 收发 ---',start)]
        rt=(tests/'test_impedance_rt.cpp').read_text().replace('// SENSOR_CAPTURE',sensor).replace('// FINAL_PDO_WRITE',output)
        rt_path=work/'rt.cpp';rt_path.write_text(rt)
        run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I'+str(source/'servo_control/include'),str(rt_path),'-o',str(work/'rt')])
        print(run([str(work/'rt')]).strip())
        run(flags+['-c',str(controller),'-o',str(obj)])
        servo=(source/'servo_control/src/ros_controller/rosLoop.cpp').read_text()
        start=servo.index('void ServoNode::position_step()')
        end=servo.index('\nbool ServoNode::position_ready()',start)
        methods=servo[start:end]+servo[servo.index('bool ServoNode::session_impedance_ready() const'):]
        for name in ('test_impedance_nominal','test_session_impedance'):
            text=(tests/(name+'.cpp')).read_text().replace('// PRODUCTION_METHODS',methods)
            path=work/(name+'.cpp');path.write_text(text);binary=work/name
            run(flags+[str(path),str(obj),'-L'+str(prefix/'lib'),'-Wl,-rpath,'+str(prefix/'lib'),
                       '-lpinocchio_default','-lpinocchio_parsers','-lyaml-cpp','-o',str(binary)])
            print(run([str(binary),str(source/'dual_arm/urdf/snowbot.urdf')]).strip())


if __name__=='__main__':main()

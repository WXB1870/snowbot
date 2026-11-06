#!/usr/bin/env python3
"""Compile actual impedance/controller/RT gate code with inert ROS and memory PDOs.

Requires a coherent Pinocchio/Eigen/urdfdom/yaml-cpp prefix. Never launches ROS,
opens a network interface or accesses a device. This is not a full ROS build.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import time
from validate_position_native import TARGET_FIELDS


def write_changed(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text()!=text:
        path.write_text(text)


def compile_cached(command, work, compiler_identity):
    """Let make track compiler depfiles; keep successful objects after a retry."""
    output=Path(command[command.index('-o')+1])
    depfile=work/(output.name+'.d')
    environment={key:os.environ.get(key) for key in
        ('CPATH','CPLUS_INCLUDE_PATH','C_INCLUDE_PATH','LIBRARY_PATH','SDKROOT','MACOSX_DEPLOYMENT_TARGET')}
    makefile=work/(output.name+'.mk')
    dependencies=[makefile,*(Path(p) for p in command[1:command.index('-o')]
                          if p.endswith(('.cpp','.o')))]
    for arg in command:
        if arg.startswith('-L'):
            for library in (p[2:] for p in command if p.startswith('-l')):
                dependencies.extend(Path(arg[2:]).glob('lib'+library+'.so*'))
                dependencies.extend(Path(arg[2:]).glob('lib'+library+'*.dylib'))
    def escape(path):
        return str(path).replace('$','$$').replace('#','\\#').replace(' ','\\ ')
    identity=json.dumps(dict(compiler=compiler_identity,environment=environment),sort_keys=True)
    write_changed(makefile,'# '+identity+'\n.DELETE_ON_ERROR:\n'+escape(output)+': '+' '.join(map(escape,dependencies))+
        '\n\t@echo COMPILE '+shlex.quote(output.name)+'\n\t@'+
        shlex.join([*command,'-MMD','-MF',str(depfile)])+'\n-include '+escape(depfile)+'\n')
    started=time.monotonic()
    result=subprocess.run(['make','--no-print-directory','-f',str(makefile),str(output)],
                          capture_output=True,text=True,timeout=300)
    if result.returncode:raise RuntimeError(result.stdout+result.stderr)
    print(result.stdout.strip()+f' elapsed={time.monotonic()-started:.1f}s',flush=True)


def write_shims(shim):
    def put(name,s):
     write_changed(shim/name,s)
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
    template<class T>struct Publisher {using SharedPtr=std::shared_ptr<Publisher<T>>;inline static unsigned calls=0;T last{};void publish(const T& v){last=v;++calls;}};
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
    parser.add_argument('--require-production-poll',action='store_true',help='Require the native fixture to embed the actual production position_poll body')
    parser.add_argument('--case',choices=('all','rt','nominal','session'),default='all',help='Select the directly affected existing native check')
    parser.add_argument('--physics-controller-output',type=Path,help='Build the simulation-only stdio torque bridge instead of running fixtures')
    parser.add_argument('--build-dir',type=Path,help='Persistent native build directory; defaults to SOURCE/../validation-impedance')
    args=parser.parse_args()
    source=args.source.resolve();prefix=args.prefix.resolve()
    tests=Path(__file__).resolve().parents[1]/'native'
    model=source/'dual_arm/urdf/snowbot.urdf'
    if not args.physics_controller_output and args.case!='rt' and not model.is_file():
        raise FileNotFoundError(model)
    work=(args.build_dir or source.parent/'validation-impedance').resolve()
    work.mkdir(parents=True,exist_ok=True)
    compiler=shutil.which('c++')
    if not compiler:raise RuntimeError('C++ compiler unavailable')
    native_include=source/'servo_control/include/bindu_execution_native'
    native_sources=[]
    if (source/'servo_control/include/position_trajectory.h').is_file():
        native_sources=[source/'servo_control/src/bindu_execution_native'/name
                        for name in ('interpolation.cpp','adaptive_stream.cpp')]
        required=[*(native_include/name for name in
                    ('interpolation.hpp','adaptive_stream.hpp','curve_math.hpp')),*native_sources]
        missing=[str(path) for path in required if not path.is_file()]
        if missing:raise RuntimeError('CANDIDATE_NATIVE_SOURCE_MISSING:'+','.join(missing))
        expected=json.loads((source.parent/'position_release.json').read_text()).get('native_execution',{})
        if any(expected.get(path.name)!=hashlib.sha256(path.read_bytes()).hexdigest() for path in required):
            raise RuntimeError('CANDIDATE_NATIVE_SHA256_MISMATCH')
    compiler_identity=subprocess.check_output([compiler,'--version'],text=True)
    shim=work/'shim';write_shims(shim)
    # The target worker has a fixed preparation deadline; use the same
    # optimization level as the existing Release deployment builds.
    flags=[compiler,'-std=c++17','-O3','-DPINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION',
        '-DPINOCCHIO_WITH_URDFDOM','-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS',
        '-DBOOST_MPL_LIMIT_LIST_SIZE=30','-DBOOST_MPL_LIMIT_VECTOR_SIZE=30',
        '-DBOOST_FUSION_INVOKE_MAX_ARITY=12']
    for path in (shim,source/'servo_control/include',native_include,prefix/'include',prefix/'include/eigen3',Path('/usr/include/eigen3'),
                 prefix/'include/pinocchio/deprecated',prefix/'include/urdfdom_headers'):
        flags+=['-I',str(path)]
    controller=source/'servo_control/src/impedance/cartesian_impedance_controller.cpp'
    obj=work/'controller.o'
    def run(command):
        if command[0]==compiler:
            compile_cached(command,work,compiler_identity)
            return ''
        result=subprocess.run(command,capture_output=True,text=True,
                              timeout=120)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        return result.stdout
    if not args.physics_controller_output and args.case in ('all','rt'):
        driver=(source/'servo_control/src/servo_control/ethercat_driver.cpp').read_text()
        start=driver.index('    obs.sensor_valid_mask=0;')
        sensor=driver[start:driver.index('    obs.shutdown_disabled=',start)]
        start=driver.index('    uint32_t position_written=0;')
        output=driver[start:driver.index('    // --- PDO 收发 ---',start)]
        rt=(tests/'test_impedance_rt.cpp').read_text().replace('// SENSOR_CAPTURE',sensor).replace('// FINAL_PDO_WRITE',output)
        rt_path=work/'rt.cpp';write_changed(rt_path,rt)
        run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I'+str(source/'servo_control/include'),'-I'+str(native_include),str(rt_path),'-o',str(work/'rt')])
        print(run([str(work/'rt')]).strip())
    if args.case=='rt' and not args.physics_controller_output:return
    # Prepare the selected fixture before compiling the expensive controller.
    if args.physics_controller_output:
        run(flags+['-c',str(controller),'-o',str(obj)])
        binary=args.physics_controller_output.resolve()
        binary.parent.mkdir(parents=True,exist_ok=True)
        run(flags+[str(tests/'impedance_physics.cpp'),str(obj),'-L'+str(prefix/'lib'),
                   '-Wl,-rpath,'+str(prefix/'lib'),'-lpinocchio_default','-lpinocchio_parsers',
                   '-lyaml-cpp','-o',str(binary)])
        print('Physics controller built: '+str(binary))
        return
    objects=[obj]
    servo=(source/'servo_control/src/ros_controller/rosLoop.cpp').read_text()
    start=servo.index('void ServoNode::position_step()')
    end=servo.index('\nbool ServoNode::position_ready()',start)
    methods=servo[start:end]+servo[servo.index('bool ServoNode::session_impedance_ready() const'):]
    names={'all':('test_impedance_nominal','test_session_impedance'),'nominal':('test_impedance_nominal',),'session':('test_session_impedance',)}[args.case]
    prepared=[]
    for name in names:
        text=(tests/(name+'.cpp')).read_text()
        if name=='test_session_impedance':
            text=text.replace('\n};\n\n// PRODUCTION_POLL_METHODS',TARGET_FIELDS+'\n};\n\n// PRODUCTION_POLL_METHODS',1)
        poll_fixture='// PRODUCTION_POLL_METHODS' in text
        if name=='test_session_impedance' and args.require_production_poll and not poll_fixture:
            raise RuntimeError('PRODUCTION_POLL_FIXTURE_REQUIRED')
        text=text.replace('// PRODUCTION_METHODS',methods)
        if poll_fixture:
            poll_start=servo.index('void ServoNode::position_interrupt()')
            poll_methods=servo[poll_start:servo.index('void ServoNode::position_step()',poll_start)]
            poll_methods=poll_methods.replace('snowbot_ctl_msgs::msg::PositionSessionServo','Packet')
            text=text.replace('// PRODUCTION_POLL_METHODS',poll_methods)
            print('Production position_poll SHA256 '+hashlib.sha256(poll_methods.encode()).hexdigest())
        path=work/(name+'.cpp');write_changed(path,text)
        prepared.append(path)
    run(flags+['-c',str(controller),'-o',str(obj)])
    for path in native_sources:
        native_obj=work/(path.stem+'.o')
        run(flags+['-c',str(path),'-o',str(native_obj)])
        objects.append(native_obj)
    for path in prepared:
        binary=work/path.stem
        run(flags+[str(path),*map(str,objects),'-L'+str(prefix/'lib'),'-Wl,-rpath,'+str(prefix/'lib'),
                   '-lpinocchio_default','-lpinocchio_parsers','-lyaml-cpp','-o',str(binary)])
        print(run([str(binary),str(model)]).strip())


if __name__=='__main__':main()

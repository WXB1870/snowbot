"""Offline G8 connection kernel, executor shim and actual position-session callbacks."""
from pathlib import Path
import json
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from tools.deployment import prepare_snowbot_connection as vendor
ROOT = Path(__file__).resolve().parents[2]


def compile_run(code, includes=()):
    compiler=shutil.which('c++')
    if not compiler:raise unittest.SkipTest('C++ compiler unavailable')
    with tempfile.TemporaryDirectory() as tmp:
        path=Path(tmp);(path/'check.cpp').write_text(code)
        result=subprocess.run([compiler,'-std=c++17','-pthread','-Wall','-Wextra','-Werror',
            '-I'+str(vendor.ASSETS),*['-I'+str(p) for p in includes],str(path/'check.cpp'),'-o',str(path/'check')],capture_output=True,text=True)
        if result.returncode:raise AssertionError(result.stderr)
        result=subprocess.run([str(path/'check')],capture_output=True,text=True,timeout=10)
        if result.returncode:raise AssertionError(result.stderr)
        return result.stdout


def cpp_messages(source):
    names=['JointPositionCommand','MidNodeCtlMsgSend','PositionObservation','PositionSessionMeta',
           'PositionSessionCommand','PositionSessionUpper','PositionSessionState']
    types={'bool':'bool','string':'std::string','float64':'double','float32':'float',
           'int64':'int64_t','int32':'int32_t','int16':'int16_t','uint64':'uint64_t','uint32':'uint32_t','uint16':'uint16_t','uint8':'uint8_t'}
    result=['namespace snowbot_ctl_msgs { namespace msg {']
    for name in names:
        result.append('struct '+name+' { using SharedPtr=std::shared_ptr<'+name+'>;')
        for line in (source/'ctl_msgs/msg'/(name+'.msg')).read_text().splitlines():
            line=line.split('#')[0].strip()
            if not line:continue
            typename,field=line.split()
            if '=' in field:continue
            base,length=re.fullmatch(r'([^\[]+)(?:\[(\d*)\])?',typename).groups()
            cpp=types.get(base,base.split('/')[-1])
            if base=='std_msgs/Header':cpp='Header'
            if '[' in typename:cpp='std::array<'+cpp+','+length+'>' if length else 'std::vector<'+cpp+'>'
            result.append(cpp+' '+field+'{};')
        result.append('};')
    return '\n'.join(result+['}}'])


def method(source,name):
    a=source.index('void '+name+'(');b=source.index('{',a);level=1;j=b+1
    while level:level+=(source[j]=='{')-(source[j]=='}');j+=1
    return source[a:j]


class VendorConnectionTests(unittest.TestCase):
    def source(self):
        source=Path(os.environ.get('SNOWBOT_POSITION_SOURCE',ROOT/'artifacts/position-connection-20261021/upstream-src'))
        if not source.is_dir():self.skipTest('reviewed position-session snapshot not distributed in Git')
        return source

    def test_native_state_sequences_and_deadlines(self):
        self.assertIn('connection checks passed',compile_run((ROOT/'tests/native/test_snowbot_connection.cpp').read_text()))

    def test_patch_preserves_upstream_servo_native_abi_and_rejects_other_sources(self):
        source=self.source()
        with tempfile.TemporaryDirectory() as tmp:
            output=vendor.prepare(source,Path(tmp)/'copy')
            manifest=json.loads((output/'connection_release.json').read_text())
            self.assertTrue(manifest['native_messages_unchanged']);self.assertTrue(manifest['servo_sources_unchanged'])
            self.assertFalse(manifest['full_driver_build_verified']);self.assertFalse(manifest['hardware_started'])
            for original in source.rglob('*'):
                relative=original.relative_to(source)
                if original.is_file() and (relative.parts[0]=='servo_control' or relative.name in (
                    'RetInformation.msg','JointPositionCommand.msg','CtlInformation.msg','MidNodeCtlMsgSend.msg',
                    'MidNodeCtlMsgRet.msg','position_session_contract.h')):
                    self.assertEqual(original.read_bytes(),(output/'src'/relative).read_bytes())
            with self.assertRaisesRegex(ValueError,'OUTPUT_EXISTS'):vendor.prepare(source,output)
            with self.assertRaisesRegex(ValueError,'OVERLAP'):vendor.prepare(source,source/'bad')
            changed=Path(tmp)/'source';shutil.copytree(source,changed)
            (changed/'robot_control/src/robot_control/msgPublisher.cpp').write_text('colleague changes\n')
            with self.assertRaisesRegex(ValueError,'BASELINE_MISMATCH'):vendor.prepare(changed,Path(tmp)/'bad')
            self.assertFalse((Path(tmp)/'bad').exists())

    def test_actual_shared_position_session_callbacks_compile_and_reconnect(self):
        with tempfile.TemporaryDirectory() as tmp:
            src=vendor.prepare(self.source(),Path(tmp)/'copy')/'src'
            bodies=[]
            for cls,pkg,file in [('RobotCtlNode','robot_control','robot_control/msgPublisher.cpp'),
                                 ('ArmCtlNode','dual_arm_control','dual_arm_control/msgPublish.cpp')]:
                code=(src/pkg/'src'/file).read_text()
                bodies.extend(method(code,cls+'::'+name) for name in ('connection_peer','connection_observe','connection_refresh','position_callback'))
            code=CALLBACK_PREAMBLE+cpp_messages(src)+CALLBACK_NODES+'\n'.join(bodies)+CALLBACK_TEST
            self.assertIn('actual shared callbacks passed',compile_run(code,[src/'robot_control/include']))


CALLBACK_PREAMBLE=r'''
#include "connection_state.h"
#include "position_session_contract.h"
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
static double ros_now=10;
namespace rclcpp {
struct Time {
 int32_t sec=0;uint32_t nanosec=0;
 Time(double t=0){sec=int32_t(t);nanosec=uint32_t(std::llround((t-sec)*1e9));if(nanosec>=1000000000){++sec;nanosec-=1000000000;}}
 double seconds()const{return sec+nanosec/1e9;}int64_t nanoseconds()const{return int64_t(sec)*1000000000+nanosec;}
};}
struct Header {rclcpp::Time stamp;};
namespace snowbot_connection {inline double steady_now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}}
struct Sub {const char* get_topic_name(){return "topic";}};
struct Reporter {
 snowbot_connection::Connection connection;uint64_t instance=42;std::string rejection;
 struct Endpoint {std::string reason,peer;};
 Reporter(){connection.initialize();}
 template<class T>Endpoint inspect(T,const std::string&,const std::string& = ""){return {"","gid"};}
};
struct Base {rclcpp::Time now(){return rclcpp::Time(ros_now);}};
'''
CALLBACK_NODES=r'''
using namespace snowbot_ctl_msgs::msg;
struct Capture {PositionSessionState latest;void publish(const PositionSessionState& msg){latest=msg;}};
struct Common : Base {
 std::unique_ptr<Reporter> connection_state_=std::make_unique<Reporter>();
 std::shared_ptr<Sub> connection_peer_sub_=std::make_shared<Sub>(),position_feedback_sub_=std::make_shared<Sub>();
 std::shared_ptr<Capture> position_state_pub_=std::make_shared<Capture>();
 uint64_t connection_boot_=0,connection_peer_instance_=0,connection_peer_epoch_=0;
 double connection_peer_received_=0;std::string connection_peer_reason_="peer_state_not_received";
 position_session::Gate position_gate_;position_session::Tracker position_feedback_;
 PositionSessionMeta position_meta_;bool position_pending_=false,position_dispatch_=false;
 std::string position_rejection_;
};
struct RobotCtlNode : Common {
 enum class ControlSource {Topic,Other};ControlSource control_source_=ControlSource::Topic;
 bool ifInitTemp=false,ifForceToRetZero=false,delivered=false;
 void joint_position_cmd_callback(JointPositionCommand::SharedPtr){delivered=true;}
 void connection_peer(PositionSessionState::SharedPtr);void connection_observe(PositionObservation::SharedPtr);
 void connection_refresh(bool publish=true);void position_callback(PositionSessionCommand::SharedPtr);
};
struct ArmCtlNode : Common {
 bool ifInitTemp=false,checkTemp=false,delivered=false;
 void upperMonitor_msg_callback(MidNodeCtlMsgSend::SharedPtr){delivered=true;}
 void connection_peer(PositionSessionState::SharedPtr);void connection_observe(PositionObservation::SharedPtr);
 void connection_refresh(bool publish=true);void position_callback(PositionSessionUpper::SharedPtr);
};
'''
CALLBACK_TEST=r'''
int main(){
 using namespace snowbot_connection;
 RobotCtlNode upper;ArmCtlNode middle;
 middle.connection_refresh();assert(!middle.position_state_pub_->latest.feedback_valid);
 auto peer=std::make_shared<PositionSessionState>();peer->header.stamp=rclcpp::Time(ros_now);
 peer->layer="servo";peer->control_source="position";peer->startup_complete=true;peer->stop_policy="disable_no_load";
 middle.connection_peer(peer);
 auto m=std::make_shared<PositionObservation>();m->boot_id=99;m->pdo_age=.001;m->wkc=m->expected_wkc=67;m->valid_mask=0xfffff;
 m->position.fill(.4);m->speed.fill(.0);m->current.fill(.1);
 for(unsigned i=1;i<6;++i){
  m->pdo_sample=i;m->header.stamp=rclcpp::Time(ros_now);middle.connection_observe(m);middle.connection_refresh();
  upper.connection_peer(std::make_shared<PositionSessionState>(middle.position_state_pub_->latest));
  upper.connection_observe(m);ros_now+=.001;
 }
 assert(middle.connection_state_->connection.feedback_fresh(ros_now,steady_now()));
 assert(upper.connection_state_->connection.feedback_fresh(ros_now,steady_now()));
 assert(!upper.delivered && !middle.delivered);
 auto target=std::make_shared<PositionSessionCommand>();target->command.l_arm_position.fill(.5);target->command.r_arm_position.fill(.5);
 target->meta.session=1;target->meta.sequence=1;target->meta.operation=1;target->meta.source_stamp_ns=upper.now().nanoseconds();target->meta.expires_at_ns=target->meta.source_stamp_ns+200000000;
 upper.position_callback(target);assert(upper.delivered);
 auto packet=std::make_shared<PositionSessionUpper>();packet->meta=upper.position_meta_;
 packet->command.if_streaming_joint_position=true;packet->command.ctl_trans_mod=3;
 middle.position_callback(packet);assert(middle.delivered);
 ArmCtlNode restarted;restarted.connection_state_->instance=43;
 for(unsigned i=6;i<11;++i){
  peer->header.stamp=rclcpp::Time(ros_now);restarted.connection_peer(peer);
  m->pdo_sample=i;m->header.stamp=rclcpp::Time(ros_now);restarted.connection_observe(m);restarted.connection_refresh();
  upper.connection_peer(std::make_shared<PositionSessionState>(restarted.position_state_pub_->latest));upper.connection_observe(m);ros_now+=.001;
 }
 restarted.position_callback(packet);assert(!restarted.delivered);
 target->meta.sequence=2;target->meta.source_stamp_ns=upper.now().nanoseconds();target->meta.expires_at_ns=target->meta.source_stamp_ns+200000000;
 upper.position_callback(target);assert(upper.position_rejection_=="new_session_required");
 std::this_thread::sleep_for(std::chrono::milliseconds(210));
 for(unsigned i=11;i<16;++i){
  peer->header.stamp=rclcpp::Time(ros_now);restarted.connection_peer(peer);
  m->pdo_sample=i;m->header.stamp=rclcpp::Time(ros_now);restarted.connection_observe(m);restarted.connection_refresh();
  upper.connection_peer(std::make_shared<PositionSessionState>(restarted.position_state_pub_->latest));upper.connection_observe(m);ros_now+=.001;
 }
 target->meta.session=2;target->meta.sequence=1;target->meta.source_stamp_ns=upper.now().nanoseconds();target->meta.expires_at_ns=target->meta.source_stamp_ns+200000000;
 upper.position_callback(target);assert(upper.position_rejection_.empty());
 packet->meta=upper.position_meta_;restarted.position_callback(packet);assert(restarted.delivered);
 std::cout<<"actual shared callbacks passed\n";
}
'''
# A tiny executor/graph shim exercises the production ROS adapter during slow
# initialization. This is a compile/run test of connection_ros.h, not a DDS test.
ROS_SHIM=r'''
#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
namespace std_msgs { namespace msg {struct String {std::string data;};}}
namespace rclcpp {
struct Time {double seconds()const{return 10.;}};
enum class QoSCompatibility {Ok,Warning,Error};
struct QoS {bool compatible=true;};
struct Check {QoSCompatibility compatibility;};
inline Check qos_check_compatible(QoS pub,QoS sub){return {pub.compatible && sub.compatible ? QoSCompatibility::Ok : QoSCompatibility::Error};}
struct TopicEndpointInfo {
 struct Hash {unsigned version=1;unsigned char value[32]{};};
 std::string ns="/",node="middle",type="Expected";QoS qos;Hash hash;
 Hash topic_type_hash()const{return hash;}
 std::string node_namespace()const{return ns;}std::string node_name()const{return node;}
 std::string topic_type()const{return type;}QoS qos_profile()const{return qos;}
 std::array<unsigned char,24> endpoint_gid()const{return {};}
};
struct TimerBase {using SharedPtr=std::shared_ptr<TimerBase>;std::function<void()> callback;};
template<class T> struct Publisher {using SharedPtr=std::shared_ptr<Publisher<T>>;std::function<void(const T&)> sink;void publish(const T& msg){sink(msg);}};
class Node {
public:
 std::atomic<unsigned> received{0},published{0};std::string last;
 std::vector<TimerBase::SharedPtr> timers;std::vector<TopicEndpointInfo> infos;
 Time now(){return {};} int get_logger(){return 0;}
 const char* get_name(){return "self";}const char* get_namespace(){return "/";}
 template<class T>T declare_parameter(const std::string&,T value){return value;}
 template<class T>typename Publisher<T>::SharedPtr create_publisher(const std::string&,int){
  auto pub=std::make_shared<Publisher<T>>();pub->sink=[this](const T& msg){++published;last=msg.data;};return pub;
 }
 template<class D,class F>TimerBase::SharedPtr create_wall_timer(D,F callback){auto timer=std::make_shared<TimerBase>();timer->callback=callback;timers.push_back(timer);return timer;}
 Node* get_node_base_interface(){return this;}
 std::vector<TopicEndpointInfo> get_publishers_info_by_topic(const std::string&){return infos;}
 std::vector<TopicEndpointInfo> get_subscriptions_info_by_topic(const std::string&){TopicEndpointInfo own;own.node="self";return {own};}
};
inline bool ok(){return true;}
inline void spin_some(Node* n){++n->received;for(auto& timer:n->timers)timer->callback();}
}
#define RCLCPP_INFO(...) ((void)0)
'''
ROS_TEST=r'''
#include "connection_ros.h"
#include <cassert>
#include <iostream>
struct Sub { const char* get_topic_name(){return "topic";}rclcpp::QoS get_actual_qos(){return {};}};
int main(){
    rclcpp::Node node;
    snowbot_connection::Reporter report(node,"middle","/middle");
    const unsigned initial=node.published.load();
    snowbot_connection::initialize(node,report,[&](){
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
        while(node.published.load()<initial+20 && std::chrono::steady_clock::now()<deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if(node.published.load()<initial+20)throw std::runtime_error("callbacks_blocked");
        throw std::runtime_error("model_missing: /no/model.urdf");
    });
    assert(node.received>=20 && node.published>=initial+20);
    assert(node.last.find("initialization_failed")!=std::string::npos);
    assert(node.last.find("/no/model.urdf")!=std::string::npos);
    assert(node.last.find("尚无反馈")!=std::string::npos);
    const auto previous=node.received.load();rclcpp::spin_some(&node);assert(node.received>previous);
    rclcpp::Node graph_node; snowbot_connection::Reporter graph(graph_node,"upper","/middle");
    graph.connection.initialize();auto sub=std::make_shared<Sub>();
    graph.graph(sub,"Expected");assert(graph.connection.status(10,100).reason=="publisher_missing");
    graph_node.infos.push_back({});graph_node.infos[0].type="Wrong";
    graph.graph(sub,"Expected");assert(graph.connection.status(10,100).reason=="message_type_mismatch");
    graph_node.infos[0].type="Expected";graph_node.infos[0].qos.compatible=false;
    graph.graph(sub,"Expected");assert(graph.connection.status(10,100).reason=="qos_incompatible_or_unknown");
    graph_node.infos[0].qos.compatible=true;graph.graph(sub,"Expected");
    assert(graph.connection.status(10,100).reason=="publisher_found_no_message");
    graph_node.infos[0].hash.value[0]=1;graph.graph(sub,"Expected");
    assert(graph.connection.status(10,100).reason=="message_type_hash_mismatch");
    graph_node.infos[0].hash.version=0;graph.graph(sub,"Expected");
    assert(graph.connection.status(10,100).reason=="message_type_hash_unavailable");
    std::cout<<"initialization communication and graph checks passed\n";
}
'''

class VendorConnectionRosAdapterTests(unittest.TestCase):
    def test_executor_remains_active_on_slow_missing_model_and_graph_diagnoses(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)
            for relative in ('rclcpp/rclcpp.hpp','rclcpp/qos.hpp','std_msgs/msg/string.hpp'):
                p=path/relative;p.parent.mkdir(parents=True,exist_ok=True)
                p.write_text(ROS_SHIM if relative=='rclcpp/rclcpp.hpp' else '#include <rclcpp/rclcpp.hpp>\n')
            (path/'bindu_connection_state.h').write_bytes((vendor.ASSETS/'connection_state.h').read_bytes())
            self.assertIn('initialization communication and graph checks passed',compile_run(ROS_TEST,[path]))

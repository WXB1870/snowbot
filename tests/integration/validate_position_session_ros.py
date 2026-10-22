#!/usr/bin/env python3
"""Offline production upper/middle relay test. Never starts servo or accesses a device."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--bindu-root',type=Path,help='Optional Bindu checkout with the installed session preflight')
    parser.add_argument('--position-protocol',choices=('session_v1','session_v3'),default='session_v1')
    parser.add_argument('--execution-mode',choices=('position','impedance'),default='position',help='Synthetic servo state/observation mode; never starts an impedance controller')
    parser.add_argument('--preflight-discovery-timeout',type=float,default=2.,help='New DDS observer discovery window; all feedback and contract freshness checks remain unchanged')
    args=parser.parse_args()
    if not 0 < args.preflight_discovery_timeout <= 10:parser.error('Require 0 < preflight discovery timeout <= 10 seconds')
    target_stream=args.position_protocol=='session_v3'
    if target_stream and not args.bindu_root:parser.error('session_v3 requires --bindu-root for the existing profile')
    if args.execution_mode!='position':parser.error('Target-stream relay validation supports pure position only')
    if int(os.environ.get('ROS_DOMAIN_ID','0'))<100 or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Require an isolated ROS domain >=100 and localhost-only transport')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message,deserialize_message
    from snowbot_ctl_msgs.msg import (PositionObservation,PositionSessionCommand,PositionSessionUpper,
        PositionSessionServo,PositionSessionState,RetInformation)
    if target_stream:
        from snowbot_ctl_msgs.msg import PositionTrajectory
        from bindu_contracts.profile import Profile
        from bindu_hardware.snowbot_session import CONTROL_NAMES
        profile=Profile.load(args.bindu_root/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_vendor_position.json')
        from tools.teleop.snowbot_pdo import observation_payload
    rclpy.init()
    observer=rclpy.create_node('position_session_validation')
    feedback=rclpy.create_node('servo_node') # Test fixture only, not the servo executable.
    obs_pub=feedback.create_publisher(PositionObservation,'/bindu/position/observation',qos_profile_sensor_data)
    ret_pub=feedback.create_publisher(RetInformation,'/dualArmCtl_fdback',10)
    servo_state_pub=feedback.create_publisher(PositionSessionState,'/bindu/position/servo_state',1)
    cmd_pub=observer.create_publisher(PositionSessionCommand,'/bindu/position/command',1)
    states={};frames=[];children=[];logs=[];sample=0;frozen=False;freeze_started=0.;launched={};mapped=set();launch_stamps={}
    q=[.1]*20;q[9]=-.1;q[7]=q[15]=0.;q[16:]=[0.,0.,.03,-.04]
    health_probes=[]
    def remember(layer,msg):
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if stamp<launch_stamps.get(layer,0):return
        states[layer]=msg
        for health in health_probes:
            try:health.state(layer,msg)
            except ValueError:pass
    for layer in ('upper','middle'):
        observer.create_subscription(PositionSessionState,'/bindu/position/'+layer+'_state',lambda m,l=layer:remember(l,m),10)
    observer.create_subscription(PositionSessionServo,'/bindu/position/servo',lambda m:frames.append(m),10)
    observations={};upper_frames=[];active_meta=None;servo_stub=None
    if target_stream:
        observer.create_subscription(PositionObservation,'/bindu/position/observation',lambda msg:observations.update(last=msg),qos_profile_sensor_data)
        observer.create_subscription(PositionSessionUpper,'/bindu/position/upper',upper_frames.append,10)
    checks={}
    def launch(layer):
        executable={'upper':'robot_control/lib/robot_control/robot_control_node',
                    'middle':'snowbot_dual_arm_control/lib/snowbot_dual_arm_control/dual_arm_control_node'}[layer]
        logfile=args.output.parent/(layer+'-'+str(len(children))+'.log')
        handle=logfile.open('w');logs.append(handle)
        command=[str(args.vendor_root/'install'/executable)]
        if layer=='upper':command+=['--ros-args','-p','control_source:=topic']
        launch_stamps[layer]=observer.get_clock().now().nanoseconds
        states.pop(layer,None)
        process=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,env=os.environ.copy())
        children.append(process);launched[process.pid]=time.monotonic()
        (args.output.parent/(str(process.pid)+'-launch.json')).write_text(json.dumps(dict(
            command=command,pid=process.pid,environment={key:os.environ.get(key,'') for key in
            ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH','ROS_DOMAIN_ID')}),indent=2)+'\n')
        return process
    def stop(process):
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
    def pump(duration,predicate=None,*,feed=True):
        nonlocal sample
        end=time.monotonic()+duration;last=0.
        while time.monotonic()<end:
            now=time.monotonic()
            if feed and now-last>=.01:
                last=now
                if not frozen:sample+=1
                msg=PositionObservation();msg.header.stamp=feedback.get_clock().now().to_msg()
                msg.boot_id=333;msg.pdo_sample=max(1,sample);msg.pdo_age=max(0.,now-freeze_started) if frozen else .001
                msg.receive_attempt=sample;msg.wkc=msg.expected_wkc=67;msg.valid_mask=0xfffff
                msg.position=q;msg.speed=[0.]*20;msg.current=[0.]*20;msg.status=[0x27]*20
                if target_stream:
                    msg.reference_mode=2;msg.execution_mode=0
                    msg.sample_time_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                    # This nominal reference is an explicit in-memory servo fixture.
                    # No servo executable, trajectory generator or device is running.
                    msg.session_selected=active_meta is not None
                    if active_meta is not None:
                        msg.session=active_meta.session;msg.command_sequence=active_meta.sequence
                        msg.segment_id=active_meta.sequence
                    msg.reference_valid=msg.output_valid=active_meta is not None
                    msg.trajectory_phase=2 if active_meta is not None else 0
                    msg.reference_position=q[:7]+q[8:15]+q[18:20]
                    msg.reference_speed=msg.reference_acceleration=[0.]*16
                    # Serialization canaries remain synthetic, including valid flags.
                    msg.output_position=q[:7]+q[8:15]+q[18:20];msg.output_speed=[0.]*16
                    msg.output_torque=[.125]*16;msg.output_kp=[1.]*16;msg.output_kd=[.5]*16
                obs_pub.publish(msg)
                state=PositionSessionState();state.header.stamp=msg.header.stamp
                state.layer='servo';state.control_source=args.execution_mode;state.startup_complete=True
                if target_stream:state.wire_version=3;state.reference_mode=2
                state.feedback_ready=True;state.instance_id=333;state.feedback_boot_id=333
                state.feedback_sample=sample;state.stop_policy='disable_no_load';state.state='ready_idle'
                servo_state_pub.publish(state)
                for health in health_probes:
                    health.observe(msg);health.state('servo',state)
                old=RetInformation();old.header.stamp=msg.header.stamp
                old.l_position=q[:8];old.r_position=q[8:16]
                old.l_speed=old.r_speed=[0.]*8;old.l_iq=old.r_iq=[0.]*8
                ret_pub.publish(old)
            rclpy.spin_once(observer,timeout_sec=.002)
            rclpy.spin_once(feedback,timeout_sec=0.)
            for process in children:
                if process.pid in launched and process.pid not in mapped and time.monotonic()-launched[process.pid]>=.3:
                    maps=Path('/proc')/str(process.pid)/'maps'
                    if maps.is_file():
                        (args.output.parent/(str(process.pid)+'-maps.log')).write_text(maps.read_text())
                        mapped.add(process.pid)
            if predicate and predicate():return True
        return bool(predicate and predicate())
    def command(session,seq,operation=1,*,middle=None,expires=None):
        nonlocal active_meta
        msg=PositionSessionCommand();now=observer.get_clock().now().nanoseconds
        msg.meta.session=session;msg.meta.sequence=seq;msg.meta.source_stamp_ns=now
        msg.meta.expires_at_ns=now+200_000_000 if expires is None else expires
        msg.meta.operation=operation;msg.meta.middle_instance_id=middle or states['middle'].instance_id
        msg.meta.feedback_boot_id=333;msg.meta.feedback_sample=sample
        msg.command.l_arm_position=q[:7];msg.command.r_arm_position=q[8:15]
        msg.command.l_arm_position[0]+=.00002 if target_stream else .01
        msg.waist_position=q[18:20]
        msg.waist_position[0]+=.00001 if target_stream else .005
        if target_stream:
            msg.meta.reference_mode=2;msg.meta.source_command_id=f'fixture-{session}-{seq}'
            msg.meta.lease_id='offline-fixture';msg.meta.epoch=7;msg.meta.profile_hash=profile.digest
            msg.trajectory.version=2;msg.trajectory.segment_id=seq
            for field,getter,cap in (('max_velocity',profile.speed,.25),
                    ('max_acceleration',profile.acceleration,.5),('max_jerk',profile.jerk,162.)):
                setattr(msg.trajectory,field,[min(cap,getter(n)) for n in CONTROL_NAMES])
            active_meta=msg.meta
        cmd_pub.publish(msg);return msg
    def stream_bootstrap(period,session):
        """Use actual Bindu send/receipt logic on DDS, with a memory servo ACK."""
        nonlocal active_meta
        from types import SimpleNamespace as S
        from bindu_hardware.snowbot_session import CONTROL_GROUPS,SourceEnvelope,SessionHealth
        from bindu_runtime.snowbot_position_session import PositionSessionDevices
        pump(.25);active_meta=None
        d=PositionSessionDevices.__new__(PositionSessionDevices)
        observer.now=lambda:observer.get_clock().now().nanoseconds*1e-9
        events=[];observer.event=lambda *args,**kwargs:events.append((args,kwargs))
        d.node=observer;d.health=SessionHealth(observer.now,reference_mode=2)
        d.envelopes=SourceEnvelope(session,observer.now)
        d.sent={};d.target_instance=d.source_identity=d.takeover_pending=None
        d.target_fault='';d.applied_sequence=0
        d.arm_type=PositionSessionCommand;d.publishers={'arms':cmd_pub}
        d.receivers={'arms':'/robot_upperMonitor','middle':'/dual_arm_control_node','head':'/servo_node'}
        d.io=S(profile=profile,source_motion=None,sending_hold=False,requesting_pause=False,stop_targets={})
        health_probes.append(d.health)
        ack_pub=feedback.create_publisher(PositionSessionState,'/bindu/position/servo_receipt',1)
        ack_sub=observer.create_subscription(PositionSessionState,'/bindu/position/servo_receipt',d.receive_receipt,1)
        try:
            def ready():
                if ack_pub.get_subscription_count()!=1 or d.health.advances<2:return False
                try:d.check_ready(CONTROL_GROUPS)
                except RuntimeError:return False
                return True
            assert pump(3.,ready),'Synthetic sender capture prerequisites not ready'
            next_input=time.monotonic();attempts=0
            target=dict(zip(CONTROL_NAMES,q[:7]+q[8:15]+q[18:20]))
            def tick():
                nonlocal next_input,attempts,active_meta
                now=time.monotonic()
                if now<next_input:return
                next_input=now+period;attempts+=1
                d.advance_session()
                d.io.source_motion=S(command_id=f'bootstrap-{session}-{attempts}',lease_id='dds-bootstrap',epoch=9,
                    stamp=observer.now(),valid_for=.2)
                d.send(CONTROL_GROUPS,target)
                if d.sent:active_meta=S(**d.sent[d.envelopes.sequence][0])
            end=time.monotonic()+.08
            while time.monotonic()<end:tick();pump(.002)
            assert attempts>=3 and d.envelopes.sequence==1 and d.takeover_pending is not None
            assert any(f.meta.session==session and f.meta.sequence==1 for f in frames)
            metadata=d.sent[1][0];original_expiry=metadata['expires_at_ns']
            assert d.takeover_pending[0]['expires_at_ns']==original_expiry
            ack=PositionSessionState();ack.header.stamp=feedback.get_clock().now().to_msg()
            ack.layer='servo';ack.wire_version=3;ack.reference_mode=2;ack.instance_id=333;ack.feedback_boot_id=333
            ack.session=session;ack.sequence=1;ack.state='accepted';ack.reason='synthetic_memory_servo'
            ack_pub.publish(ack)
            assert pump(.06,lambda:d.takeover_pending is None)
            assert d.envelopes.sequence==1  # Receipt never dispatches another source.
            # Keep the original input schedule: an ACK never forces an extra tick.
            end=time.monotonic()+2*period
            while d.envelopes.sequence==1 and time.monotonic()<end:tick();pump(.002)
            assert d.envelopes.sequence==2
            end=time.monotonic()+.04
            while time.monotonic()<end:tick();pump(.002)
            assert d.envelopes.sequence>=3  # Normal updates do not wait for receipts.
            d.io.sending_hold=True;d.send(CONTROL_GROUPS,target);d.io.sending_hold=False
            sequence=d.envelopes.sequence
            assert pump(.08,lambda:any(f.meta.session==session and f.meta.sequence==sequence and f.meta.operation==2 for f in frames))
            checks[f'bootstrap_{round(1/period)}hz_single_takeover_ack_then_unblocked_target_stream']=True
        finally:
            health_probes.remove(d.health)
            observer.destroy_subscription(ack_sub);feedback.destroy_publisher(ack_pub)

    try:
        pump(.5)
        assert not observer.get_subscriptions_info_by_topic('/ctl_data_pub'),'Offline test requires no native servo receiver'
        assert all(i.node_name=='position_session_validation' for i in observer.get_subscriptions_info_by_topic('/bindu/position/servo')),'Offline test requires no session servo receiver'
        upper=launch('upper')
        assert pump(8,lambda:'upper' in states),'upper must report while middle is absent'
        checks['upper_first_state_without_target']=True
        middle=launch('middle')
        assert pump(12,lambda:all(l in states and states[l].startup_complete and states[l].feedback_ready for l in ('upper','middle'))),{l:(m.state,m.reason) for l,m in states.items()}
        checks['late_middle_and_two_feedback_samples']=True
        assert cmd_pub.get_subscription_count()==1
        msg=command(101,1)
        assert pump(.15,lambda:len(frames)>=2),dict(error='No production middle position frames',states={l:(m.state,m.reason) for l,m in states.items()},processes=[p.poll() for p in children])
        matching=[f for f in frames if f.meta.session==101]
        assert len(matching)>=2
        assert all(f.meta==msg.meta for f in matching)
        assert all(f.command.l_position[0]==msg.command.l_arm_position[0] for f in matching)
        assert all(list(f.waist_position)==list(msg.waist_position) for f in matching)
        assert matching[1].frame_sequence>matching[0].frame_sequence
        checks['target_meta_atomic_and_derived_frames_do_not_renew']=True
        checks['waist_arm_target_metadata_remain_one_envelope']=True
        if target_stream:
            assert msg.trajectory.version==2 and msg.trajectory.effective_at_ns==0
            assert all(not getattr(msg.trajectory,name) for name in
                       ('piece_start','duration','source_duration','source_elapsed','control_count','controls'))
            assert all(frame.trajectory==msg.trajectory and frame.meta==msg.meta for frame in matching)
            relayed=[frame for frame in upper_frames if frame.meta.session==101]
            assert relayed and all(frame.trajectory==msg.trajectory and frame.meta==msg.meta for frame in relayed)
            assert all(frame.meta.expires_at_ns==msg.meta.expires_at_ns for frame in matching)
            checks['empty_curve_target_recipe_and_source_identity_relayed_unchanged']=True
            observed=observations['last']
            assert observed.reference_mode==2 and observed.execution_mode==0
            assert observed.reference_valid and observed.output_valid and observed.segment_id==1
            assert observed.sample_time_ns==observed.header.stamp.sec*1_000_000_000+observed.header.stamp.nanosec
            assert list(observed.reference_position)==q[:7]+q[8:15]+q[18:20]
            assert list(observed.output_torque)==[.125]*16 and list(observed.output_kp)==[1.]*16 and list(observed.output_kd)==[.5]*16
            assert deserialize_message(serialize_message(observed),type(observed))==observed
            assert deserialize_message(serialize_message(msg.trajectory),PositionTrajectory)==msg.trajectory
            checks['synthetic_nominal_reference_roundtrip_without_servo_execution']=True
            payload=observation_payload(observed)
            for name in ('execution_mode','trajectory_phase','reference_valid','hold_basis','sample_time_ns'):
                assert payload[name]==getattr(observed,name)
            for name in ('reference_position','reference_speed','reference_acceleration','output_position',
                         'output_speed','output_torque','output_kp','output_kd','hold_position'):
                assert len(payload[name])==16 and payload[name]==list(getattr(observed,name))
            assert payload['reference_valid'] is True and payload['output_valid'] is True
            # Older observations omit optional fields; they must remain missing, not false/zero.
            from types import SimpleNamespace
            base=('header','boot_id','pdo_sample','pdo_age','wkc','expected_wkc','valid_mask','position','speed','status')
            legacy=observation_payload(SimpleNamespace(**{name:getattr(observed,name) for name in base}))
            assert not any(name in legacy for name in ('execution_mode','trajectory_phase','reference_valid','hold_basis','sample_time_ns','output_valid'))
            checks['existing_log_payload_preserves_typed_fields_valid_and_missing']=True
        # Serialization of all envelopes and old interfaces uses the real generated types.
        for value in (msg,PositionSessionUpper(),matching[0],PositionObservation(hold_valid=True,hold_sample=1,hold_position=[.2]*16),RetInformation()):
            clone=deserialize_message(serialize_message(value),type(value));assert clone==value
        checks['generated_message_roundtrips']=True
        old_count=len(frames);cmd_pub.publish(msg);pump(.04)
        assert all(f.meta.sequence==1 for f in frames[old_count:] if f.meta.session==101)
        if target_stream:assert all(f.meta==msg.meta and f.trajectory==msg.trajectory for f in frames[old_count:] if f.meta.session==101)
        checks['duplicate_source_has_no_new_identity_or_deadline']=True
        pump(.25);count=len(frames);pump(.08);assert len(frames)==count
        checks['source_expiry_stops_middle_republication']=True
        command(101,2,0);pump(.06);assert len(frames)==count
        checks['target_after_timeout_requires_takeover']=True
        command(101,3,1);assert pump(.12,lambda:len(frames)>count)
        checks['explicit_retakeover_after_deadline']=True
        frozen=True;freeze_started=time.monotonic();pump(.04)
        queued=command(999,1,1);pump(.01);frozen=False
        assert pump(2,lambda:states['middle'].feedback_ready)
        count=len(frames);assert observer.get_clock().now().nanoseconds<queued.meta.expires_at_ns;cmd_pub.publish(queued);pump(.03);assert len(frames)==count
        checks['queued_new_session_predating_recovery_baseline_rejected']=True
        command(101,4,0);pump(.08);assert len(frames)==count
        checks['pdo_freeze_requires_new_session_after_recovery']=True
        command(102,1,1);assert pump(.12,lambda:len(frames)>count)
        checks['new_session_after_pdo_recovery']=True
        old_instance=states['middle'].instance_id;stop(middle);pump(.25)
        middle=launch('middle')
        assert pump(12,lambda:states.get('middle') is not None and states['middle'].instance_id!=old_instance and all(states.get(layer) is not None and states[layer].feedback_ready for layer in ('upper','middle'))),{l:(m.state,m.reason,m.feedback_ready) for l,m in states.items()}
        count=len(frames);command(103,1,1,middle=old_instance);pump(.08);assert len(frames)==count
        checks['old_middle_instance_rejected']=True
        command(104,1,1);assert pump(.15,lambda:len(frames)>count)
        checks['restart_new_instance_and_session_reconnect']=True
        # Model failures keep status/communications alive and never enter control.
        stop(upper);stop(middle)
        urdf=args.vendor_root/'install/dual_arm/share/dual_arm/urdf/snowbot.urdf'
        hidden=urdf.with_suffix('.urdf.test-hidden');urdf.rename(hidden)
        try:
            states.pop('middle',None);broken=launch('middle')
            assert pump(8,lambda:'middle' in states and states['middle'].state=='initialization_failed')
            checks['model_failure_still_reports_state']=True;stop(broken)
        finally:hidden.rename(urdf)
        # Reverse startup order, before any motion target.
        states.clear();middle=launch('middle');pump(.5);upper=launch('upper')
        assert pump(10,lambda:all(l in states and states[l].startup_complete and states[l].feedback_ready for l in ('upper','middle')))
        checks['middle_first_and_idle_connection']=True
        if target_stream:
            count=len(frames);command(201,1,1)
            assert pump(.1,lambda:any(f.meta.session==201 for f in frames[count:]))
            # No servo admission receipt is ever published in this test. Ordinary
            # sources and PAUSE must still pass through both production relays.
            for sequence in (2,3):
                packet=command(201,sequence,0)
                assert pump(.06,lambda:any(f.meta==packet.meta for f in frames))
            pause=command(201,4,2)
            assert pump(.08,lambda:any(f.meta==pause.meta for f in frames))
            paused=[f for f in frames if f.meta==pause.meta]
            assert paused and all(f.command.l_position[0]==pause.command.l_arm_position[0]
                and list(f.waist_position)==list(pause.waist_position) for f in paused)
            assert all(f.trajectory==pause.trajectory for f in paused)
            checks['ordinary_updates_and_pause_relay_without_servo_ack_or_target_rewrite']=True
        if target_stream:
            servo_stub=feedback.create_subscription(PositionSessionServo,'/bindu/position/servo',lambda msg:None,1)
            assert pump(3.,lambda:any(i.node_name=='servo_node' for i in observer.get_subscriptions_info_by_topic('/bindu/position/servo')))
            stream_bootstrap(.02,301)
            stream_bootstrap(.01,302)
        if args.bindu_root:
            # The preflight expects this route; the fixture never applies outputs.
            if servo_stub is None:
                servo_stub=feedback.create_subscription(PositionSessionServo,'/bindu/position/servo',lambda msg:None,1)
            observer.destroy_publisher(cmd_pub)
            assert pump(3,lambda:not observer.get_publishers_info_by_topic('/bindu/position/command'))
            probe_log=args.output.parent/'preflight.log'
            with probe_log.open('w') as handle:
                probe=subprocess.Popen([sys.executable,'-c',
                    "import json;from tools.deployment.runtime_check import check_robot_interface;"
                    f"print(json.dumps(check_robot_interface(dict(position_protocol={args.position_protocol!r},execution_mode={args.execution_mode!r},"
                    f"vendor_namespace='/',namespace='bindu_snowbot'),{args.preflight_discovery_timeout!r})))"],
                    cwd=args.bindu_root,stdout=handle,stderr=subprocess.STDOUT,env=os.environ.copy())
                children.append(probe)
                assert pump(args.preflight_discovery_timeout+3.,lambda:probe.poll() is not None),'read-only preflight did not finish'
            assert probe.returncode==0,probe_log.read_text()
            preflight=json.loads(probe_log.read_text().splitlines()[-1])
            assert preflight['robot_interface']=='available',preflight
            checks['read_only_preflight_with_real_type_hashes_and_qos']=True
        result=dict(passed=True,checks=checks,synthetic_input=True,real_controller_input=False,
                    servo_executable_started=False,hardware_commands_sent=False,frames=len(frames),
                    domain=os.environ['ROS_DOMAIN_ID'],preflight_discovery_timeout=args.preflight_discovery_timeout,
                    position_protocol=args.position_protocol,execution_mode=args.execution_mode,
                    observation_origin='synthetic_fixture',relay_frames_origin='production_upper_middle',
                    servo_acknowledgement_tested=False,synthetic_memory_ack_tested=target_stream,servo_trajectory_sampling_tested=False)
        args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    except BaseException as exc:
        args.output.write_text(json.dumps(dict(passed=False,checks=checks,error=repr(exc),layer_states={layer:dict(reason=m.reason,state=m.state,instance_id=m.instance_id,startup_complete=m.startup_complete,feedback_ready=m.feedback_ready) for layer,m in states.items()},child_returncodes={str(p.pid):p.poll() for p in children},endpoint_graph={topic:[dict(node=i.node_namespace.rstrip('/')+'/'+i.node_name,type=i.topic_type,gid=list(i.endpoint_gid)) for i in observer.get_publishers_info_by_topic(topic)] for topic in ('/bindu/position/upper_state','/bindu/position/middle_state','/bindu/position/servo')},servo_executable_started=False,position_protocol=args.position_protocol,execution_mode=args.execution_mode,preflight_discovery_timeout=args.preflight_discovery_timeout,servo_acknowledgement_tested=False),indent=2)+'\n')
        raise
    finally:
        for process in children:stop(process)
        for handle in logs:handle.close()
        observer.destroy_node();feedback.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()

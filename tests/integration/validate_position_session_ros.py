#!/usr/bin/env python3
"""Offline production upper/middle relay test. Never starts servo or accesses a device."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def trajectory_description(profile, names, initial, target):
    """Export two genuine native pieces; this is relay data, not servo acceptance."""
    from bindu_execution.interpolation import PreparedSegment, State
    first=PreparedSegment.target(profile,names,State.rest(initial),
                                 tuple((a+b)*.5 for a,b in zip(initial,target)),0.)
    second=PreparedSegment.target(profile,names,first.sample(first.end),target,first.end)
    description=(*first.describe(),*second.describe())
    certified=PreparedSegment.from_description(profile,names,description)
    if certified.pieces<2:raise RuntimeError('Require a nonempty native multi-piece trajectory')
    result=dict(version=1,piece_start=[],duration=[],source_duration=[],source_elapsed=[],
                control_count=[],controls=[])
    for start,duration,source_duration,source_elapsed,axes in description:
        result['piece_start'].append(start);result['duration'].append(duration)
        result['source_duration'].append(source_duration);result['source_elapsed'].append(source_elapsed)
        for controls in axes:
            result['control_count'].append(len(controls));result['controls'].extend(controls)
    for field,getter in (('max_velocity',profile.speed),('max_acceleration',profile.acceleration),('max_jerk',profile.jerk)):
        result[field]=[getter(name) for name in names]
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--bindu-root',type=Path,help='Optional Bindu checkout with the installed session preflight')
    parser.add_argument('--position-protocol',choices=('session_v1','session_v2'),default='session_v1')
    parser.add_argument('--execution-mode',choices=('position','impedance'),default='position',help='Synthetic servo state/observation mode; never starts an impedance controller')
    parser.add_argument('--preflight-discovery-timeout',type=float,default=2.,help='New DDS observer discovery window; all feedback and contract freshness checks remain unchanged')
    args=parser.parse_args()
    if not 0 < args.preflight_discovery_timeout <= 10:parser.error('Require 0 < preflight discovery timeout <= 10 seconds')
    trajectory=args.position_protocol=='session_v2'
    if trajectory and not args.bindu_root:parser.error('session_v2 requires --bindu-root for the existing profile/native trajectory core')
    if args.execution_mode=='impedance' and not trajectory:parser.error('Impedance metadata coverage requires session_v2')
    if int(os.environ.get('ROS_DOMAIN_ID','0'))<100 or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Require an isolated ROS domain >=100 and localhost-only transport')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message,deserialize_message
    from snowbot_ctl_msgs.msg import (PositionObservation,PositionSessionCommand,PositionSessionUpper,
        PositionSessionServo,PositionSessionState,RetInformation)
    if trajectory:
        from snowbot_ctl_msgs.msg import PositionTrajectory
        from bindu_contracts.profile import Profile
        from bindu_hardware.snowbot_session import CONTROL_NAMES
        profile=Profile.load(args.bindu_root/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_vendor_position.json')
        # Load the native core before starting freshness-sensitive synthetic feeds.
        from bindu_execution.interpolation import PreparedSegment
    rclpy.init()
    observer=rclpy.create_node('position_session_validation')
    feedback=rclpy.create_node('servo_node') # Test fixture only, not the servo executable.
    obs_pub=feedback.create_publisher(PositionObservation,'/bindu/position/observation',qos_profile_sensor_data)
    ret_pub=feedback.create_publisher(RetInformation,'/dualArmCtl_fdback',10)
    servo_state_pub=feedback.create_publisher(PositionSessionState,'/bindu/position/servo_state',1)
    cmd_pub=observer.create_publisher(PositionSessionCommand,'/bindu/position/command',1)
    states={};frames=[];children=[];logs=[];sample=0;frozen=False;freeze_started=0.
    q=[.1]*20;q[9]=-.1;q[7]=q[15]=0.;q[16:]=[0.,0.,.03,-.04]
    for layer in ('upper','middle'):
        observer.create_subscription(PositionSessionState,'/bindu/position/'+layer+'_state',lambda m,l=layer:states.update({l:m}),10)
    observer.create_subscription(PositionSessionServo,'/bindu/position/servo',lambda m:frames.append(m),10)
    observations={};upper_frames=[]
    if trajectory:
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
        process=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,env=os.environ.copy())
        children.append(process);return process
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
                if trajectory:
                    msg.reference_mode=1;msg.execution_mode=int(args.execution_mode=='impedance')
                    msg.sample_time_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                    # Explicitly no accepted segment, sampled output or physical hold.
                    msg.reference_valid=msg.output_valid=False
                    msg.reference_position=q[:7]+q[8:15]+q[18:20]
                    msg.reference_speed=msg.reference_acceleration=[0.]*16
                    # Serialization canaries are synthetic; output_valid stays false.
                    msg.output_position=q[:7]+q[8:15]+q[18:20];msg.output_speed=[0.]*16
                    msg.output_torque=[.125]*16;msg.output_kp=[1.]*16;msg.output_kd=[.5]*16
                    if args.execution_mode=='impedance':
                        msg.wrench=[0.]*12;msg.sensor_valid_mask=3
                        msg.sensor_data_no=[sample%65536]*2;msg.sensor_age=[.001]*2
                obs_pub.publish(msg)
                state=PositionSessionState();state.header.stamp=msg.header.stamp
                state.layer='servo';state.control_source=args.execution_mode;state.startup_complete=True
                if trajectory:state.wire_version=2;state.reference_mode=1
                state.feedback_ready=True;state.instance_id=333;state.feedback_boot_id=333
                state.feedback_sample=sample;state.stop_policy='disable_no_load';state.state='ready_idle'
                servo_state_pub.publish(state)
                old=RetInformation();old.header.stamp=msg.header.stamp
                old.l_position=q[:8];old.r_position=q[8:16]
                old.l_speed=old.r_speed=[0.]*8;old.l_iq=old.r_iq=[0.]*8
                ret_pub.publish(old)
            rclpy.spin_once(observer,timeout_sec=.002)
            rclpy.spin_once(feedback,timeout_sec=0.)
            if predicate and predicate():return True
        return bool(predicate and predicate())
    def command(session,seq,operation=1,*,middle=None,expires=None):
        msg=PositionSessionCommand();now=observer.get_clock().now().nanoseconds
        msg.meta.session=session;msg.meta.sequence=seq;msg.meta.source_stamp_ns=now
        msg.meta.expires_at_ns=now+200_000_000 if expires is None else expires
        msg.meta.operation=operation;msg.meta.middle_instance_id=middle or states['middle'].instance_id
        msg.meta.feedback_boot_id=333;msg.meta.feedback_sample=sample
        msg.command.l_arm_position=q[:7];msg.command.r_arm_position=q[8:15]
        msg.command.l_arm_position[0]+=.00002 if trajectory else .01
        msg.waist_position=q[18:20]
        msg.waist_position[0]+=.00001 if trajectory else .005
        if trajectory:
            msg.meta.reference_mode=1;msg.meta.source_command_id=f'fixture-{session}-{seq}'
            msg.meta.lease_id='offline-fixture';msg.meta.epoch=7;msg.meta.profile_hash=profile.digest
            initial=tuple(q[:7]+q[8:15]+q[18:20])
            target=tuple(msg.command.l_arm_position)+tuple(msg.command.r_arm_position)+tuple(msg.waist_position)
            for field,value in trajectory_description(profile,CONTROL_NAMES,initial,target).items():
                setattr(msg.trajectory,field,value)
            msg.trajectory.segment_id=seq;msg.trajectory.effective_at_ns=now+60_000_000
        cmd_pub.publish(msg);return msg
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
        if trajectory:
            assert len(msg.trajectory.duration)>=2 and len(msg.trajectory.controls)>0
            assert all(frame.trajectory==msg.trajectory and frame.meta==msg.meta for frame in matching)
            relayed=[frame for frame in upper_frames if frame.meta.session==101]
            assert relayed and all(frame.trajectory==msg.trajectory and frame.meta==msg.meta for frame in relayed)
            assert all(frame.meta.expires_at_ns==msg.meta.expires_at_ns and frame.trajectory.effective_at_ns==msg.trajectory.effective_at_ns for frame in matching)
            checks['native_multi_piece_description_and_source_identity_relayed_unchanged']=True
            observed=observations['last']
            assert observed.reference_mode==1 and observed.execution_mode==int(args.execution_mode=='impedance')
            assert not observed.reference_valid and not observed.output_valid and observed.segment_id==0
            assert observed.sample_time_ns==observed.header.stamp.sec*1_000_000_000+observed.header.stamp.nanosec
            assert list(observed.reference_position)==q[:7]+q[8:15]+q[18:20]
            assert list(observed.output_torque)==[.125]*16 and list(observed.output_kp)==[1.]*16 and list(observed.output_kd)==[.5]*16
            assert deserialize_message(serialize_message(observed),type(observed))==observed
            assert deserialize_message(serialize_message(msg.trajectory),PositionTrajectory)==msg.trajectory
            checks['synthetic_observation_new_fields_and_mode_roundtrip_without_ack']=True
        # Serialization of all envelopes and old interfaces uses the real generated types.
        for value in (msg,PositionSessionUpper(),matching[0],PositionObservation(hold_valid=True,hold_sample=1,hold_position=[.2]*16),RetInformation()):
            clone=deserialize_message(serialize_message(value),type(value));assert clone==value
        checks['generated_message_roundtrips']=True
        old_count=len(frames);cmd_pub.publish(msg);pump(.04)
        assert all(f.meta.sequence==1 for f in frames[old_count:] if f.meta.session==101)
        if trajectory:assert all(f.meta==msg.meta and f.trajectory==msg.trajectory for f in frames[old_count:] if f.meta.session==101)
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
        if args.bindu_root:
            # The preflight expects this route; the fixture never applies outputs.
            feedback.create_subscription(PositionSessionServo,'/bindu/position/servo',lambda msg:None,1)
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
                    servo_acknowledgement_tested=False,servo_trajectory_sampling_tested=False)
        args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    except BaseException as exc:
        args.output.write_text(json.dumps(dict(passed=False,checks=checks,error=repr(exc),servo_executable_started=False,position_protocol=args.position_protocol,execution_mode=args.execution_mode,preflight_discovery_timeout=args.preflight_discovery_timeout,servo_acknowledgement_tested=False),indent=2)+'\n')
        raise
    finally:
        for process in children:stop(process)
        for handle in logs:handle.close()
        observer.destroy_node();feedback.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()

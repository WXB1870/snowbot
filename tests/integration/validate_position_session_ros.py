#!/usr/bin/env python3
"""Offline production upper/middle relay test. Never starts servo or accesses a device."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID','0'))<100 or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Require an isolated ROS domain >=100 and localhost-only transport')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message,deserialize_message
    from snowbot_ctl_msgs.msg import (PositionObservation,PositionSessionCommand,PositionSessionUpper,
        PositionSessionServo,PositionSessionState,RetInformation)
    from std_msgs.msg import String
    rclpy.init()
    observer=rclpy.create_node('position_session_validation')
    feedback=rclpy.create_node('servo_node') # Test fixture only, not the servo executable.
    obs_pub=feedback.create_publisher(PositionObservation,'/bindu/position/observation',qos_profile_sensor_data)
    ret_pub=feedback.create_publisher(RetInformation,'/dualArmCtl_fdback',10)
    cmd_pub=observer.create_publisher(PositionSessionCommand,'/bindu/position/command',1)
    states={};frames=[];children=[];logs=[];sample=0;frozen=False;freeze_started=0.
    q=[.1]*20;q[9]=-.1;q[7]=q[15]=0.;q[16:]=[0.]*4
    for layer in ('upper','middle'):
        observer.create_subscription(PositionSessionState,'/bindu/position/'+layer+'_state',lambda m,l=layer:states.update({l:m}),10)
    observer.create_subscription(PositionSessionServo,'/bindu/position/servo',lambda m:frames.append(m),10)
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
                obs_pub.publish(msg)
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
        msg.command.l_arm_position[0]+=.01
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
        assert matching[1].frame_sequence>matching[0].frame_sequence
        checks['target_meta_atomic_and_derived_frames_do_not_renew']=True
        # Serialization of all envelopes and old interfaces uses the real generated types.
        for value in (msg,PositionSessionUpper(),matching[0],PositionObservation(),RetInformation()):
            clone=deserialize_message(serialize_message(value),type(value));assert clone==value
        checks['generated_message_roundtrips']=True
        old_count=len(frames);cmd_pub.publish(msg);pump(.04)
        assert all(f.meta.sequence==1 for f in frames[old_count:] if f.meta.session==101)
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
        assert pump(12,lambda:states.get('middle') is not None and states['middle'].instance_id!=old_instance and states['middle'].feedback_ready)
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
        result=dict(passed=True,checks=checks,synthetic_input=True,real_controller_input=False,
                    servo_executable_started=False,hardware_commands_sent=False,frames=len(frames),
                    domain=os.environ['ROS_DOMAIN_ID'])
        args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    except BaseException as exc:
        args.output.write_text(json.dumps(dict(passed=False,checks=checks,error=repr(exc),servo_executable_started=False),indent=2)+'\n')
        raise
    finally:
        for process in children:stop(process)
        for handle in logs:handle.close()
        observer.destroy_node();feedback.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()

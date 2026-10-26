#!/usr/bin/env python3
"""Prepare a local vendor session; optional impedance is never enabled here."""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path
import shutil
import subprocess
import tempfile

ASSETS=Path(__file__).resolve().parent/'snowbot_vendor/position_session'
COMPONENTS=('observation','takeover','timing','session','receipt','target')
NATIVE_FILES=('interpolation.hpp','interpolation.cpp','adaptive_stream.hpp','adaptive_stream.cpp','curve_math.hpp','rigid_prediction.hpp')

def baseline_source():
    root=Path(__file__).resolve().parents[2]
    identity=json.loads((root/'vendor/snowbot/baseline.json').read_text())
    manifest=root/identity['manifest']
    if (identity.get('schema')!=1 or identity.get('role')!='immutable_input' or
            hashlib.sha256(manifest.read_bytes()).hexdigest()!=identity['manifest_sha256']):
        raise ValueError('POSITION_BASELINE_IDENTITY_MISMATCH')
    source=(root/identity['source']).resolve()
    if root not in source.parents or not source.is_dir():
        raise ValueError('POSITION_BASELINE_SOURCE_MISSING')
    return source



def release_output(release_id):
    """Select an immutable named output in this application's release directory."""
    if (not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]{0,95}', release_id)
            or release_id.lower() in ('latest', 'current', 'final')):
        raise ValueError('POSITION_RELEASE_ID_INVALID')
    root=Path(__file__).resolve().parents[2]
    releases=root/'releases'
    if releases.is_symlink():
        raise ValueError('POSITION_RELEASE_DIRECTORY_SYMLINK')
    return releases/release_id


def source_identity(path):
    """Record Git provenance when available; file hashes remain authoritative."""
    path=Path(path).resolve()
    try:
        def git(*args):
            return subprocess.check_output(['git','-C',str(path),*args],stderr=subprocess.DEVNULL,text=True).strip()
        return dict(path=str(path),repository=git('rev-parse','--show-toplevel'),
                    commit=git('rev-parse','HEAD'),
                    tracked_dirty=bool(git('status','--porcelain','--untracked-files=no')))
    except (OSError,subprocess.CalledProcessError):
        return dict(path=str(path),repository=None,commit=None,tracked_dirty=None)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def prepare(source,output,*,observation_only=False,with_impedance=False,execution_source=None,prediction_urdf=None,prediction_tool=None,waist_velocity_ratio=None,waist_feedforward_gain=0.):
    if (type(waist_feedforward_gain) not in (int,float) or not math.isfinite(waist_feedforward_gain)
            or not 0 <= waist_feedforward_gain <= .1):raise ValueError('WAIST_FEEDFORWARD_GAIN_INVALID')
    if waist_velocity_ratio is None:
        if waist_feedforward_gain:raise ValueError('WAIST_VELOCITY_RATIO_REQUIRED')
    else:
        if (not isinstance(waist_velocity_ratio,(tuple,list)) or len(waist_velocity_ratio)!=2 or any(type(x) not in (int,float) or not math.isfinite(x)
                or not 1 <= x <= 200 for x in waist_velocity_ratio)):raise ValueError('WAIST_VELOCITY_RATIO_INVALID')
        if observation_only or with_impedance:raise ValueError('WAIST_UNITS_REQUIRES_POSITION_SESSION')
    if bool(prediction_urdf)!=bool(prediction_tool):raise ValueError('RIGID_PREDICTION_MODEL_AND_TOOL_REQUIRED')
    if prediction_urdf and (observation_only or with_impedance):raise ValueError('RIGID_PREDICTION_REQUIRES_TARGET_STREAM')
    if observation_only and with_impedance:raise ValueError("IMPEDANCE_REQUIRES_FULL_SESSION")
    source,output=Path(source).resolve(),Path(output).absolute()
    if not source.is_dir() or source==output or source in output.parents or output in source.parents:
        raise ValueError('POSITION_SOURCE_OUTPUT_OVERLAP_OR_MISSING')
    if output.exists() or output.is_symlink():raise ValueError('POSITION_OUTPUT_EXISTS')
    if execution_source:
        native=Path(execution_source).resolve()
    elif not observation_only:
        from bindu_contracts.resources import package_share
        native=package_share('bindu_execution')/'native'
    else:
        native=None
    if not observation_only and any(not (native/n).is_file() or (native/n).is_symlink() for n in NATIVE_FILES):
        raise ValueError('POSITION_EXECUTION_SOURCE_REQUIRED')
    baseline=json.loads((ASSETS/'baseline_manifest.json').read_text())
    if any(p.is_symlink() for p in source.rglob('*')):raise ValueError('POSITION_SOURCE_SYMLINK')
    for rel,expected in baseline.items():
        if not (source/rel).is_file() or digest(source/rel)!=expected:
            raise ValueError('POSITION_BASELINE_MISMATCH:'+rel)
    # Preserve unlisted colleague assets too. Only reviewed patches may modify files.
    output.parent.mkdir(parents=True,exist_ok=True)
    arm_reference_margin=0.
    stage=Path(tempfile.mkdtemp(prefix='.position-',dir=output.parent))
    try:
        shutil.copytree(source,stage/'src')
        patches=COMPONENTS[:1] if observation_only else COMPONENTS[:4]
        for name in patches:
            subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/(name+'.patch'))],cwd=stage/'src',check=True,capture_output=True,text=True)
        if not observation_only:
            for package in ('robot_control','dual_arm_control','servo_control'):
                for name in ('position_session_contract.h','position_connection_diagnostics.h'):
                    shutil.copy2(ASSETS/name,stage/'src'/package/'include'/name)
            for package in ('robot_control','dual_arm_control'):
                shutil.copy2(ASSETS/'position_session_ros.h',stage/'src'/package/'include/position_session_ros.h')
        if not observation_only:
            shutil.copy2(ASSETS/'position_trajectory.h',stage/'src/servo_control/include/position_trajectory.h')
            for name in NATIVE_FILES:
                folder='include' if name.endswith('.hpp') else 'src'
                destination=stage/'src/servo_control'/folder/'bindu_execution_native'/name
                destination.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(native/name,destination)
        if with_impedance:
            subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'impedance.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
            patches=(*patches,'impedance')
        if not observation_only:
            subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'receipt.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
            shutil.copy2(ASSETS/'position_receipt.h',stage/'src/servo_control/include/position_receipt.h')
            patches=(*patches,'receipt')
            # The target stream is a pure-position extension. Keep the optional
            # impedance assembly's original ingress and PAUSE behavior byte-for-byte.
            if not with_impedance:
                subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'target.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
                patches=(*patches,'target')
                subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'pause_hold.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
                patches=(*patches,'pause_hold')
                subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'drive_maintenance.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
                patches=(*patches,'drive_maintenance')
                if prediction_urdf:
                    from bindu_kinematics import model_codegen
                    generator=Path(model_codegen.__file__)
                    model_tool=json.loads(Path(prediction_tool).read_text())
                    model_names=[f'arm_{side}_joint_{i}' for side in ('l','r') for i in range(1,8)]+['body_joint_1','body_joint_2']
                    arm_reference_margin=model_tool.get('posture_reference_margin_rad',0.)
                    if (type(arm_reference_margin) not in (int,float) or not math.isfinite(arm_reference_margin)
                            or not 0 <= arm_reference_margin < .2):raise ValueError('ARM_REFERENCE_MARGIN_INVALID')
                    header=model_codegen.model_header(prediction_urdf,model_names,model_tool['left_frame'],model_tool['right_frame'])
                    header+='\n#define SNOWBOT_ARM_REFERENCE_MARGIN '+repr(float(arm_reference_margin))+'\n'
                    (stage/'src/servo_control/include/bindu_execution_native/prediction_model.hpp').write_text(header)
                    subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'prediction.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
                    patches=(*patches,'prediction')
        if not observation_only and not with_impedance:
            subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'runtime_timing.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
            patches=(*patches,'runtime_timing')
        if waist_velocity_ratio is not None:
            header=(ASSETS/'position_waist_units.h').read_text()
            header=header.replace('{1.0, 1.0}', '{'+', '.join(repr(float(x)) for x in waist_velocity_ratio)+'}')
            header=header.replace('feedforward_gain = 0.0', 'feedforward_gain = '+repr(float(waist_feedforward_gain)))
            (stage/'src/servo_control/include/position_waist_units.h').write_text(header)
            subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'waist_units.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
            patches=(*patches,'waist_units')
        # Remove only patch-generated backups, retaining any pre-existing assets.
        for p in (stage/'src').rglob('*.orig'):
            if not (source/p.relative_to(stage/'src')).exists():p.unlink()

        changed={}
        for p in (stage/'src').rglob('*'):
            if not p.is_file():continue
            rel=str(p.relative_to(stage/'src'));original=source/rel
            if not original.exists() or digest(original)!=digest(p):
                changed[rel]=dict(before=digest(original) if original.exists() else None,after=digest(p))
        # The four original wire definitions must remain byte-identical.
        for name in ('RetInformation','JointPositionCommand','MidNodeCtlMsgSend','CtlInformation'):
            rel='ctl_msgs/msg/'+name+'.msg'
            if digest(stage/'src'/rel)!=digest(source/rel):raise ValueError('LEGACY_MESSAGE_CHANGED:'+name)
        pairing_path=Path(__file__).resolve().parents[2]/'dependencies.repos'
        pairing=json.loads(pairing_path.read_text())['repositories']['bindu']
        manifest=dict(preparation=dict(tool_sha256=digest(Path(__file__)),snowbot=source_identity(ASSETS),
            recommended_bindu_source=pairing,source_pairing_sha256=digest(pairing_path),
            execution=None if native is None else source_identity(native),artifact_role='generated_candidate',
            deployment_verified=False,physical_acceptance=False),arm_reference_margin_rad=arm_reference_margin,schema=1,baseline='restored-original-20261021',latest_colleague_source_verified=False,
            baseline_manifest_sha256=digest(ASSETS/'baseline_manifest.json'),
            components={n:digest(ASSETS/(n+'.patch')) for n in patches},shared_helpers={} if observation_only else {n:digest(ASSETS/n) for n in ('position_session_contract.h','position_connection_diagnostics.h','position_session_ros.h')},files=changed,
            rigid_prediction=(dict(enabled=True,recipe_version=3,urdf_sha256=digest(Path(prediction_urdf)),tool_sha256=digest(Path(prediction_tool)),generator_sha256=digest(generator),names=model_names,left_frame=model_tool['left_frame'],right_frame=model_tool['right_frame']) if prediction_urdf else dict(enabled=False)),
            feedback_publication=dict(isolated=not observation_only and not with_impedance,
                mode='latest_only_worker' if not observation_only and not with_impedance else 'original',
                max_sample_age_ms=20 if not observation_only and not with_impedance else None,
                timestamp='acquisition_start' if not observation_only and not with_impedance else None,fastdds_default='ASYNCHRONOUS' if not observation_only and not with_impedance else None,
                site_override_preserved=True),
            csp_diagnostics=dict(version=1,slots=[16,17,18,19],command_source='pre_send_pdo_buffer',
                feedback_source='same_receive_attempt',sdo_reads_added=False,physical_units_verified=False),
            driver_started=False,hardware_commands_sent=False,full_build_verified=False,
            csp_encoder=dict(position_counts_per_rev=262144,speed_counts_per_rev=65536,physical_units_verified=False),
            waist_units=dict(enabled=waist_velocity_ratio is not None,position_scale_changed=True,
                physical_units_verified=False,motor_per_position_velocity=waist_velocity_ratio,
                velocity_command_scaled=waist_velocity_ratio is not None,
                velocity_object='0x60B1' if waist_feedforward_gain else '0x60FF',
                feedforward_gain=waist_feedforward_gain,
                helper_sha256=digest(stage/'src/servo_control/include/position_waist_units.h') if waist_velocity_ratio is not None else None),
            stop_policy='unconfirmed',execution_mode='position',reference_mode='follower',trajectory_available=not observation_only,target_stream_available=not observation_only and not with_impedance,
            session_wire_version=3,target_wire_version=5,target_reference_mode=2,target_prepare_ms=10.,target_motion_ttl_ms=300,target_pause_ttl_ms=2000,
            receipt_helper_sha256=None if observation_only else digest(ASSETS/'position_receipt.h'),
            trajectory_helper_sha256=None if observation_only else digest(ASSETS/'position_trajectory.h'),
            native_execution={} if observation_only else {n:digest(native/n) for n in NATIVE_FILES},impedance_available=with_impedance,
            impedance_calibration_confirmed=False,source_tree_sha256=hashlib.sha256(json.dumps(baseline,sort_keys=True).encode()).hexdigest())
        (stage/'position_release.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
        stage.rename(output)
    except BaseException:
        shutil.rmtree(stage,ignore_errors=True);raise
    return output

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path,nargs='?',default=baseline_source())
    destination=p.add_mutually_exclusive_group(required=True)
    destination.add_argument('--release-id',help='Unique immutable candidate name under snowbot/releases; latest/current/final are rejected')
    destination.add_argument('--output',type=Path,help='Explicit output for temporary checks or externally managed assembly')
    p.add_argument('--observation-only',action='store_true')
    p.add_argument('--with-impedance',action='store_true',help='Include optional backend; default execution mode remains position')
    p.add_argument('--execution-source',type=Path,help='Reviewed Bindu native execution source directory; copied and hashed, never built or started here')
    p.add_argument('--prediction-urdf',type=Path,help='Reviewed actual kinematic URDF for rigid prediction; local generation only')
    p.add_argument('--prediction-tool',type=Path,help='Reviewed tool config selecting the two end frames; must accompany prediction-urdf')
    p.add_argument('--waist-velocity-ratio',type=float,nargs=2,metavar=('WAIST1','WAIST2'),help='Explicit motor/current-position velocity ratios for feedback division AND command multiplication; positions and limits remain unchanged')
    p.add_argument('--waist-feedforward-gain',type=float,default=0.,help='Optional waist CSP 0x60B1 gain, 0..0.1; requires explicit velocity ratios; default preserves 0x60FF')
    a=p.parse_args()
    output=release_output(a.release_id) if a.release_id is not None else a.output
    print(prepare(a.source,output,observation_only=a.observation_only,with_impedance=a.with_impedance,execution_source=a.execution_source,prediction_urdf=a.prediction_urdf,prediction_tool=a.prediction_tool,waist_velocity_ratio=a.waist_velocity_ratio,waist_feedforward_gain=a.waist_feedforward_gain))

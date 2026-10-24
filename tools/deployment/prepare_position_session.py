#!/usr/bin/env python3
"""Prepare a local vendor session; optional impedance is never enabled here."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

ASSETS=Path(__file__).resolve().parent/'snowbot_vendor/position_session'
COMPONENTS=('observation','takeover','timing','session','receipt','target')
NATIVE_FILES=('interpolation.hpp','interpolation.cpp','adaptive_stream.hpp','adaptive_stream.cpp','curve_math.hpp','rigid_prediction.hpp')

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def prepare(source,output,*,observation_only=False,with_impedance=False,execution_source=None,prediction_urdf=None,prediction_tool=None):
    if bool(prediction_urdf)!=bool(prediction_tool):raise ValueError('RIGID_PREDICTION_MODEL_AND_TOOL_REQUIRED')
    if prediction_urdf and (observation_only or with_impedance):raise ValueError('RIGID_PREDICTION_REQUIRES_TARGET_STREAM')
    if observation_only and with_impedance:raise ValueError("IMPEDANCE_REQUIRES_FULL_SESSION")
    source,output=Path(source).resolve(),Path(output).absolute()
    if not source.is_dir() or source==output or source in output.parents or output in source.parents:
        raise ValueError('POSITION_SOURCE_OUTPUT_OVERLAP_OR_MISSING')
    if output.exists() or output.is_symlink():raise ValueError('POSITION_OUTPUT_EXISTS')
    native=Path(execution_source).resolve() if execution_source else Path(__file__).resolve().parents[3]/'galbot/bindu/src/control/bindu_execution/native'
    if not observation_only and any(not (native/n).is_file() or (native/n).is_symlink() for n in NATIVE_FILES):
        raise ValueError('POSITION_EXECUTION_SOURCE_REQUIRED')
    baseline=json.loads((ASSETS/'baseline_manifest.json').read_text())
    if any(p.is_symlink() for p in source.rglob('*')):raise ValueError('POSITION_SOURCE_SYMLINK')
    for rel,expected in baseline.items():
        if not (source/rel).is_file() or digest(source/rel)!=expected:
            raise ValueError('POSITION_BASELINE_MISMATCH:'+rel)
    # Preserve unlisted colleague assets too. Only reviewed patches may modify files.
    output.parent.mkdir(parents=True,exist_ok=True)
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
                    import importlib.util
                    generator=native.parents[3]/'tools/analysis/replay_geometric_prediction.py'
                    spec=importlib.util.spec_from_file_location('rigid_generator',generator)
                    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
                    model_tool=json.loads(Path(prediction_tool).read_text())
                    model_names=[f'arm_{side}_joint_{i}' for side in ('l','r') for i in range(1,8)]+['body_joint_1','body_joint_2']
                    header=module.model_header(prediction_urdf,model_names,model_tool['left_frame'],model_tool['right_frame'])
                    (stage/'src/servo_control/include/bindu_execution_native/prediction_model.hpp').write_text(header)
                    subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'prediction.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
                    patches=(*patches,'prediction')
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
        manifest=dict(schema=1,baseline='restored-original-20261021',latest_colleague_source_verified=False,
            baseline_manifest_sha256=digest(ASSETS/'baseline_manifest.json'),
            components={n:digest(ASSETS/(n+'.patch')) for n in patches},shared_helpers={} if observation_only else {n:digest(ASSETS/n) for n in ('position_session_contract.h','position_connection_diagnostics.h','position_session_ros.h')},files=changed,
            rigid_prediction=(dict(enabled=True,recipe_version=3,urdf_sha256=digest(Path(prediction_urdf)),tool_sha256=digest(Path(prediction_tool)),generator_sha256=digest(generator),names=model_names,left_frame=model_tool['left_frame'],right_frame=model_tool['right_frame']) if prediction_urdf else dict(enabled=False)),
            driver_started=False,hardware_commands_sent=False,full_build_verified=False,
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
    p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--observation-only',action='store_true')
    p.add_argument('--with-impedance',action='store_true',help='Include optional backend; default execution mode remains position')
    p.add_argument('--execution-source',type=Path,help='Reviewed Bindu native execution source directory; copied and hashed, never built or started here')
    p.add_argument('--prediction-urdf',type=Path,help='Reviewed actual kinematic URDF for rigid prediction; local generation only')
    p.add_argument('--prediction-tool',type=Path,help='Reviewed tool config selecting the two end frames; must accompany prediction-urdf')
    a=p.parse_args()
    print(prepare(a.source,a.output,observation_only=a.observation_only,with_impedance=a.with_impedance,execution_source=a.execution_source,prediction_urdf=a.prediction_urdf,prediction_tool=a.prediction_tool))

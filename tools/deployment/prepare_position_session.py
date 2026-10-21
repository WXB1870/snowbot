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
COMPONENTS=('observation','takeover','timing','session')

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def prepare(source,output,*,observation_only=False,with_impedance=False):
    if observation_only and with_impedance:raise ValueError("IMPEDANCE_REQUIRES_FULL_SESSION")
    source,output=Path(source).resolve(),Path(output).absolute()
    if not source.is_dir() or source==output or source in output.parents or output in source.parents:
        raise ValueError('POSITION_SOURCE_OUTPUT_OVERLAP_OR_MISSING')
    if output.exists() or output.is_symlink():raise ValueError('POSITION_OUTPUT_EXISTS')
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
        patches=COMPONENTS[:1] if observation_only else COMPONENTS
        for name in patches:
            subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/(name+'.patch'))],cwd=stage/'src',check=True,capture_output=True,text=True)
        if not observation_only:
            for package in ('robot_control','dual_arm_control','servo_control'):
                for name in ('position_session_contract.h','position_connection_diagnostics.h'):
                    shutil.copy2(ASSETS/name,stage/'src'/package/'include'/name)
            for package in ('robot_control','dual_arm_control'):
                shutil.copy2(ASSETS/'position_session_ros.h',stage/'src'/package/'include/position_session_ros.h')
        if with_impedance:
            subprocess.run(['patch','-p1','--batch','--forward','--fuzz=0','-i',str(ASSETS/'impedance.patch')],cwd=stage/'src',check=True,capture_output=True,text=True)
            patches=(*patches,'impedance')
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
            driver_started=False,hardware_commands_sent=False,full_build_verified=False,
            stop_policy='unconfirmed',execution_mode='position',impedance_available=with_impedance,
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
    a=p.parse_args()
    print(prepare(a.source,a.output,observation_only=a.observation_only,with_impedance=a.with_impedance))

#!/usr/bin/env python3
"""Add reviewed upper/middle connection behavior to the existing position-session copy.

Never connects to a robot, starts ROS, builds, or modifies the input tree.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

ASSETS = Path(__file__).resolve().parent/'snowbot_vendor'
BASELINE = {
  "ctl_msgs/msg/PositionSessionMeta.msg": "b601907c277a5b7851cca11425557c9a6ac5509fdf36d9975eb8a67ee16d3d08",
  "ctl_msgs/msg/PositionSessionState.msg": "f6aec892025969f84a906d8a10ff0d4b9607293b408cefb7fce3019c1d2d3bf1",
  "dual_arm_control/include/dual_arm_control/msgPublish.h": "f3e69b90bd32af3bb73ffd5614fa59e0634deafda6aa146765cad9aaf781ae2c",
  "dual_arm_control/src/controller/ik_solver_wrapper.cpp": "8c47d72f84b98c3278223cfb2494a138db22ce31032c01936298a4657f231a95",
  "dual_arm_control/src/dual_arm_control/msgPublish.cpp": "5b6df5f1b75bd236c4943ed5719d702b80cd5e55b892c33edd7e1b2b08d2b2d5",
  "robot_control/include/robot_control/msgPublisher.h": "e8eea2e0594ee98dc1ac7b63d6571a2113e39fa66d6c53ce4fdde9edf7a361f4",
  "robot_control/src/robot_control/msgPublisher.cpp": "92e02d971d1c01e6eddb9c08ea0e4d673aee040d1aff5d32c5fdc36687294182",
  "ctl_msgs/msg/PositionObservation.msg": "16e9f6cddca9f031e91573195f2d8c1fc9612529d3eaba0cd11787b715032a2c",
  "robot_control/include/position_session_contract.h": "2364cc5c40e594b5765355629137427138b2886c60a52551770397fbfed294e8",
  "dual_arm_control/include/position_session_contract.h": "2364cc5c40e594b5765355629137427138b2886c60a52551770397fbfed294e8",
  "servo_control/include/position_session_contract.h": "2364cc5c40e594b5765355629137427138b2886c60a52551770397fbfed294e8"
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source, output):
    source, output = Path(source).resolve(), Path(output).absolute()
    if not source.is_dir() or source == output or source in output.parents or output in source.parents:
        raise ValueError('CONNECTION_SOURCE_OUTPUT_OVERLAP_OR_MISSING')
    if output.exists() or output.is_symlink():
        raise ValueError('CONNECTION_OUTPUT_EXISTS')
    if any(p.is_symlink() for p in source.rglob('*')):
        raise ValueError('CONNECTION_SOURCE_SYMLINK')
    for relative, expected in BASELINE.items():
        if not (source/relative).is_file() or digest(source/relative) != expected:
            raise ValueError('CONNECTION_POSITION_SESSION_BASELINE_MISMATCH:'+relative)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.snowbot-connection-', dir=output.parent))
    try:
        shutil.copytree(source, stage/'src')
        subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                        '-i', str(ASSETS/'connection_118.patch')], cwd=stage/'src',
                       check=True, capture_output=True, text=True)
        for package in ('robot_control', 'dual_arm_control'):
            for name in ('state', 'ros'):
                shutil.copyfile(ASSETS/('connection_'+name+'.h'),
                    stage/'src'/package/'include'/('bindu_connection_'+name+'.h'))
        # G8 must not change servo, the original wire ABI, or G4/G5 algorithms.
        for path in source.rglob('*'):
            if not path.is_file(): continue
            rel = path.relative_to(source)
            if (rel.parts[0] == 'servo_control' or rel.name in (
                'RetInformation.msg', 'JointPositionCommand.msg', 'MidNodeCtlMsgSend.msg',
                'MidNodeCtlMsgRet.msg', 'CtlInformation.msg', 'position_session_contract.h')):
                if digest(path) != digest(stage/'src'/rel):
                    raise ValueError('CONNECTION_PROTECTED_SOURCE_CHANGED:'+str(rel))
        changes = {str(p.relative_to(stage/'src')):dict(before=digest(source/p.relative_to(stage/'src'))
                if (source/p.relative_to(stage/'src')).exists() else None, after=digest(p))
            for p in (stage/'src').rglob('*') if p.is_file() and
                (not (source/p.relative_to(stage/'src')).exists() or
                 digest(p) != digest(source/p.relative_to(stage/'src')))}
        manifest = dict(schema=1, source='reviewed_position_session_connection_increment',
            source_files=BASELINE, files=changes,
            assets={name:digest(ASSETS/name) for name in ('connection_118.patch','connection_state.h','connection_ros.h')},
            native_messages_unchanged=True, servo_sources_unchanged=True,
            source_is_latest_colleague_version=False, hardware_started=False,
            full_driver_build_verified=False)
        (stage/'connection_release.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
        stage.rename(output)
    except BaseException:
        shutil.rmtree(stage,ignore_errors=True)
        raise
    return output


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path,help='已应用 observation/takeover/timing/session 的本机源码根目录')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(prepare(args.source,args.output))

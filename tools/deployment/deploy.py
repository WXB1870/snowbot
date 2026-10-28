"""Snowbot x86/Jazzy assembly. prepare/build never start a device or ROS node."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT/'tools/deployment/snowbot_vendor/position_session'
TARGETS = ('snowbot_servo_control', 'snowbot_dual_arm_control', 'robot_control')


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def locked_bindu(destination, *, acquire=False):
    """Only the managed clone may advance; user checkouts are verified, never edited."""
    dependency = json.loads((ROOT/'dependencies.repos').read_text())['repositories']['bindu']
    version = dependency['version']
    if dependency['type'] != 'git' or not re.fullmatch('[0-9a-f]{40}', version):
        raise ValueError('dependencies.repos must lock Bindu to a full commit')
    managed = ROOT/'artifacts/dependencies/bindu'
    path = destination or managed
    created = False
    if not path.exists():
        if not acquire or destination:
            raise ValueError('Bindu source missing; run prepare first')
        path.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(['git', 'clone', '--no-checkout', dependency['url'], str(path)], check=True)
        created = True
    if Path(git(path, 'rev-parse', '--show-toplevel')).resolve() != path.resolve():
        raise ValueError('Bindu must be a repository root')
    if created or git(path, 'rev-parse', 'HEAD') != version:
        if destination or not acquire:
            raise ValueError('Existing Bindu checkout does not match dependencies.repos; left unchanged')
        if not created and git(path, 'status', '--porcelain'):
            raise ValueError('Managed Bindu has local changes; left unchanged')
        if git(path, 'remote', 'get-url', 'origin') != dependency['url']:
            raise ValueError('Managed Bindu origin mismatch; left unchanged')
        available = subprocess.run(['git', '-C', str(path), 'cat-file', '-e', version+'^{commit}'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
        if not available:
            subprocess.run(['git', '-C', str(path), 'fetch', 'origin', version], check=True)
        subprocess.run(['git', '-C', str(path), 'checkout', '--detach', version], check=True)
    # Docs and unrelated local notes need not invalidate the selected executable source.
    if git(path, 'status', '--porcelain', '--', 'src', 'tools'):
        raise ValueError('Bindu executable source has local changes; commit before assembly')
    return path.resolve()


def source_environment(bindu):
    env = os.environ.copy()
    # Source-only code generation needs stdlib modules, no ROS or native loading.
    env['PYTHONPATH'] = os.pathsep.join(str(p.parent) for p in (bindu/'src').glob('*/*/package.xml'))
    env['BINDU_APPLICATION_SOURCE'] = str(ROOT)
    return env


def x86(bindu, workspace, command, *args):
    subprocess.run([sys.executable, '-m', 'tools.deployment.x86_real', command,
                    '--workspace', str(workspace/'upper'), *map(str, args)],
                   cwd=bindu, env=source_environment(bindu), check=True)


def assemble(bindu, workspace):
    from tools.deployment import prepare_position_session as position
    robot = ROOT/'src/snowbot_bringup/config/robots/snowbot'
    model = ROOT/'src/snowbot_description/urdf/snowbot_control_real.urdf'
    tool = robot/'bimanual_snowbot_real.json'
    native = bindu/'src/control/bindu_execution/native'
    inputs = [Path(position.__file__), ROOT/'dependencies.repos', ROOT/'vendor/snowbot/baseline.json', model, tool,
              *sorted(ASSETS.glob('*')), *(native/n for n in position.NATIVE_FILES),
              bindu/'src/capabilities/bindu_kinematics/bindu_kinematics/model_codegen.py']
    signature = hashlib.sha256(''.join(digest(p) for p in inputs if p.is_file()).encode()).hexdigest()
    vendor = workspace/'vendor'
    identity = workspace/'assembly.json'
    if identity.exists():
        saved = json.loads(identity.read_text())
        for name, expected in saved['files'].items():
            if not (vendor/name).is_file() or digest(vendor/name) != expected:
                raise ValueError('Generated vendor changed: '+name+'; preserve it and use a new --workspace')
        if set(saved['files']) != {str(p.relative_to(vendor)) for p in (vendor/'src').rglob('*') if p.is_file()}:
            raise ValueError('Generated vendor file list changed; use a new --workspace')
        if saved['signature'] == signature:
            return vendor
        # Preserve the previous generated source and all build caches on recipe updates.
        backup = workspace/('vendor-before-'+saved['signature'][:12])
        if backup.exists():
            raise ValueError('Vendor backup already exists; use a new --workspace')
        vendor.rename(backup)
    elif vendor.exists():
        raise ValueError('Unknown vendor directory; left unchanged; use a new --workspace')
    workspace.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(ROOT/'tools/deployment/prepare_position_session.py'),
                    '--output', str(vendor), '--execution-source', str(native),
                    '--prediction-urdf', str(model), '--prediction-tool', str(tool)],
                   cwd=ROOT, env=source_environment(bindu), check=True)
    # Leave site-owned parameters intact on rebuilds, including changed recipes.
    config = workspace/'vendor-position.yaml'
    if not config.exists():
        config.write_bytes((ASSETS/'target_stream.yaml').read_bytes())
    files = {str(p.relative_to(vendor)): digest(p) for p in (vendor/'src').rglob('*') if p.is_file()}
    identity.write_text(json.dumps(dict(signature=signature, files=files), indent=2)+'\n')
    return vendor




def require_stopped(bindu):
    # Reuse the framework's read-only process guard before replacing any binaries.
    subprocess.run([sys.executable, '-m', 'tools.deployment.process_guard'],
                   cwd=bindu, env=source_environment(bindu), check=True)


def prepare_ros_dependencies(bindu, vendor):
    # Resolve only the required package closure across all three source roots.
    roots = [vendor/'src', bindu/'src', ROOT/'src']
    paths = subprocess.check_output(['/usr/bin/colcon', 'list', '--base-paths',
        *map(str, roots), '--packages-up-to', *TARGETS, 'snowbot_bringup',
        'bindu_bimanual_qp', '--paths-only'], text=True).splitlines()
    subprocess.run(['/usr/bin/rosdep', 'install', '--from-paths', *paths, '--ignore-src',
        '--rosdistro', 'jazzy', '--skip-keys', 'pinocchio proxsuite', '-y'], check=True)


def vendor_build(vendor, workspace):
    # A clean, vendor-only shell prevents numerical venv libraries entering servo.
    env = {key: value for key, value in os.environ.items() if key in
           ('HOME', 'USER', 'LANG', 'TERM', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
            'FASTDDS_BUILTIN_TRANSPORTS', 'CMAKE_BUILD_PARALLEL_LEVEL')}
    env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    script = 'source /opt/ros/jazzy/setup.bash; export LD_LIBRARY_PATH=/opt/openrobots/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}; exec "$@"'
    subprocess.run(['bash', '--noprofile', '--norc', '-ec', script, 'snowbot-build',
                    '/usr/bin/python3', '/usr/bin/colcon', '--log-base', str(workspace/'vendor-log'), 'build',
                    '--base-paths', str(vendor/'src'), '--build-base', str(workspace/'vendor-build'),
                    '--install-base', str(vendor/'install'), '--executor', 'sequential',
                    '--packages-up-to', *TARGETS, '--cmake-args', '-DCMAKE_BUILD_TYPE=Release',
                    '-DPython3_EXECUTABLE=/usr/bin/python3', '-DPYTHON_EXECUTABLE=/usr/bin/python3'],
                   cwd=workspace, env=env, check=True)
    for package, executable in zip(TARGETS, ('servo_node', 'dual_arm_control_node', 'robot_control_node')):
        if not (vendor/'install'/package/'lib'/package/executable).is_file():
            raise ValueError('Vendor executable missing: '+package+'/'+executable)


def receipt(bindu, workspace):
    vendor = workspace/'vendor'
    # Resolve the installed, effective site recipe, including local model/profile overrides.
    inspect = """import hashlib,json,sys
from pathlib import Path
from bindu_runtime.teleop_deployment import load_deployment
from bindu_runtime.robot_setup import RobotSetup
from bindu_contracts.resources import package_share
site=Path(sys.argv[1]);_,args=load_deployment(site,package_share('bindu_runtime'))
robot=RobotSetup.load(args['robot_config'])
paths={site,Path(args['robot_config']),robot.profile_path,robot.urdf,robot.display_urdf,robot.tool_config}
paths.update(Path(args[k]) for k in ('pico_config','gripper_config','base_config','cert_file') if args.get(k))
print(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p is not None}))
"""
    configs = json.loads(subprocess.check_output(['bash', '--noprofile', '--norc', '-ec',
        'source "$1"; shift; exec python3 "$@"', 'snowbot-receipt', str(workspace/'upper/env.sh'),
        '-c', inspect, str(workspace/'upper/site.json')], text=True))
    configs[str(workspace/'vendor-position.yaml')] = digest(workspace/'vendor-position.yaml')
    data = dict(schema=1, snowbot=dict(commit=git(ROOT, 'rev-parse', 'HEAD'),
                tracked_dirty=bool(git(ROOT, 'status', '--porcelain', '--untracked-files=no'))),
                bindu=dict(commit=git(bindu, 'rev-parse', 'HEAD'), path=str(bindu)),
                dependency_lock_sha256=digest(ROOT/'dependencies.repos'),
                vendor_manifest_sha256=digest(vendor/'position_release.json'),
                configuration_hashes=configs,
                software_build_completed=True, driver_started=False, physical_acceptance=False)
    (workspace/'deployment-manifest.json').write_text(json.dumps(data, indent=2)+'\n')


def require_platform():
    import platform
    if (platform.system() != 'Linux' or platform.machine() != 'x86_64' or
        platform.freedesktop_os_release().get('ID') != 'ubuntu' or
        platform.freedesktop_os_release().get('VERSION_ID') != '24.04'):
        raise ValueError('Full Snowbot assembly requires Ubuntu 24.04 x86_64 / ROS 2 Jazzy')
    for path in ('/opt/ros/jazzy/setup.bash', '/usr/bin/colcon',
                 '/opt/openrobots/lib/libpinocchio_parsers.so.4.1.0',
                 '/opt/openrobots/lib/libpinocchio_default.so.4.1.0'):
        if not Path(path).is_file():
            raise ValueError('Install system/ROS/vendor prerequisites first: '+path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'build', 'start', 'plan'))
    parser.add_argument('--workspace', type=Path, default=ROOT/'artifacts/robot')
    parser.add_argument('--bindu-source', type=Path, help='Verify an existing matching checkout; never modify it')
    options, extra = parser.parse_known_args(argv)
    workspace = options.workspace.expanduser().resolve()
    existing = options.bindu_source.expanduser().resolve() if options.bindu_source else None
    try:
        if extra and options.command != 'start':
            raise ValueError('Extra arguments are only accepted by start')
        if options.command == 'plan':
            bindu = locked_bindu(existing)
            print(json.dumps(dict(snowbot=git(ROOT, 'rev-parse', 'HEAD'), bindu=git(bindu, 'rev-parse', 'HEAD'),
                workspace=str(workspace), vendor_packages=TARGETS, upper_command=['tools.deployment.x86_real',
                'build', '--workspace', str(workspace/'upper')], driver_started=False), indent=2))
            return 0
        if options.command in ('prepare', 'build'):
            require_platform()
        bindu = locked_bindu(existing, acquire=options.command in ('prepare', 'build'))
        if options.command == 'start':
            # x86_real start already performs the runtime precheck exactly once.
            x86(bindu, workspace, 'start', *extra)
            return 0
        if options.command == 'build' and not (workspace/'upper/deployment.json').is_file():
            raise ValueError('Run prepare once before build')
        require_stopped(bindu)
        vendor = assemble(bindu, workspace)
        if options.command == 'prepare':
            prepare_ros_dependencies(bindu, vendor)
        vendor_build(vendor, workspace)
        if options.command == 'prepare':
            x86(bindu, workspace, 'prepare', '--vendor-setup', vendor/'install/local_setup.bash',
                '--application-source', ROOT)
        x86(bindu, workspace, 'build')
        receipt(bindu, workspace)
        print('Built without starting hardware. Site configuration: '+str(workspace/'upper/site.json'))
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print('Snowbot deployment: '+str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

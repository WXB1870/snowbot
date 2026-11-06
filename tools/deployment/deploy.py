"""Snowbot x86/Jazzy assembly. prepare/build never start a device or ROS node."""
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT/'tools/deployment/snowbot_vendor/position_session'
MACHINES = ROOT/'src/snowbot_bringup/config/deployments/machines'
TARGETS = ('snowbot_servo_control', 'snowbot_dual_arm_control', 'robot_control')


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@contextmanager
def build_stage(name):
    started = time.monotonic()
    print('DEPLOY '+name+' START', flush=True)
    try:
        yield
    except BaseException:
        print(f'DEPLOY {name} FAILED elapsed={time.monotonic()-started:.1f}s', flush=True)
        raise
    else:
        print(f'DEPLOY {name} DONE elapsed={time.monotonic()-started:.1f}s', flush=True)


def machine_config(workspace, name=None):
    if name is None:
        profile = workspace/'upper/deployment.json'
        if profile.exists():
            name = json.loads(profile.read_text()).get('machine_profile')
    return name, json.loads((MACHINES/(name+'.json')).read_text()) if name else {}


def configure_machine(workspace, name, config):
    if name is None:
        return  # Existing deployments without a machine profile keep their site settings.
    profile = workspace/'upper/deployment.json'
    data = json.loads(profile.read_text())
    data['machine_profile'] = name
    profile.write_text(json.dumps(data, indent=2)+'\n')
    site = json.loads((MACHINES.parent/'snowbot_vr_real.json').read_text())
    arguments = site['arguments']
    arguments.update(config['arguments'])
    for key in ('cert_file', 'key_file'):
        if arguments[key]:
            arguments[key] = str(Path(arguments[key]).expanduser())
    (workspace/'upper/site.json').write_text(json.dumps(site, indent=2)+'\n')
    parameters = (ASSETS/'target_stream.yaml').read_text().rstrip()+'\n'
    parameters += '    ethercat_interface: '+json.dumps(config['ethercat_interface'])+'\n'
    if config.get('position_control_fifo'):
        parameters += '    position_control_fifo: true\n'
    (workspace/'vendor-position.yaml').write_text(parameters)


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


def x86(bindu, workspace, command, *args, replace=False):
    argv = [sys.executable, '-m', 'tools.deployment.x86_real', command,
            '--workspace', str(workspace/'upper'), *map(str, args)]
    env = source_environment(bindu)
    if replace:
        os.chdir(bindu)
        os.execvpe(sys.executable, argv, env)
    subprocess.run(argv, cwd=bindu, env=env, check=True)


def assemble(bindu, workspace, *, with_impedance=None):
    from tools.deployment import prepare_position_session as position
    robot = ROOT/'src/snowbot_bringup/config/robots/snowbot'
    model = ROOT/'src/snowbot_description/urdf/snowbot_control.urdf'
    tool = robot/'bimanual_snowbot_real.json'
    native = bindu/'src/control/bindu_execution/native'
    # The receipt records paired revisions; generation depends on actual inputs.
    inputs = [Path(position.__file__), ROOT/'vendor/snowbot/baseline.json', model, tool,
              *sorted(ASSETS.glob('*')), *(native/n for n in position.NATIVE_FILES),
              bindu/'src/capabilities/bindu_kinematics/bindu_kinematics/model_codegen.py']
    vendor = workspace/'vendor'
    identity = workspace/'assembly.json'
    saved = json.loads(identity.read_text()) if identity.exists() else None
    if with_impedance is None:
        with_impedance = bool(saved and saved.get('with_impedance', False))
    signature = hashlib.sha256((''.join(digest(p) for p in inputs if p.is_file())+
                                (':impedance' if with_impedance else '')).encode()).hexdigest()
    if identity.exists():
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
                    '--prediction-urdf', str(model), '--prediction-tool', str(tool),
                    *(['--with-impedance'] if with_impedance else [])],
                   cwd=ROOT, env=source_environment(bindu), check=True)
    # Leave site-owned parameters intact on rebuilds, including changed recipes.
    config = workspace/'vendor-position.yaml'
    if not config.exists():
        config.write_bytes((ASSETS/'target_stream.yaml').read_bytes())
    files = {str(p.relative_to(vendor)): digest(p) for p in (vendor/'src').rglob('*') if p.is_file()}
    if saved:
        for name, value in files.items():
            if saved['files'].get(name) == value:
                shutil.copystat(backup/name, vendor/name)
        if (backup/'install').is_dir():
            (backup/'install').rename(vendor/'install')
    identity.write_text(json.dumps(dict(signature=signature, files=files,
                                        with_impedance=with_impedance), indent=2)+'\n')
    return vendor




def require_stopped(bindu):
    # Reuse the framework's read-only process guard before replacing any binaries.
    subprocess.run([sys.executable, '-m', 'tools.deployment.process_guard'],
                   cwd=bindu, env=source_environment(bindu), check=True)


def prepare_ros_dependencies(bindu, vendor, distro):
    # Resolve only the required package closure across all three source roots.
    roots = [vendor/'src', bindu/'src', ROOT/'src']
    paths = subprocess.check_output(['/usr/bin/colcon', 'list', '--base-paths',
        *map(str, roots), '--packages-up-to', *TARGETS, 'snowbot_bringup',
        'bindu_bimanual_qp', '--paths-only'], text=True).splitlines()
    subprocess.run(['/usr/bin/rosdep', 'install', '--from-paths', *paths, '--ignore-src',
        # These legacy keys have no rosdep definition; require_platform checks
        # the installed colcon build task and JSON header before reaching here.
        '--rosdistro', distro, '--skip-keys', 'pinocchio proxsuite ament_python libnlohmann-json-dev', '-y'], check=True)


def vendor_build(vendor, workspace, distro):
    # A clean, vendor-only shell prevents numerical venv libraries entering servo.
    env = {key: value for key, value in os.environ.items() if key in
           ('HOME', 'USER', 'LANG', 'TERM', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
            'FASTDDS_BUILTIN_TRANSPORTS', 'CMAKE_BUILD_PARALLEL_LEVEL')}
    env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    script = (f'source /opt/ros/{distro}/setup.bash; '
              'export CMAKE_PREFIX_PATH=/opt/openrobots${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}; '
              'export LD_LIBRARY_PATH=/opt/openrobots/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}; exec "$@"')
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
paths.update(Path(args[k]) for k in ('pico_config','gripper_config','base_config','parameters_config','cert_file') if args.get(k))
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
    info = platform.freedesktop_os_release() if platform.system() == 'Linux' else {}
    targets = {('x86_64', '24.04'): ('jazzy', '4.1.0'),
               ('aarch64', '22.04'): ('humble', '3.9.0')}
    target = targets.get((platform.machine(), info.get('VERSION_ID')))
    if info.get('ID') != 'ubuntu' or target is None:
        raise ValueError('Full assembly requires Ubuntu 24.04 x86_64/Jazzy or Ubuntu 22.04 aarch64/Humble')
    distro, pinocchio = target
    for path in (f'/opt/ros/{distro}/setup.bash', '/usr/bin/colcon',
                 '/usr/lib/python3/dist-packages/colcon_ros/task/ament_python/build.py',
                 '/usr/include/nlohmann/json.hpp',
                 f'/opt/openrobots/lib/libpinocchio_parsers.so.{pinocchio}',
                 f'/opt/openrobots/lib/libpinocchio_default.so.{pinocchio}'):
        if not Path(path).is_file():
            raise ValueError('Install system/ROS/vendor prerequisites first: '+path)
    return distro


def keep_service_log(line, previous_wkc, now):
    if ('TARGET_PREDICTION ' in line and 'accepted=1' in line and
            line.rstrip().endswith('error=')):
        return False
    match = re.search(r'\[ECAT-MON\].*WKC=(\d+)/(\d+).*bad_wkc=(\d+)', line)
    if match:
        key = match.groups()
        if previous_wkc and previous_wkc[:3] == list(key) and now-previous_wkc[3] < 10.:
            return False
        previous_wkc[:] = [*key, now]
    return True


class ServiceConsole:
    """Select brief console messages after the complete output has been saved."""
    def __init__(self):
        self.wkc, self.repeats = {}, {}
        self.notices = set()

    def __call__(self, stream, line):
        from snowbot_bringup.visualization.snowbot_diagnostics import console_log
        now = time.monotonic()
        if not keep_service_log(line, self.wkc.setdefault(stream, []), now):
            return None
        if '[ECAT-MON]' in line:
            fields = re.findall(r'\b(?:pdo|WKC|bad_wkc|good_age_ms)=[\d./]+', line)
            return f'[{stream}] '+ ' '.join(fields)+'\n'
        if line.startswith('[LOG_REPEAT] '):
            value = json.loads(line[len('[LOG_REPEAT] '):])
            return f'[{stream}] {value["message"].split(";", 1)[0]} (repeat +{value["suppressed_repeats"]})\n'
        raw = line.removeprefix(f'[{stream}] ')
        value = console_log(raw, stream)
        if value is None:
            if re.search(r'\[EtherCAT\] \d+ slaves found|\b(?:WARN(?:ING)?|ERROR|FATAL|FAULT|FAILED|TARGET_REJECTED|DEVICE_FEEDBACK_DETAIL|Traceback|\w*Error|\w*Exception)\b|警告|错误|失败|异常|回零|回位|安全退出', raw, re.IGNORECASE):
                return line
            if 'TARGET_PREDICTION ' in raw:
                return line
            return None
        message = value['message']
        key = (value['name'], value['level'], message.split(';', 1)[0])
        if key[1:] == (30, 'Position session: legacy_source_disabled_for_target_stream'):
            if stream in self.notices:
                return None
            self.notices.add(stream)
            return f'[{stream}] INFO target_stream: legacy input ignored\n'
        previous = self.repeats.get(stream)
        if previous is not None and previous[0] == key:
            previous[2] += 1
            if now-previous[1] < 10.:
                return None
            count = previous[2]
            previous[1:] = [now, 0]
            return f'[{stream}] {key[2]} (repeat +{count}; details in log)\n'
        self.repeats[stream] = [key, now, 0]
        level = {30:'WARN', 40:'ERROR', 50:'FATAL'}[value['level']]
        return f'[{stream}] {level} {message}\n'


def wait_service(children, ready, spin, stopped, timeout=120.):
    deadline = time.monotonic()+timeout
    while True:
        if stopped.is_set():
            return False
        for name, child in children:
            code = child.poll()
            if code is not None:
                raise RuntimeError(f'{name} exited: code={code}')
        if ready():
            return True
        if time.monotonic() >= deadline:
            raise RuntimeError('STARTUP_TIMEOUT')
        spin()


def stop_services(children):
    # Base shutdown is a servo action: keep upper/middle alive until its final
    # stop/disable acknowledgement. Operator-only children still reverse.
    success = True
    ordered = sorted(reversed(children), key=lambda item: item[0] != 'servo')
    for name, child in ordered:
        if child.poll() is not None:
            continue
        print(f'STOP {name}', flush=True)
        try:
            os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError:
            continue
        try:
            code = child.wait(timeout=60.)
        except subprocess.TimeoutExpired:
            print(f'STOP_WAIT {name}: pid={child.pid}; still running, no forced kill',
                  file=sys.stderr, flush=True)
            code = child.wait()
        print(f'EXIT {name} code={code}', flush=True)
        if code not in (0, -signal.SIGINT, 128+signal.SIGINT):
            success = False
    return success


def popen_group(command, **kwargs):
    # Keep the tty session (and sudo cache), with a separate group for SIGINT.
    if sys.version_info >= (3, 11):
        return subprocess.Popen(command, process_group=0, **kwargs)
    # Run setpgid after exec: preexec_fn is unsafe with the log reader threads.
    return subprocess.Popen([sys.executable, '-c',
        'import os,sys; os.setpgid(0,0); os.execvpe(sys.argv[1],sys.argv[1:],os.environ)',
        *command], **kwargs)


def copy_service_output(name, child, writer):
    try:
        for line in child.stdout:
            writer.write_text(name, f'[{name}] '+line)
    finally:
        child.stdout.close()


def base_commands(workspace, execution_mode='impedance'):
    vendor_env = {k:v for k,v in os.environ.items() if k in
        ('HOME', 'USER', 'LANG', 'TERM', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'FASTDDS_BUILTIN_TRANSPORTS', 'IMPEDANCE_PARAMS_YAML')}
    vendor_env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    shell = ['bash', '--noprofile', '--norc', '-ec',
             'source "$1/tools/deployment/snowbot_vendor/env_118.sh" "$2/vendor"; shift 2; exec stdbuf -oL -eL "$@"',
             'snowbot-vendor', str(ROOT), str(workspace)]
    sudo = ['sudo', '-n', 'env', *[k+'='+vendor_env[k] for k in
            ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'FASTDDS_BUILTIN_TRANSPORTS', 'IMPEDANCE_PARAMS_YAML') if k in vendor_env]]
    return (
        ('servo', [*sudo, *shell, 'ros2', 'run', 'snowbot_servo_control', 'servo_node',
                   '--ros-args', '--params-file', str(workspace/'vendor-position.yaml'),
                   '-p', 'execution_mode:='+execution_mode], vendor_env),
        ('middle', [*shell, 'ros2', 'run', 'snowbot_dual_arm_control', 'dual_arm_control_node'], vendor_env),
        ('upper', [*shell, 'ros2', 'launch', 'robot_control', 'robot_control.launch.py', 'control_source:=topic'], vendor_env))


def services(bindu, workspace, execution_mode='impedance'):
    """Operator-run native chain; the upper application has its own terminal."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from snowbot_ctl_msgs.msg import PositionSessionState
    from snowbot_bringup.visualization.snowbot_diagnostics import DiagnosticWriter

    require_stopped(bindu)  # Do not start a second EtherCAT/controller chain.
    logdir = workspace/'startup-logs'/('services-'+time.strftime('%Y%m%d-%H%M%S')+f'-{os.getpid()}')
    writer = DiagnosticWriter(logdir/'process-output.jsonl', echo=ServiceConsole())
    stopped, children, readers = threading.Event(), [], []
    handlers = {s: signal.signal(s, lambda *_: stopped.set())
                for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('snowbot_primitive_services')
    states = {}
    for layer in ('servo', 'middle', 'upper'):
        node.create_subscription(PositionSessionState, '/bindu/position/'+layer+'_state',
            lambda msg, layer=layer: states.update({layer:msg}), 1)

    plan = base_commands(workspace, execution_mode)

    def ready(name):
        state = states.get(name)
        if state is not None and state.state == 'initialization_failed':
            raise RuntimeError(name+':'+state.reason)
        # feedback_ready also depends on layers which have not started yet.
        return state is not None and state.startup_complete

    result = 0
    try:
        try:
            print(f'LOGS {logdir}', flush=True)
            for name, command, env in plan:
                if stopped.is_set():
                    break
                # Separate groups for ordered signals; keep the tty session's sudo cache.
                child = popen_group(command, cwd=ROOT, env=env,
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         text=True, errors='replace')
                children.append((name, child))
                reader = threading.Thread(target=copy_service_output, args=(name, child, writer), daemon=True)
                reader.start(); readers.append(reader)
                print(f'START {name} pid={child.pid}', flush=True)
                if not wait_service(children, lambda: ready(name),
                                    lambda: rclpy.spin_once(node, timeout_sec=.05), stopped):
                    break
                print(f'INITIALIZED {name}', flush=True)
            if not stopped.is_set():
                print(f'SNOWBOT_BASE_STARTED execution_mode={execution_mode}; initialization complete, see feedback warnings', flush=True)
        except (OSError, RuntimeError) as exc:
            result = 1
            print(f'START_FAILED {name}: {exc}; startup paused, Ctrl+C for ordered exit',
                  file=sys.stderr, flush=True)
        exited = set()
        while not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
            for name, child in children:
                if child.poll() is not None and name not in exited:
                    exited.add(name); result = 1
                    print(f'SERVICE_EXITED {name}: code={child.returncode}; no automatic restart',
                          file=sys.stderr, flush=True)
    finally:
        if not stop_services(children):
            result = 1
        for reader in readers:
            reader.join()
        node.destroy_node(); rclpy.shutdown()
        writer.close()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    return result


def base_mode(states):
    if not all(name in states and states[name].startup_complete for name in ('servo', 'middle', 'upper')):
        return None
    mode = states['servo'].control_source
    if mode not in ('position', 'impedance'):
        raise RuntimeError('UNSUPPORTED_BASE_MODE:'+mode)
    return mode


def executor_command(cfg, runs, execution_mode, namespace):
    return ['ros2', 'launch', 'snowbot_bringup', 'snowbot_adapter.launch.py',
            'runtime_mode:=real', 'device_backend:=snowbot_vendor', 'namespace:='+namespace,
            'vendor_namespace:=/', 'profile:='+str(cfg/'snowbot_real_position.json'),
            'parameters_config:='+str(cfg/'runtime_parameters.json'),
            'gripper_config:='+str(cfg/'grippers.json'),
            'execution_mode:='+execution_mode, 'command_enabled:=true',
            'trajectory_max_velocity:=1.25', 'trajectory_max_acceleration:=4.5',
            'trajectory_max_jerk:=259.2',
            'recording_mode:=compact', 'output:='+str(runs)]


def replay_commands(frontend, cfg, runs, execution_mode, stamp):
    common = [sys.executable, '-u', '-m', 'snowbot_bringup.runtime.snowbot_primitive_replay',
              '--library', str(cfg/'primitive-library/manifest.json'),
              '--execution-mode', execution_mode, '--execute']
    result = []
    if frontend in ('replay', 'prepare'):
        result.append([*common, '--action', 'Prepare', '--speed', '0.5', '--output', str(runs/f'Prepare-{stamp}.jsonl')])
    if frontend in ('replay', 'menu'):
        result.append([*common, '--interactive', '--speed', '1.0', '--tool-id', 'g099-curved', '--output-dir', str(runs)])
    return result


def operator(bindu, workspace, frontend, *, home=False):
    """Wait for the base, then own either VR or the replay executor/client."""
    # Invocation-local choice, inherited by either the VR or replay executor.
    os.environ['SNOWBOT_STARTUP_HOME'] = '1' if home else '0'
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from snowbot_ctl_msgs.msg import PositionSessionState
    from bindu_interfaces.msg import RuntimeEvent
    from bindu_interfaces.srv import Lease
    from ament_index_python.packages import get_package_share_directory
    from snowbot_bringup.runtime.run_vr import startup_lock
    from snowbot_bringup.visualization.snowbot_diagnostics import DiagnosticWriter

    stopped, stop_signal = threading.Event(), []
    def interrupt(sig, _):
        stop_signal[:] = [sig]
        stopped.set()
    handlers = {s:signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('snowbot_upper_launcher')
    states = {}
    for layer in ('servo', 'middle', 'upper'):
        node.create_subscription(PositionSessionState, '/bindu/position/'+layer+'_state',
                                 lambda msg, layer=layer: states.update({layer:msg}), 1)
    children, readers, client, writer, lock = [], [], None, None, None
    mode, result = None, 0
    try:
        print('WAIT_BASE', flush=True)
        if not wait_service([], lambda:base_mode(states) is not None,
                            lambda:rclpy.spin_once(node, timeout_sec=.05), stopped):
            return 0
        mode = base_mode(states)
        print('BASE_CONNECTED execution_mode='+mode, flush=True)
        if frontend != 'vr':
            namespace = json.loads((workspace/'upper/site.json').read_text())['arguments']['namespace'].strip('/')
            lock = startup_lock(int(os.environ.get('ROS_DOMAIN_ID', '0')), namespace)
            stamp = time.strftime('%Y%m%d-%H%M%S')+f'-{os.getpid()}'
            runs = workspace/'primitive-runs'; runs.mkdir(parents=True, exist_ok=True)
            logdir = workspace/'startup-logs'/('replay-'+stamp)
            print(f'LOGS {logdir}', flush=True)
            writer = DiagnosticWriter(logdir/'process-output.jsonl', echo=ServiceConsole())
            def event(msg):
                if msg.state in ('FAILED', 'TARGET_REJECTED', 'STARTUP_HOLD_FAILED', 'DEVICE_FEEDBACK_DETAIL'):
                    writer.record_log('executor-events', dict(name=msg.component, level=40,
                        message=msg.state+':'+msg.code, text=f'[executor] {msg.state}: {msg.code}\n'), text=True)
            node.create_subscription(RuntimeEvent, '/'+namespace+'/events', event, 100)
            cfg = Path(get_package_share_directory('snowbot_bringup'))/'config/robots/snowbot'
            child = popen_group(executor_command(cfg, runs, mode, namespace),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors='replace')
            children.append(('executor', child))
            reader = threading.Thread(target=copy_service_output, args=('executor', child, writer), daemon=True)
            reader.start(); readers.append(reader)
            readiness = node.create_client(Lease, '/'+namespace+'/execution/lease')
            pending = None
            def executor_ready():
                nonlocal pending
                if not readiness.service_is_ready():
                    return False
                if pending is None:
                    pending = readiness.call_async(Lease.Request(operation='ready',
                        resources=['waist', 'left_arm', 'right_arm']))
                if not pending.done():
                    return False
                reply, pending = pending.result(), None
                if reply.code in ('STARTUP_HOLD_FAILED', 'STOP_UNCONFIRMED'):
                    raise RuntimeError(reply.code)
                return reply.ok
            if not wait_service(children, executor_ready,
                                lambda:rclpy.spin_once(node, timeout_sec=.05), stopped):
                return 0
            print('EXECUTOR_READY', flush=True)
            for command in replay_commands(frontend, cfg, runs, mode, stamp):
                # The interactive client shares this terminal's foreground group.
                client = subprocess.Popen(command)
                while client.poll() is None and not stopped.is_set():
                    rclpy.spin_once(node, timeout_sec=.05)
                    if child.poll() is not None:
                        raise RuntimeError(f'EXECUTOR_EXITED:code={child.returncode}')
                if client.poll() is None and stop_signal != [signal.SIGINT]:
                    client.send_signal(signal.SIGINT)
                result = client.wait()
                if result or stopped.is_set():
                    break
            if frontend == 'executor':
                while not stopped.is_set():
                    rclpy.spin_once(node, timeout_sec=.1)
                    if child.poll() is not None:
                        raise RuntimeError(f'EXECUTOR_EXITED:code={child.returncode}')
    except (OSError, RuntimeError) as exc:
        result = 1
        print(f'UPPER_FAILED: {exc}', file=sys.stderr, flush=True)
    finally:
        if client is not None and client.poll() is None:
            if stop_signal != [signal.SIGINT]:
                client.send_signal(signal.SIGINT)
            client.wait()  # Its existing lease-release/stop confirmation finishes first.
        if not stop_services(children):
            result = 1
        for reader in readers:
            reader.join()
        node.destroy_node(); rclpy.shutdown()
        if writer is not None:
            writer.close()
        if lock is not None:
            lock.close()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    if frontend == 'vr' and mode is not None and not result and not stopped.is_set():
        x86(bindu, workspace, 'start', '--enable-commands', '--execution-mode', mode, replace=True)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'build', 'start', 'plan', 'services', 'operator'))
    parser.add_argument('--workspace', type=Path, default=ROOT/'artifacts/robot')
    parser.add_argument('--bindu-source', type=Path, help='Verify an existing matching checkout; never modify it')
    parser.add_argument('--machine', choices=sorted(p.stem for p in MACHINES.glob('*.json')),
                        help='Select machine defaults; prepare/build remember the selection for later runs')
    parser.add_argument('--with-impedance', action='store_const', const=True, default=None,
                        help='Include the impedance backend during prepare/build; subsequent builds retain it')
    parser.add_argument('--validate', choices=('rt', 'nominal', 'session', 'all'),
                        help='Run the selected offline impedance check before prepare/build installs software')
    parser.add_argument('--no-impedance', action='store_true', help='Base startup only: select position control')
    parser.add_argument('--home', action='store_true', help='Operator startup: smoothly zero arms and waist after powered hold')
    parser.add_argument('--frontend', choices=('vr', 'replay', 'menu', 'prepare', 'executor'), default='replay')
    options, extra = parser.parse_known_args(argv)
    workspace = options.workspace.expanduser().resolve()
    existing = options.bindu_source.expanduser().resolve() if options.bindu_source else None
    try:
        machine_name, machine = machine_config(workspace, options.machine)
        os.environ.setdefault('ROS_DOMAIN_ID', str(machine.get('ros_domain_id', 0)))
        if extra and options.command != 'start':
            raise ValueError('Extra arguments are only accepted by start')
        if options.no_impedance and options.command != 'services':
            raise ValueError('--no-impedance belongs to the base startup only')
        if options.home and options.command != 'operator':
            raise ValueError('--home belongs to terminal 2 (operator) only')
        if options.validate and options.command not in ('prepare', 'build'):
            raise ValueError('--validate belongs to prepare/build only')
        if options.command == 'plan':
            bindu = locked_bindu(existing)
            print(json.dumps(dict(snowbot=git(ROOT, 'rev-parse', 'HEAD'), bindu=git(bindu, 'rev-parse', 'HEAD'),
                workspace=str(workspace), vendor_packages=TARGETS, upper_command=['tools.deployment.x86_real',
                'build', '--workspace', str(workspace/'upper')], driver_started=False), indent=2))
            return 0
        if options.command in ('prepare', 'build'):
            distro = require_platform()
        bindu = locked_bindu(existing, acquire=options.command in ('prepare', 'build'))
        if options.command == 'services':
            return services(bindu, workspace, 'position' if options.no_impedance else 'impedance')
        if options.command == 'operator':
            return operator(bindu, workspace, options.frontend, home=options.home)
        if options.command == 'start':
            # x86_real start already performs the runtime precheck exactly once.
            x86(bindu, workspace, 'start', *extra)
            return 0
        if options.command == 'build' and not (workspace/'upper/deployment.json').is_file():
            raise ValueError('Run prepare once before build')
        require_stopped(bindu)
        with build_stage('assemble'):
            vendor = assemble(bindu, workspace, with_impedance=options.with_impedance)
        if options.command == 'prepare':
            with build_stage('dependencies'):
                prepare_ros_dependencies(bindu, vendor, distro)
        if options.validate:
            with build_stage('validate-'+options.validate):
                subprocess.run(['/usr/bin/python3', str(ROOT/'tests/integration/validate_session_impedance.py'),
                    '--source', str(vendor/'src'), '--prefix', '/opt/openrobots',
                    '--case', options.validate, '--require-production-poll',
                    '--build-dir', str(workspace/'validation/impedance')], check=True)
        with build_stage('vendor-build'):
            vendor_build(vendor, workspace, distro)
        if options.command == 'prepare':
            with build_stage('upper-prepare'):
                x86(bindu, workspace, 'prepare', '--vendor-setup', vendor/'install/local_setup.bash',
                    '--application-source', ROOT)
        configure_machine(workspace, machine_name, machine)
        with build_stage('upper-build'):
            x86(bindu, workspace, 'build')
        with build_stage('receipt'):
            receipt(bindu, workspace)
        print('Built without starting hardware. Site configuration: '+str(workspace/'upper/site.json'))
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print('Snowbot deployment: '+str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

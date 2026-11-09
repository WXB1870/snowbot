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


def sched_event_filters(tids, *, startup_window=False):
    tids = sorted(set(map(int, tids)))
    filters = {
        'sched_switch': ' || '.join(f'prev_pid == {tid} || next_pid == {tid}' for tid in tids) or 'prev_pid < 0',
        'sched_wakeup': ' || '.join(f'pid == {tid}' for tid in tids) or 'pid < 0'}
    if startup_window == 'follow':
        # Keep the switches between peers while the adapter is runnable off-CPU.
        target = filters['sched_wakeup']+' || comm == "snowbot_adapter"'
        return dict(sched_switch='0', sched_wakeup=target, sched_migrate_task=target)
    if startup_window:
        # Catch the adapter by its kernel comm before its first log arrives.
        filters['sched_switch'] += ' || prev_comm == "snowbot_adapter" || next_comm == "snowbot_adapter"'
        filters['sched_wakeup'] += ' || comm == "snowbot_adapter"'
    return filters


def sched_trace_window(text, fault_ns, before_s=2.):
    start = fault_ns/1e9-before_s
    lines, times = [], []
    for line in text.splitlines():
        event = re.search(r'\s(\d+\.\d+): (?:sched:)?sched_(?:switch|wakeup|migrate_task):', line)
        if line.startswith('#'):
            lines.append(line)
        elif event and float(event[1]) >= start:
            lines.append(line); times.append(float(event[1]))
    return '\n'.join(lines)+'\n', dict(first_event_s=min(times) if times else None,
        last_event_s=max(times) if times else None, events=len(times))


def follow_pdo_fault(detail, armed_ns, now_ns):
    return (detail['state'] == 'DEVICE_COMMAND_DETAIL'
            and detail['code'] in ('SNOWBOT_PDO_STALE', 'SNOWBOT_PDO_NOT_ADVANCING')
            and detail.get('task_id', '').startswith('pico_')
            and armed_ns <= detail['steady_ns'] <= now_ns)


def trace_scheduling(workspace, *, tracefs=None, startup_window=False):
    """One diagnostic capture; never launches, signals or controls a robot process."""
    if os.geteuid() != 0:
        raise ValueError('trace-scheduling需要root；请现场用sudo运行该采集命令')
    if tracefs is None:
        tracefs = next((p for p in (Path('/sys/kernel/tracing'),Path('/sys/kernel/debug/tracing'))
                        if (p/'available_events').is_file()), None)
    if tracefs is None:
        raise ValueError('tracefs不可用；未修改全局跟踪设置')
    instance = tracefs/'instances'/('snowbot-sched-'+str(os.getpid()))
    instance.mkdir()
    stopped = threading.Event()
    handlers = {s:signal.signal(s,lambda *_:stopped.set()) for s in (signal.SIGINT,signal.SIGTERM)}
    def write(name, value): (instance/name).write_text(str(value))
    source, control_source, offsets, tids, fault_ns, noticed_ns = None, None, {}, {}, None, None
    identified_ns={}
    deadline, configured, simulation, before_s = None, set(), False, 15. if startup_window else 2.
    startup_deadline=None
    buffer_kb=8192 if startup_window else 2048
    follow = startup_window == 'follow'
    post_notice_s = .02 if follow else .2
    if follow:
        before_s, buffer_kb = .20, 1024
    armed_ns = time.monotonic_ns()
    events = sched_event_filters([], startup_window=startup_window)
    try:
        write('tracing_on',0); write('current_tracer','nop'); write('trace_clock','mono')
        write('buffer_size_kb',buffer_kb); write('options/overwrite',1)
        for event, expression in events.items():
            write('events/sched/'+event+'/filter',expression)
            write('events/sched/'+event+'/enable',1)
        write('trace',''); write('tracing_on',1)
        print('SCHED_TRACE_READY：独立内核环形缓冲；请在其他终端启动base。Ctrl+C仅结束采集。',flush=True)
        while not stopped.is_set():
            candidates = list((workspace/'startup-logs').glob('services-*/servo.log'))
            latest = max(candidates,key=lambda p:p.stat().st_mtime_ns) if candidates else None
            if latest is not None and latest != source and noticed_ns is None:
                source, control_source, offsets, tids, identified_ns = latest, None, {}, {}, {}
                write('tracing_on',0)
                for event,expression in sched_event_filters([],startup_window=startup_window).items():write('events/sched/'+event+'/filter',expression)
                write('trace','');write('tracing_on',1);configured=set()
            if source is not None:
                controls = list((workspace.parent/'dependencies/bindu/log/vr').glob('*/control.log'))
                latest_control = max(controls,key=lambda p:p.stat().st_mtime_ns) if controls else None
                if latest_control != control_source and noticed_ns is None:
                    if control_source is not None:offsets.pop(str(control_source),None)
                    control_source=latest_control;tids.pop('adapter',None);identified_ns.pop('adapter',None)
                    if set(tids.values()) != configured:
                        for event,expression in sched_event_filters(tids.values(),startup_window=startup_window).items():
                            write('events/sched/'+event+'/filter',expression)
                        configured=set(tids.values())
                lines = []
                paths = [source,source.with_name('middle.log')]
                if control_source is not None:paths.append(control_source)
                for path in paths:
                    if not path.exists():continue
                    with path.open() as stream:
                        stream.seek(offsets.get(str(path), 0))
                        while True:
                            line=stream.readline()
                            if not line or not line.endswith('\n'):break
                            lines.append(line);offsets[str(path)]=stream.tell()
                for line in lines:
                    thread = re.search(r'\[SCHED-THREAD\] role=(pdo|control|middle|adapter) pid=(\d+) tid=(\d+)',line)
                    if thread and Path('/proc/'+thread[3]).exists():
                        if (startup_window and startup_window not in ('hold','follow') and startup_deadline is None and
                                (startup_window!='adapter' or thread[1]=='adapter')):
                            startup_deadline=time.monotonic_ns()+15000000000
                        if tids.get(thread[1])!=int(thread[3]):identified_ns[thread[1]]=time.monotonic_ns()
                        tids[thread[1]] = int(thread[3])
                        if thread[1] in ('pdo','control') and tids.get('servo_ros')!=int(thread[2]):
                            tids['servo_ros']=int(thread[2]);identified_ns['servo_ros']=time.monotonic_ns()
                        if set(tids.values()) != configured:
                            for event, expression in sched_event_filters(tids.values(),startup_window=startup_window).items():
                                write('events/sched/'+event+'/filter',expression)
                            configured=set(tids.values())
                            print('SCHED_TRACE_ARMED '+json.dumps(tids),flush=True)
                    if 'adapter' in tids and 'DEVICE_FAULT ' in line and noticed_ns is None:
                        detail=json.loads(line.split('DEVICE_FAULT ',1)[1])
                        now_ns = time.monotonic_ns()
                        if follow and not follow_pdo_fault(detail, armed_ns, now_ns):
                            continue
                        if detail['state']=='DEVICE_COMMAND_DETAIL':
                            noticed_ns=now_ns;deadline=noticed_ns+round(post_notice_s*1e9)
                            fault_ns=detail['steady_ns'];simulation=bool(detail.get('simulation',False))
                    if follow:
                        continue
                    if configured and any(marker in line for marker in (
                            '[ECAT-FAULT]', 'POSITION_CYCLE_FIRST_FAULT', 'POSITION_GATE_INTERRUPT',
                            'POSITION_FEEDBACK_RESET', 'POSITION_INTERFACE_INIT_DELAY', 'POSITION_WATCHDOG_FAULT')) and noticed_ns is None:
                        noticed_ns=time.monotonic_ns();deadline=noticed_ns+200000000
                        simulation='simulation=1' in line
                        gap=re.search(r'(?:previous_progress_gap_ms|wall_ms)=([\d.]+)',line)
                        if gap:before_s=max(before_s,float(gap[1])/1000.+.2)
                    first = any(marker in line for marker in ('first_bad=1', 'POSITION_CYCLE_FIRST_FAULT',
                        'POSITION_GATE_INTERRUPT', 'POSITION_FEEDBACK_RESET', 'POSITION_INTERFACE_INIT_DELAY', 'POSITION_WATCHDOG_FAULT'))
                    stamp = re.search(r'(?:recv_end_ns|steady_ns)=(\d+)',line) if first else None
                    if stamp and noticed_ns is not None: fault_ns=int(stamp[1])
            if deadline is not None and time.monotonic_ns() >= deadline:break
            if startup_deadline is not None and time.monotonic_ns() >= startup_deadline:break
            time.sleep(.02)
        write('tracing_on',0)
        frozen_ns=time.monotonic_ns()
        if fault_ns is None:fault_ns=noticed_ns or frozen_ns
        text,coverage=sched_trace_window((instance/'trace').read_text(),fault_ns,before_s=before_s)
        if follow:
            coverage['per_cpu_stats'] = {p.parent.name:p.read_text()
                                         for p in sorted((instance/'per_cpu').glob('cpu*/stats'))}
        thread_info={}
        recorded=set(map(int,re.findall(r'(?:prev_pid|next_pid|\bpid)=(\d+)',text))) | set(tids.values())
        for tid in sorted(recorded-{0}):
            try:
                status=(Path('/proc')/str(tid)/'status').read_text()
                thread_info[tid]=dict(comm=(Path('/proc')/str(tid)/'comm').read_text().strip(),
                    policy=os.sched_getscheduler(tid),priority=os.sched_getparam(tid).sched_priority,
                    cpus_allowed=re.search(r'Cpus_allowed_list:\s*(\S+)',status)[1])
                if follow:
                    tgid = int(re.search(r'Tgid:\s*(\d+)',status)[1])
                    thread_info[tid].update(tgid=tgid, process=(Path('/proc')/str(tgid)/'comm').read_text().strip())
            except (FileNotFoundError,ProcessLookupError):thread_info[tid]=dict(exited=True)
        output=workspace/'scheduler-traces'/(time.strftime('%Y%m%d-%H%M%S')+'-'+str(os.getpid()))
        output.mkdir(parents=True)
        (output/'sched.trace').write_text(text)
        info=dict(clock='mono',fault_ns=fault_ns,noticed_ns=noticed_ns,frozen_ns=frozen_ns,
            requested_before_s=before_s,post_notice_s=post_notice_s,buffer_kb_per_cpu=buffer_kb,
            startup_window=startup_window,startup_deadline_ns=startup_deadline,
            tids=tids,tids_identified_ns=identified_ns,
            adapter_comm_filter='snowbot_adapter' if startup_window else None,
            source_log=str(source) if source else None,
            source_logs=[str(p) for p in paths] if source else [],coverage=coverage,threads=thread_info,
            manual_stop=stopped.is_set(),simulation=simulation,hardware_started=False)
        (output/'metadata.json').write_text(json.dumps(info,indent=2)+'\n')
        print('SCHED_TRACE_SAVED '+str(output),flush=True)
        return output
    finally:
        write('tracing_on',0)
        for event in events:
            if (instance/'events/sched'/event/'enable').exists():
                write('events/sched/'+event+'/enable',0)
        instance.rmdir()
        for sig,handler in handlers.items():signal.signal(sig,handler)


def failure_banner(message):
    text = '\n'+'!'*72+'\n!!! '+message.rstrip()+'\n'+'!'*72+'\n'
    return '\033[1;31m'+text+'\033[0m' if sys.stderr.isatty() and 'NO_COLOR' not in os.environ else text


def base_wait_status(states):
    lines, fault = [], False
    for layer in ('servo', 'middle', 'upper'):
        state = states.get(layer)
        if state is None:
            lines.append(layer+': 未收到状态')
            continue
        reason = getattr(state, 'reason', '') or '-'
        status = getattr(state, 'state', 'unknown')
        fault |= status == 'initialization_failed' or bool(re.search('fault|invalid|failed|pdo_stale', reason, re.I))
        lines.append(f'{layer}: initialized={int(state.startup_complete)} '
                     f'feedback_ready={int(getattr(state,"feedback_ready",False))} state={status} reason={reason}')
    return '\n'.join(lines), fault


class ServiceConsole:
    """Select brief console messages after the complete output has been saved."""
    STARTUP_WAIT_MESSAGES = frozenset('Position session: '+reason for reason in (
        'initializing', 'publisher_missing', 'publisher_discovering',
        'publisher_found_no_message', 'middle_instance_or_state_unavailable'))

    def __init__(self, *, initializing=False):
        self.wkc, self.repeats = {}, {}
        self.notices = set()
        self.initializing = initializing

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
            if self.initializing and value.get('level') == 30 and value['message'] in self.STARTUP_WAIT_MESSAGES:
                return None
            return f'[{stream}] {value["message"].split(";", 1)[0]} (repeat +{value["suppressed_repeats"]})\n'
        raw = line.removeprefix(f'[{stream}] ')
        value = console_log(raw, stream)
        if value is None:
            if re.search(r'\b(?:ERROR|FATAL|START_FAILED|SERVICE_EXITED)\b|\[ECAT-FAULT\]', raw):
                return failure_banner(line)
            if re.search(r'\[EtherCAT\] \d+ slaves found|\b(?:WARN(?:ING)?|ERROR|FATAL|FAULT|FAILED|TARGET_REJECTED|DEVICE_FEEDBACK_DETAIL|Traceback|\w*Error|\w*Exception)\b|警告|错误|失败|异常|回零|回位|安全退出', raw, re.IGNORECASE):
                return line
            if 'TARGET_PREDICTION ' in raw:
                return line
            return None
        message = value['message']
        key = (value['name'], value['level'], message.split(';', 1)[0])
        if self.initializing and value['level'] == 30 and message in self.STARTUP_WAIT_MESSAGES:
            notice = ('startup_wait', stream)
            if notice in self.notices:
                return None
            self.notices.add(notice)
            return f'[{stream}] INFO 正在建立反馈链\n'
        if key[1:] == (30, 'Position session: legacy_source_disabled_for_target_stream'):
            if stream in self.notices:
                return None
            self.notices.add(stream)
            return f'[{stream}] INFO target_stream: legacy input ignored\n'
        fault = (value['level'] >= 40 or re.search('fault|failed|invalid', message, re.I) or
                 not self.initializing and 'pdo_stale' in message)
        previous = self.repeats.get(stream)
        if previous is not None and previous[0] == key:
            previous[2] += 1
            if now-previous[1] < 10.:
                return None
            count = previous[2]
            previous[1:] = [now, 0]
            summary = f'[{stream}] {key[2]} (repeat +{count}; details in log)\n'
            return failure_banner(summary) if fault else summary
        self.repeats[stream] = [key, now, 0]
        level = {30:'WARN', 40:'ERROR', 50:'FATAL'}[value['level']]
        text = f'[{stream}] {level} {message}\n'
        return failure_banner(text) if fault else text


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
        ('middle', [*sudo, *shell, 'ros2', 'run', 'snowbot_dual_arm_control', 'dual_arm_control_node'], vendor_env),
        ('upper', [*shell, 'ros2', 'launch', 'robot_control', 'robot_control.launch.py', 'control_source:=topic'], vendor_env))


def services(bindu, workspace, execution_mode='impedance'):
    """Operator-run native chain; the upper application has its own terminal."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from snowbot_ctl_msgs.msg import PositionSessionState
    from snowbot_bringup.visualization.snowbot_diagnostics import DiagnosticWriter

    require_stopped(bindu)  # Do not start a second EtherCAT/controller chain.
    logdir = workspace/'startup-logs'/('services-'+time.strftime('%Y%m%d-%H%M%S')+f'-{os.getpid()}')
    console = ServiceConsole(initializing=True)
    writer = DiagnosticWriter(logdir/'process-output.jsonl', echo=console)
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
            from snowbot_hardware.pdo_reader_trace import PRIVATE_DDS
            private_dds = workspace/'upper/install/snowbot_hardware'/PRIVATE_DDS
            if private_dds.is_file():
                # Use the base terminal's existing sudo ticket. This separate process
                # only observes the adapter and exits when this base process exits.
                collector = ROOT/'src/snowbot_hardware/snowbot_hardware/pdo_reader_trace.py'
                try:
                    logdir.mkdir(exist_ok=True, parents=True)
                    with (logdir/'pdo-reader.log').open('w') as log:
                        popen_group(['sudo', '-n', '/usr/bin/python3', '-I', str(collector),
                                     str(os.getpid()), str(private_dds), str(logdir)],
                                    stdout=log, stderr=subprocess.STDOUT)
                except OSError as exc:
                    print(f'PDO_READER_TRACE_UNAVAILABLE {exc}', file=sys.stderr, flush=True)
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
                print(f'SNOWBOT_BASE_STARTED execution_mode={execution_mode}; 进程初始化完成，正在确认反馈链', flush=True)
        except (OSError, RuntimeError) as exc:
            result = 1
            print(failure_banner(f'START_FAILED {name}: {exc}; 启动未完成，Ctrl+C按顺序退出；日志：{logdir}'),
                  file=sys.stderr, flush=True)
        exited = set()
        while not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
            if console.initializing and all(layer in states and states[layer].startup_complete and
                    states[layer].feedback_ready for layer in ('servo', 'middle', 'upper')):
                console.initializing = False
                print('SNOWBOT_BASE_FEEDBACK_READY 三层反馈链已就绪', flush=True)
            for name, child in children:
                if child.poll() is not None and name not in exited:
                    exited.add(name); result = 1
                    print(failure_banner(f'SERVICE_EXITED {name}: code={child.returncode}; 不自动重启；日志：{logdir}'),
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
        print('WAIT_BASE：正在连接底层，请稍候…', flush=True)
        last_wait, last_wait_at = None, time.monotonic()
        def base_ready():
            nonlocal last_wait, last_wait_at
            ready = base_mode(states) is not None
            if not ready:
                detail, fault = base_wait_status(states)
                now = time.monotonic()
                if detail != last_wait and (fault or now-last_wait_at >= 5.):
                    message = 'WAIT_BASE：等待底层就绪\n'+detail
                    if fault:
                        message += '\n请查看终端1的首个故障。'
                    print(failure_banner(message) if fault else message, flush=True)
                    last_wait, last_wait_at = detail, now
            return ready
        if not wait_service([], base_ready,
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
        print(failure_banner(f'UPPER_FAILED: {exc}; 请查看终端1及上述底层状态'), file=sys.stderr, flush=True)
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


def open_grippers(workspace):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from ament_index_python.packages import get_package_share_directory
    from snowbot_bringup.runtime.run_vr import startup_lock
    from snowbot_hardware.adapter import SnowbotAdapter
    from snowbot_hardware.gripper_node import run_gripper_open

    site = json.loads((workspace/'upper/site.json').read_text())['arguments']
    namespace = site['namespace'].strip('/')
    lock = startup_lock(int(os.environ.get('ROS_DOMAIN_ID', '0')), namespace)
    cfg = Path(get_package_share_directory('snowbot_bringup'))/'config/robots/snowbot'
    parameters = dict(runtime_mode='real', profile=str(cfg/'snowbot_real_position.json'),
        execution_mode=site['execution_mode'], command_enabled='false', gripper_only='true',
        startup_home='false', gripper_config=str(cfg/'grippers.json'),
        parameters_config=str(cfg/'runtime_parameters.json'))
    args = ['--ros-args', '-r', '__ns:=/'+namespace]
    for key, value in parameters.items():
        args += ['-p', key+':='+value]
    node = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = SnowbotAdapter()
        print('OPEN_GRIPPERS: 请先可靠支撑杆/铲；仅打开双爪，不接管双臂或腰部。', flush=True)
        measured = run_gripper_open(node, lambda:rclpy.spin_once(node, timeout_sec=.005))
        print('OPEN_GRIPPERS_SUCCEEDED measured_q_rad='+json.dumps(measured), flush=True)
    finally:
        if node is not None:
            # The gripper runner has already released and confirmed its stop.
            node.closing = True
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'build', 'start', 'plan', 'services', 'operator', 'open-grippers', 'trace-scheduling'))
    parser.add_argument('--workspace', type=Path, default=ROOT/'artifacts/robot')
    parser.add_argument('--startup-window',nargs='?',choices=('base','adapter','hold','follow'),const='base',
                        help='trace-scheduling: base/adapter capture 15 seconds; hold waits for a fault; follow waits for a PICO PDO command fault')
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
        if options.command == 'trace-scheduling':
            trace_scheduling(workspace,startup_window=options.startup_window)
            return 0
        if options.command == 'plan':
            bindu = locked_bindu(existing)
            print(json.dumps(dict(snowbot=git(ROOT, 'rev-parse', 'HEAD'), bindu=git(bindu, 'rev-parse', 'HEAD'),
                workspace=str(workspace), vendor_packages=TARGETS, upper_command=['tools.deployment.x86_real',
                'build', '--workspace', str(workspace/'upper')], driver_started=False), indent=2))
            return 0
        if options.command in ('prepare', 'build'):
            distro = require_platform()
        bindu = locked_bindu(existing, acquire=options.command in ('prepare', 'build'))
        if options.command == 'open-grippers':
            return open_grippers(workspace)
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
    except (ValueError, OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print('Snowbot deployment: '+str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

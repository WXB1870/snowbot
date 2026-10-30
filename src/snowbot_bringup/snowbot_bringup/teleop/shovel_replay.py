"""Small browser remote for the existing, simulation-only shovel GUI player."""
import argparse
import json
import queue
import re
import shlex
import subprocess
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# Runs under the login user's Python, never imports ROS or a hardware backend.
REMOTE = r'''
import hashlib, json, os, signal, subprocess, sys, time, uuid
from pathlib import Path
request = json.loads(sys.argv[1])
workspace = Path(request['workspace']).resolve()
root = Path(request['motion_root']).resolve()
launcher = workspace/'snowbot/artifacts/dependencies/bindu/tools/run_shovel_physics.sh'
clips = {'sweep': root/'motion.json', 'flip': root/'flip-p60-lift12/motion.json'}
panel = root/'replay-panel'
current = panel/'current.json'
def read_current():
    return json.loads(current.read_text()) if current.exists() else None
def matching(motions):
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            args = (proc/'cmdline').read_bytes().split(b'\0')
            if any(str(p).encode() in args for p in motions) and any(
                    a.endswith(b'/run_shovel_physics.py') or a.endswith(b'/run_shovel_physics.sh') for a in args):
                found.append(int(proc.name))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            pass
    return found
def stop():
    previous = read_current()
    motions = [root/name for name in ('motion.json','motion-2x.json','motion-4x.json','motion-8x.json')]
    motions += [root/'flip-p60-lift12'/name for name in ('motion.json','motion-8x.json')]
    if previous:
        motions.append(Path(previous['motion']))
    for sig in (signal.SIGTERM, signal.SIGKILL):
        pids = matching(motions)
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        if not pids:
            break
        if sig == signal.SIGTERM:
            deadline = time.monotonic()+3
            while matching(motions) and time.monotonic() < deadline:
                time.sleep(.1)
    if previous:
        previous['closed'] = True
        current.write_text(json.dumps(previous))
def state():
    meta = read_current()
    if not meta or meta.get('closed'):
        return {'phase':'idle','window_open':False}
    out = Path(meta['output'])
    alive = bool(matching([Path(meta['motion'])]))
    result = out/'result.json'
    failure = out/'failure.json'
    result_data = json.loads(result.read_text()) if result.exists() else {}
    if failure.exists() or result_data.get('status') not in (None,'passed'):
        phase = 'failed'
    elif result_data:
        phase = 'completed'
    elif not alive:
        phase = 'failed'
    else:
        phase = 'starting'
    progress = 0.
    log_path = Path(meta['log'])
    if log_path.exists():
        with log_path.open('rb') as stream:
            stream.seek(max(0, log_path.stat().st_size-65536))
            lines = stream.read().decode(errors='replace').splitlines()
        for line in reversed(lines):
            if line.startswith('PHYSICS baseline '):
                progress = max(0.,float(line.split()[2]))
                if phase == 'starting' and progress > 0:
                    phase = 'playing'
                break
    error = '本轮仿真未正常完成，请重新播放。' if phase == 'failed' else None
    return dict(phase=phase,window_open=alive,clip=meta['clip'],speed=meta['speed'],
                duration_s=meta['duration_s'],progress_s=meta['duration_s'] if phase=='completed' else progress,error=error)
action = request['action']
if action == 'play':
    clip, speed = request['clip'], request['speed']
    if clip not in clips or type(speed) is not int or speed not in (1,2,4,8):
        raise ValueError('INVALID_REPLAY_SELECTION')
    original = clips[clip]
    motion = json.loads(original.read_text())
    physics = root/'physics-damped.json'
    if (json.loads(physics.read_text()).get('simulation_only') is not True or
            json.loads(Path(motion['config_path']).read_text()).get('simulation_only') is not True):
        raise ValueError('SIMULATION_ONLY_REQUIRED')
    if not launcher.is_file() or not os.access(launcher, os.X_OK):
        raise ValueError('REPLAY_LAUNCHER_MISSING')
    # Validate the existing timeline before closing a visible previous run.
    times = [s['t'] for s in motion['samples']]
    if (times[0] != 0 or any(b <= a for a,b in zip(times,times[1:])) or
            abs(times[-1]-motion['duration_s']) > 1e-6):
        raise ValueError('INVALID_REPLAY_TIMELINE')
    stop()
    job = panel/(time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    job.mkdir(parents=True)
    for sample in motion['samples']:
        sample['t'] /= speed
    motion['duration_s'] /= speed
    motion['presentation'] = dict(speed=speed,source_motion=str(original),
        source_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
        purpose='accelerated simulation GUI reference replay')
    motion_path, output, log = job/'motion.json',job/'output',job/'run.log'
    motion_path.write_text(json.dumps(motion)+'\n')
    app = workspace/'snowbot'
    bindu = app/'artifacts/dependencies/bindu'
    env = dict(os.environ, BINDU_APPLICATION_SOURCE=str(app),BINDU_FRAMEWORK_SOURCE=str(bindu),
        BINDU_PACKAGE_PATH=':'.join(str(app/'src'/n) for n in
            ('snowbot_bringup','snowbot_description','snowbot_hardware','snowbot_teleop')),
        DISPLAY=':1',XAUTHORITY='/run/user/1000/gdm/Xauthority',XDG_RUNTIME_DIR='/run/user/1000',
        DBUS_SESSION_BUS_ADDRESS='unix:path=/run/user/1000/bus',
        ISAAC_SIM_PATH='/home/wxb/isaacsim-6.2.0',BINDU_ISAAC_STATE='/home/wxb/bindu/artifacts/isaac62-state')
    with log.open('wb') as stream:
        process = subprocess.Popen([str(launcher),str(motion_path),'--config',str(physics),
            '--output',str(output),'--gui','--keep-open'],env=env,stdin=subprocess.DEVNULL,
            stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    current.write_text(json.dumps(dict(clip=clip,speed=speed,pid=process.pid,
        motion=str(motion_path),output=str(output),log=str(log),duration_s=motion['duration_s'])))
elif action == 'close':
    stop()
elif action != 'state':
    raise ValueError('INVALID_REPLAY_ACTION')
print(json.dumps(state(),ensure_ascii=False))
'''


class Playback:
    def __init__(self, remote, workspace, motion_root):
        self.remote = remote
        self.base = dict(workspace=workspace, motion_root=motion_root)
        self.lock = threading.Lock()
        self.pending = queue.Queue()
        self.value = dict(phase='connecting', busy=False)
        threading.Thread(target=self._worker, daemon=True).start()

    def submit(self, data):
        if not isinstance(data, dict) or data.get('action') not in ('play', 'close'):
            raise ValueError('请选择播放或关闭窗口。')
        if data['action'] == 'play' and (
                data.get('clip') not in ('sweep', 'flip') or
                type(data.get('speed')) is not int or data['speed'] not in (1, 2, 4, 8)):
            raise ValueError('请选择已有动作和1、2、4、8倍速。')
        with self.lock:
            if self.value.get('busy'):
                raise RuntimeError('上一项操作正在处理中。')
            self.value.update(busy=True, phase='starting' if data['action']=='play' else 'closing', error=None)
            if data['action'] == 'play':
                self.value.update(clip=data['clip'], speed=data['speed'], duration_s=None, progress_s=0.)
        self.pending.put({k: data[k] for k in ('action', 'clip', 'speed') if k in data})

    def snapshot(self):
        with self.lock:
            return dict(self.value)

    def _worker(self):
        while True:
            try:
                data = self.pending.get(timeout=3)
            except queue.Empty:
                data = dict(action='state')
            request = dict(self.base, **data)
            command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', self.remote,
                       'python3', '-c', shlex.quote(REMOTE), shlex.quote(json.dumps(request))]
            try:
                response = subprocess.run(command, capture_output=True, text=True, timeout=25, check=True)
                state = json.loads(response.stdout)
            except (subprocess.SubprocessError, OSError, ValueError):
                state = dict(phase='error', error='暂时无法连接116，请检查网络后重试。')
            with self.lock:
                if data['action'] == 'state' and not self.pending.empty():
                    continue
                self.value = dict(state, busy=not self.pending.empty())


def page():
    source = Path(__file__).resolve().parents[2]/'web/shovel_replay.html'
    if source.is_file():
        return source.read_bytes()
    from ament_index_python.packages import get_package_share_directory
    return (Path(get_package_share_directory('snowbot_bringup'))/'web/shovel_replay.html').read_bytes()


def serve(playback, port):
    html = page()

    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, data, content_type='application/json; charset=utf-8'):
            raw = data if isinstance(data, bytes) else json.dumps(data, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(raw)

        def local_request(self):
            allowed = (f'127.0.0.1:{port}', f'localhost:{port}')
            return (self.headers.get('Host') in allowed and
                    self.headers.get('Origin') in (None, *(f'http://{host}' for host in allowed)))

        def do_GET(self):
            if not self.local_request():
                return self.reply(403, dict(error='请从本机地址打开操作台。'))
            if self.path == '/':
                return self.reply(200, html, 'text/html; charset=utf-8')
            if self.path == '/api/state':
                return self.reply(200, playback.snapshot())
            self.reply(404, dict(error='页面不存在。'))

        def do_POST(self):
            if not self.local_request() or self.headers.get_content_type() != 'application/json':
                return self.reply(403, dict(error='请求来源或格式无效。'))
            if self.path != '/api/action':
                return self.reply(404, dict(error='操作不存在。'))
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 2048:
                    raise ValueError('请求长度无效。')
                playback.submit(json.loads(self.rfile.read(length)))
            except (ValueError, UnicodeError) as exc:
                return self.reply(400, dict(error=str(exc)))
            except RuntimeError as exc:
                return self.reply(409, dict(error=str(exc)))
            self.reply(202, playback.snapshot())

        def log_message(self, *args):
            pass

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--remote', required=True, help='SSH login for the simulation developer machine')
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--motion-root', required=True)
    parser.add_argument('--port', type=int, default=8776)
    parser.add_argument('--open', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9@._-]*', args.remote):
        parser.error('invalid SSH destination')
    if not 1 <= args.port <= 65535 or not all(Path(p).is_absolute() for p in (args.workspace,args.motion_root)):
        parser.error('use absolute remote paths and a valid local port')
    playback = Playback(args.remote, args.workspace, args.motion_root)
    server = serve(playback, args.port)
    url = f'http://127.0.0.1:{args.port}/'
    print('回放操作台：'+url, flush=True)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()

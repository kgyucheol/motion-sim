"""Editor-side handle for the persistent Isaac worker process (motionsim/isaac_worker.py)."""
import os
import subprocess
import sys
import tempfile
import threading
import time
from multiprocessing.connection import Client
from pathlib import Path

from .isaac_worker import authkey

ROOT = Path(__file__).resolve().parents[1]
STARTUP_TIMEOUT = 240.   # seconds; Isaac Sim usually needs ~40 s


class IsaacWorker:
    def __init__(self, model_id, port=8091, on_state=None):
        self.model_id, self.port = model_id, port
        self.on_state = on_state or (lambda state, detail: None)
        self.state, self.detail = 'stopped', ''
        self.process = None
        self.connection = None
        self.lock = threading.Lock()
        self.log_path = Path(tempfile.gettempdir()) / f'motionsim_isaac_worker_{port}.log'

    def _set(self, state, detail=''):
        self.state, self.detail = state, detail
        self.on_state(state, detail)

    def start(self):
        """Launch the worker in the background; the state becomes 'ready' once Isaac has loaded."""
        with self.lock:
            if self.state in ('starting', 'ready', 'running'):
                return
            env = dict(os.environ, OMNI_KIT_ACCEPT_EULA='YES', PYTHONPATH=str(ROOT))
            log = open(self.log_path, 'w')
            self.process = subprocess.Popen([sys.executable, '-m', 'motionsim.isaac_worker', '--model', self.model_id,
                                             '--port', str(self.port), '--parent-pid', str(os.getpid())],
                                            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
            self._set('starting', 'Isaac Sim을 띄우는 중입니다 (보통 40초 정도).')
        threading.Thread(target=self._connect, daemon=True).start()

    def _connect(self):
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self._set('failed', f'워커가 종료되었습니다 (코드 {self.process.returncode}). 로그: {self.log_path}')
                return
            try:
                connection = Client(('127.0.0.1', self.port), authkey=authkey())
            except OSError:
                time.sleep(1.)
                continue
            message = connection.recv()
            if message.get('model') != self.model_id:
                connection.close()
                self._set('failed', f'포트 {self.port}의 워커 모델({message.get("model")})이 편집기 모델과 다릅니다.')
                return
            self.connection = connection
            self._set('ready', f'모델 {self.model_id}')
            return
        self._set('failed', f'{STARTUP_TIMEOUT:.0f}초 안에 워커가 준비되지 않았습니다. 로그: {self.log_path}')

    def run(self, project, on_progress, hold_seconds=1.):
        """Blocking: send one job and return the worker's result or error message."""
        with self.lock:
            if self.state != 'ready':
                raise RuntimeError('Isaac 워커가 준비되지 않았습니다.')
            self._set('running', 'Isaac에서 계산 중입니다.')
        try:
            self.connection.send({'type': 'run', 'project': project, 'hold_seconds': hold_seconds})
            while True:
                message = self.connection.recv()
                if message['type'] == 'progress':
                    on_progress(message['value'])
                else:
                    return message
        except (EOFError, OSError) as error:
            self._set('failed', f'워커와 연결이 끊겼습니다: {error}. 로그: {self.log_path}')
            raise RuntimeError('Isaac 워커와 연결이 끊겼습니다.') from error
        finally:
            if self.state == 'running':
                self._set('ready', f'모델 {self.model_id}')

    def stop(self):
        with self.lock:
            if self.connection is not None:
                try:
                    self.connection.send({'type': 'shutdown'})
                except OSError:
                    pass
                self.connection.close()
                self.connection = None
            if self.process is not None and self.process.poll() is None:
                try:
                    self.process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
            self._set('stopped', '')

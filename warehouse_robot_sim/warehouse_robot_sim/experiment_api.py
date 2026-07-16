from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import threading
import time
from urllib.parse import parse_qs, urlparse

from ament_index_python.packages import get_package_share_directory

from warehouse_robot_sim.layout_config import available_layouts


SCHEDULERS = {
    'fcfs': 'FCFS',
    'sjf': 'Shortest Job First',
    'sjf_aging': 'SJF + Aging',
    'nearest_robot': 'Nearest Robot',
}


class ExperimentManager:
    def __init__(self, host='127.0.0.1', port=8080):
        self.host = host
        self.port = port
        self.package_share = Path(get_package_share_directory('warehouse_robot_sim'))
        self.workspace_root = self._find_workspace_root()
        self.results_root = self.workspace_root / 'results'
        self.dashboard_path = self.package_share / 'dashboard' / 'index.html'
        self.lock = threading.Lock()
        self.experiments = {}
        self.active_id = None

    def _find_workspace_root(self):
        current = self.package_share
        for parent in [current] + list(current.parents):
            if (parent / 'install' / 'setup.bash').exists():
                return parent
        return Path.cwd()

    def config(self):
        layouts = available_layouts()
        return {
            'layouts': layouts,
            'schedulers': [{'id': key, 'name': name} for key, name in SCHEDULERS.items()],
            'robot_counts': [1, 2, 3],
            'defaults': {
                'layout': 'standard',
                'robot_count': 3,
                'scheduler_policy': 'sjf_aging',
                'job_count': 8,
                'seed': 7,
            },
            'active_experiment_id': self.active_id,
        }

    def create_experiment(self, payload):
        config = self._validate(payload)
        with self.lock:
            if self.active_id and self.experiments[self.active_id]['status'] in {
                'starting', 'running', 'reviewing', 'stopping'
            }:
                raise ValueError('An experiment is already running.')

            experiment_id = datetime.now().strftime('%Y%m%d_%H%M%S_') + secrets.token_hex(3)
            result_dir = self.results_root / experiment_id
            result_dir.mkdir(parents=True, exist_ok=True)
            experiment = {
                'id': experiment_id,
                'config': config,
                'status': 'starting',
                'created_at': time.time(),
                'updated_at': time.time(),
                'result_dir': str(result_dir),
                'processes': [],
                'stop_event': threading.Event(),
                'message': 'Starting Gazebo and Nav2.',
                'summary': None,
            }
            self.experiments[experiment_id] = experiment
            self.active_id = experiment_id

        thread = threading.Thread(target=self._run_experiment, args=(experiment,), daemon=True)
        thread.start()
        return self.public_experiment(experiment_id)

    def _validate(self, payload):
        layout_names = {layout['name'] for layout in available_layouts()}
        layout = str(payload.get('layout', 'standard'))
        if layout not in layout_names:
            raise ValueError(f'Unknown layout: {layout}')

        try:
            robot_count = int(payload.get('robot_count', 3))
            job_count = int(payload.get('job_count', 8))
            seed = int(payload.get('seed', 0))
        except (TypeError, ValueError) as exc:
            raise ValueError('Robot count, job count, and seed must be integers.') from exc

        scheduler_policy = str(payload.get('scheduler_policy', 'sjf_aging'))
        if scheduler_policy not in SCHEDULERS:
            raise ValueError(f'Unknown scheduler: {scheduler_policy}')
        if robot_count < 1 or robot_count > 3:
            raise ValueError('Robot count must be between 1 and 3.')
        if job_count < 1 or job_count > 50:
            raise ValueError('Job count must be between 1 and 50.')
        if seed < 0:
            raise ValueError('Seed must be zero or greater.')

        return {
            'layout': layout,
            'robot_count': robot_count,
            'scheduler_policy': scheduler_policy,
            'job_count': job_count,
            'seed': seed,
        }

    def _run_experiment(self, experiment):
        config = experiment['config']
        result_dir = Path(experiment['result_dir'])
        robots = [f'robot{i}' for i in range(1, config['robot_count'] + 1)]

        try:
            # Start the same world, Nav2, and dispatcher commands used for a manual run.
            self._write_requested_config(result_dir, config)
            self._set_status(experiment, 'starting', 'Launching Gazebo world.')
            self._start_world_process(experiment)
            if not self._sleep_or_stop(experiment, 7.0):
                return

            self._set_status(experiment, 'starting', 'Launching Nav2 stacks.')
            self._start_process(
                experiment,
                'nav',
                [
                    'ros2', 'launch', 'warehouse_robot_sim', 'multi_robot_navigation.launch.py',
                    f'layout:={config["layout"]}',
                    f'robot_count:={config["robot_count"]}',
                ],
            )
            if not self._sleep_or_stop(experiment, 8.0):
                return

            self._set_status(experiment, 'running', 'Dispatcher is running jobs.')
            dispatcher = self._start_process(
                experiment,
                'dispatcher',
                [
                    'ros2', 'run', 'warehouse_robot_sim', 'multi_robot_dispatcher_node',
                    '--ros-args',
                    '-p', 'use_sim_time:=true',
                    '-p', f'layout:={config["layout"]}',
                    '-p', f'robots:={json.dumps(robots)}',
                    '-p', f'scheduler_policy:={config["scheduler_policy"]}',
                    '-p', f'max_jobs:={config["job_count"]}',
                    '-p', f'seed:={config["seed"]}',
                    '-p', 'job_interval_sec:=3.0',
                    '-p', 'arrival_jitter_sec:=1.5',
                    '-p', 'assignment_stagger_sec:=0.5',
                    '-p', f'results_dir:={result_dir.as_posix()}',
                ],
            )
            return_code = self._wait_for_dispatcher(experiment, dispatcher)
            summary = self._load_summary(result_dir)
            status = 'completed'
            message = 'Experiment complete.'
            if experiment['stop_event'].is_set():
                status = 'cancelled'
                message = 'Experiment stopped.'
            elif return_code != 0:
                status = 'completed_with_failures' if summary else 'failed'
                message = 'Dispatcher exited with failures.'
            elif summary and summary.get('jobs_failed', 0) > 0:
                status = 'completed_with_failures'
                message = 'Experiment finished with failed jobs.'
            if not experiment['stop_event'].is_set():
                self._set_status(
                    experiment,
                    'reviewing',
                    'Experiment complete. Gazebo is staying open for review.',
                    summary,
                )
                if not self._sleep_or_stop(experiment, 120.0):
                    return
            self._set_status(experiment, status, message, summary)
        except Exception as exc:
            self._set_status(experiment, 'failed', str(exc))
        finally:
            self._stop_processes(experiment)
            with self.lock:
                if self.active_id == experiment['id']:
                    self.active_id = None

    def _start_process(self, experiment, name, args):
        result_dir = Path(experiment['result_dir'])
        log_path = result_dir / f'{name}.log'
        log_file = log_path.open('w', encoding='utf-8')
        process = subprocess.Popen(
            args,
            cwd=str(self.workspace_root),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        experiment['processes'].append({'name': name, 'process': process, 'log_file': log_file})
        return process

    def _start_world_process(self, experiment):
        config = experiment['config']
        return self._start_process(
            experiment,
            'world',
            [
                'ros2', 'launch', 'warehouse_robot_sim', 'multi_robot_world.launch.py',
                f'layout:={config["layout"]}',
                f'robot_count:={config["robot_count"]}',
            ],
        )

    def _wait_for_dispatcher(self, experiment, dispatcher):
        while dispatcher.poll() is None:
            if experiment['stop_event'].is_set():
                return dispatcher.returncode or -1
            experiment['summary'] = self._load_summary(Path(experiment['result_dir']))
            experiment['updated_at'] = time.time()
            time.sleep(1.0)
        return dispatcher.returncode

    def _sleep_or_stop(self, experiment, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            if experiment['stop_event'].is_set():
                self._set_status(experiment, 'cancelled', 'Experiment stopped.')
                return False
            time.sleep(0.25)
        return True

    def _stop_processes(self, experiment):
        # Each command has its own process group, so its ROS child nodes stop too.
        for item in reversed(experiment.get('processes', [])):
            process = item['process']
            if process is None:
                continue
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=8)
                except Exception:
                    process.kill()
            try:
                item['log_file'].close()
            except Exception:
                pass

    def stop_experiment(self, experiment_id):
        with self.lock:
            experiment = self.experiments.get(experiment_id)
            if not experiment:
                raise ValueError('Experiment not found.')
            experiment['stop_event'].set()
            experiment['status'] = 'stopping'
            experiment['message'] = 'Stopping experiment.'
            experiment['updated_at'] = time.time()
        return self.public_experiment(experiment_id)

    def public_experiment(self, experiment_id):
        # Only return fields that the dashboard needs.
        with self.lock:
            experiment = self.experiments.get(experiment_id)
            if not experiment:
                raise ValueError('Experiment not found.')
            result_dir = Path(experiment['result_dir'])
            summary = experiment.get('summary') or self._load_summary(result_dir)
            recent_events = self._load_recent_events(result_dir)
            return {
                'id': experiment['id'],
                'config': experiment['config'],
                'status': experiment['status'],
                'message': experiment['message'],
                'created_at': experiment['created_at'],
                'updated_at': experiment['updated_at'],
                'result_dir': experiment['result_dir'],
                'summary': summary,
                'recent_events': recent_events,
            }

    def _set_status(self, experiment, status, message, summary=None):
        with self.lock:
            experiment['status'] = status
            experiment['message'] = message
            experiment['updated_at'] = time.time()
            if summary is not None:
                experiment['summary'] = summary

    def _write_requested_config(self, result_dir, config):
        with (result_dir / 'config.json').open('w', encoding='utf-8') as file:
            json.dump(config, file, indent=2)

    def _load_summary(self, result_dir):
        summary_path = result_dir / 'summary.json'
        if not summary_path.exists():
            return None
        try:
            with summary_path.open('r', encoding='utf-8') as file:
                return json.load(file)
        except json.JSONDecodeError:
            return None

    def _load_recent_events(self, result_dir, limit=12):
        events_path = result_dir / 'events.jsonl'
        if not events_path.exists():
            return []
        try:
            lines = events_path.read_text(encoding='utf-8').splitlines()[-limit:]
            return [json.loads(line) for line in lines if line.strip()]
        except (OSError, json.JSONDecodeError):
            return []


class ExperimentRequestHandler(BaseHTTPRequestHandler):
    manager = None

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/':
            self._serve_dashboard()
            return
        if parsed.path == '/api/config':
            self._send_json(self.manager.config())
            return
        if parsed.path.startswith('/api/experiments/'):
            experiment_id = parsed.path.rsplit('/', 1)[-1]
            try:
                self._send_json(self.manager.public_experiment(experiment_id))
            except ValueError as exc:
                self._send_json({'error': str(exc)}, status=404)
            return
        self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/experiments':
            payload = self._read_payload()
            try:
                self._send_json(self.manager.create_experiment(payload), status=201)
            except ValueError as exc:
                self._send_json({'error': str(exc)}, status=400)
            return
        if parsed.path.startswith('/api/experiments/') and parsed.path.endswith('/stop'):
            experiment_id = parsed.path.split('/')[-2]
            try:
                self._send_json(self.manager.stop_experiment(experiment_id))
            except ValueError as exc:
                self._send_json({'error': str(exc)}, status=404)
            return
        self.send_error(404)

    def _serve_dashboard(self):
        html = self.manager.dashboard_path.read_text(encoding='utf-8')
        body = html.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_payload(self):
        length = int(self.headers.get('Content-Length', '0'))
        raw_body = self.rfile.read(length).decode('utf-8') if length else '{}'
        content_type = self.headers.get('Content-Type', '')
        if 'application/json' in content_type:
            return json.loads(raw_body or '{}')
        parsed = parse_qs(raw_body)
        return {key: values[-1] for key, values in parsed.items()}

    def _send_json(self, payload, status=200):
        body = json.dumps(payload, indent=2).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format_string, *args):
        return


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def make_server(manager):
    for port in range(manager.port, manager.port + 10):
        try:
            manager.port = port
            return ReusableThreadingHTTPServer(
                (manager.host, manager.port),
                ExperimentRequestHandler,
            )
        except OSError:
            continue
    raise OSError('Could not bind experiment API to ports 8080-8089.')


def main():
    manager = ExperimentManager()
    ExperimentRequestHandler.manager = manager
    server = make_server(manager)
    print(f'Experiment dashboard running at http://{manager.host}:{manager.port}')
    print(f'Results will be saved under {manager.results_root}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()

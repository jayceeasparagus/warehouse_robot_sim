import csv
from contextlib import contextmanager
import json
from pathlib import Path
import sqlite3
import threading
import time


TERMINAL_STATUSES = {
    'completed',
    'completed_with_failures',
    'failed',
    'cancelled',
}


class ExperimentStore:
    """Stores experiment configs, results, jobs, and events in SQLite."""

    def __init__(self, database_path):
        self.database_path = Path(database_path)
        self.database_path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self._create_tables()

    def _connect(self):
        connection = sqlite3.connect(self.database_path, timeout=10.0)
        connection.row_factory = sqlite3.Row
        connection.execute('PRAGMA foreign_keys = ON')
        connection.execute('PRAGMA busy_timeout = 10000')
        return connection

    @contextmanager
    def _database(self):
        connection = self._connect()
        try:
            with connection:
                yield connection
        finally:
            connection.close()

    def _create_tables(self):
        schema = """
        CREATE TABLE IF NOT EXISTS experiments (
            id TEXT PRIMARY KEY,
            created_at REAL NOT NULL,
            updated_at REAL NOT NULL,
            completed_at REAL,
            status TEXT NOT NULL,
            layout TEXT NOT NULL,
            robot_count INTEGER NOT NULL,
            scheduler_policy TEXT NOT NULL,
            job_count INTEGER NOT NULL,
            seed INTEGER NOT NULL,
            result_dir TEXT NOT NULL,
            message TEXT NOT NULL DEFAULT '',
            summary_json TEXT
        );

        CREATE TABLE IF NOT EXISTS jobs (
            experiment_id TEXT NOT NULL,
            job_id INTEGER NOT NULL,
            pickup TEXT,
            dropoff TEXT,
            robot TEXT,
            status TEXT,
            attempts INTEGER,
            created_time REAL,
            assigned_time REAL,
            pickup_time REAL,
            completed_time REAL,
            wait_sec REAL,
            completion_sec REAL,
            failure_reason TEXT,
            PRIMARY KEY (experiment_id, job_id),
            FOREIGN KEY (experiment_id) REFERENCES experiments(id) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS job_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            experiment_id TEXT NOT NULL,
            sequence INTEGER NOT NULL,
            event_type TEXT NOT NULL,
            sim_time_sec REAL,
            job_id INTEGER,
            robot TEXT,
            route TEXT,
            details_json TEXT NOT NULL,
            UNIQUE (experiment_id, sequence),
            FOREIGN KEY (experiment_id) REFERENCES experiments(id) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS robot_metrics (
            experiment_id TEXT NOT NULL,
            robot TEXT NOT NULL,
            utilization REAL NOT NULL,
            PRIMARY KEY (experiment_id, robot),
            FOREIGN KEY (experiment_id) REFERENCES experiments(id) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS station_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            experiment_id TEXT NOT NULL,
            event_type TEXT NOT NULL,
            sim_time_sec REAL,
            station TEXT NOT NULL,
            robot TEXT,
            job_id INTEGER,
            details_json TEXT NOT NULL,
            FOREIGN KEY (experiment_id) REFERENCES experiments(id) ON DELETE CASCADE
        );

        CREATE INDEX IF NOT EXISTS idx_experiments_created
            ON experiments(created_at DESC);
        CREATE INDEX IF NOT EXISTS idx_events_experiment
            ON job_events(experiment_id, sequence);
        """
        with self.lock, self._database() as connection:
            connection.execute('PRAGMA journal_mode = WAL')
            connection.executescript(schema)

    def save_experiment(self, experiment):
        config = experiment['config']
        summary = experiment.get('summary')
        status = experiment['status']
        completed_at = time.time() if status in TERMINAL_STATUSES else None
        summary_json = json.dumps(summary) if summary is not None else None

        values = (
            experiment['id'],
            experiment['created_at'],
            experiment['updated_at'],
            completed_at,
            status,
            config['layout'],
            config['robot_count'],
            config['scheduler_policy'],
            config['job_count'],
            config['seed'],
            experiment['result_dir'],
            experiment.get('message', ''),
            summary_json,
        )
        with self.lock, self._database() as connection:
            connection.execute(
                """
                INSERT INTO experiments (
                    id, created_at, updated_at, completed_at, status, layout,
                    robot_count, scheduler_policy, job_count, seed, result_dir,
                    message, summary_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(id) DO UPDATE SET
                    updated_at = excluded.updated_at,
                    completed_at = COALESCE(excluded.completed_at, experiments.completed_at),
                    status = excluded.status,
                    layout = excluded.layout,
                    robot_count = excluded.robot_count,
                    scheduler_policy = excluded.scheduler_policy,
                    job_count = excluded.job_count,
                    seed = excluded.seed,
                    result_dir = excluded.result_dir,
                    message = excluded.message,
                    summary_json = COALESCE(excluded.summary_json, experiments.summary_json)
                """,
                values,
            )

    def sync_result_files(self, experiment_id, result_dir):
        result_dir = Path(result_dir)
        summary = self._read_json(result_dir / 'summary.json')
        jobs = self._read_jobs(result_dir / 'jobs.csv')
        events = self._read_events(result_dir / 'events.jsonl')

        with self.lock, self._database() as connection:
            if summary is not None:
                connection.execute(
                    'UPDATE experiments SET summary_json = ?, updated_at = ? WHERE id = ?',
                    (json.dumps(summary), time.time(), experiment_id),
                )

            connection.execute('DELETE FROM jobs WHERE experiment_id = ?', (experiment_id,))
            for job in jobs:
                connection.execute(
                    """
                    INSERT INTO jobs VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        experiment_id,
                        self._to_int(job.get('job_id')),
                        job.get('pickup'),
                        job.get('dropoff'),
                        job.get('robot'),
                        job.get('status'),
                        self._to_int(job.get('attempts')),
                        self._to_float(job.get('created_time')),
                        self._to_float(job.get('assigned_time')),
                        self._to_float(job.get('pickup_time')),
                        self._to_float(job.get('completed_time')),
                        self._to_float(job.get('wait_sec')),
                        self._to_float(job.get('completion_sec')),
                        job.get('failure_reason'),
                    ),
                )

            connection.execute(
                'DELETE FROM job_events WHERE experiment_id = ?', (experiment_id,)
            )
            connection.execute(
                'DELETE FROM station_events WHERE experiment_id = ?', (experiment_id,)
            )
            for sequence, event in enumerate(events):
                details = {
                    key: value for key, value in event.items()
                    if key not in {'event', 'sim_time_sec', 'job_id', 'robot', 'route'}
                }
                connection.execute(
                    """
                    INSERT INTO job_events (
                        experiment_id, sequence, event_type, sim_time_sec,
                        job_id, robot, route, details_json
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        experiment_id,
                        sequence,
                        event.get('event', ''),
                        self._to_float(event.get('sim_time_sec')),
                        self._to_int(event.get('job_id')),
                        event.get('robot'),
                        event.get('route'),
                        json.dumps(details),
                    ),
                )
                station = self._station_from_event(event)
                if station:
                    connection.execute(
                        """
                        INSERT INTO station_events (
                            experiment_id, event_type, sim_time_sec, station,
                            robot, job_id, details_json
                        ) VALUES (?, ?, ?, ?, ?, ?, ?)
                        """,
                        (
                            experiment_id,
                            event.get('event', ''),
                            self._to_float(event.get('sim_time_sec')),
                            station,
                            event.get('robot'),
                            self._to_int(event.get('job_id')),
                            json.dumps(details),
                        ),
                    )

            connection.execute(
                'DELETE FROM robot_metrics WHERE experiment_id = ?', (experiment_id,)
            )
            for robot, utilization in (summary or {}).get('robot_utilization', {}).items():
                connection.execute(
                    'INSERT INTO robot_metrics VALUES (?, ?, ?)',
                    (experiment_id, robot, float(utilization)),
                )

    def import_existing_results(self, results_root):
        imported = 0
        for result_dir in sorted(Path(results_root).iterdir()):
            if not result_dir.is_dir():
                continue
            config = self._read_json(result_dir / 'config.json')
            if not config:
                continue
            summary = self._read_json(result_dir / 'summary.json')
            status = (summary or {}).get('status', 'incomplete')
            modified = result_dir.stat().st_mtime
            experiment = {
                'id': result_dir.name,
                'config': config,
                'status': status,
                'created_at': modified,
                'updated_at': modified,
                'result_dir': str(result_dir),
                'message': 'Imported from result files.',
                'summary': summary,
            }
            self.save_experiment(experiment)
            self.sync_result_files(result_dir.name, result_dir)
            imported += 1
        return imported

    def list_experiments(self, limit=20):
        limit = max(1, min(int(limit), 100))
        with self._database() as connection:
            rows = connection.execute(
                'SELECT * FROM experiments ORDER BY created_at DESC LIMIT ?', (limit,)
            ).fetchall()
        return [self._summary_row(row) for row in rows]

    def get_experiment(self, experiment_id, event_limit=12):
        with self._database() as connection:
            row = connection.execute(
                'SELECT * FROM experiments WHERE id = ?', (experiment_id,)
            ).fetchone()
            if row is None:
                return None
            event_rows = connection.execute(
                """
                SELECT * FROM job_events
                WHERE experiment_id = ? ORDER BY sequence DESC LIMIT ?
                """,
                (experiment_id, event_limit),
            ).fetchall()

        recent_events = []
        for event_row in reversed(event_rows):
            event = {
                'event': event_row['event_type'],
                'sim_time_sec': event_row['sim_time_sec'],
            }
            for key in ('job_id', 'robot', 'route'):
                if event_row[key] is not None:
                    event[key] = event_row[key]
            event.update(json.loads(event_row['details_json']))
            recent_events.append(event)

        return {
            'id': row['id'],
            'config': self._config_from_row(row),
            'status': row['status'],
            'message': row['message'],
            'created_at': row['created_at'],
            'updated_at': row['updated_at'],
            'result_dir': row['result_dir'],
            'summary': self._decode_summary(row['summary_json']),
            'recent_events': recent_events,
        }

    def _summary_row(self, row):
        summary = self._decode_summary(row['summary_json']) or {}
        return {
            'id': row['id'],
            'created_at': row['created_at'],
            'status': row['status'],
            'layout': row['layout'],
            'robot_count': row['robot_count'],
            'scheduler_policy': row['scheduler_policy'],
            'job_count': row['job_count'],
            'seed': row['seed'],
            'jobs_completed': summary.get('jobs_completed', 0),
            'jobs_failed': summary.get('jobs_failed', 0),
            'makespan_sec': summary.get('makespan_sec', 0.0),
        }

    @staticmethod
    def _config_from_row(row):
        return {
            'layout': row['layout'],
            'robot_count': row['robot_count'],
            'scheduler_policy': row['scheduler_policy'],
            'job_count': row['job_count'],
            'seed': row['seed'],
        }

    @staticmethod
    def _decode_summary(value):
        return json.loads(value) if value else None

    @staticmethod
    def _read_json(path):
        if not path.exists():
            return None
        try:
            return json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            return None

    @staticmethod
    def _read_jobs(path):
        if not path.exists():
            return []
        try:
            with path.open('r', newline='', encoding='utf-8') as job_file:
                return list(csv.DictReader(job_file))
        except OSError:
            return []

    @staticmethod
    def _read_events(path):
        if not path.exists():
            return []
        events = []
        try:
            for line in path.read_text(encoding='utf-8').splitlines():
                if line.strip():
                    events.append(json.loads(line))
        except (OSError, json.JSONDecodeError):
            return []
        return events

    @staticmethod
    def _station_from_event(event):
        route = event.get('route', '')
        if '->' not in route:
            return None
        pickup, dropoff = route.split('->', 1)
        if event.get('event') == 'pickup_complete':
            return pickup
        if event.get('event') == 'job_completed':
            return dropoff
        return None

    @staticmethod
    def _to_float(value):
        if value in (None, ''):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _to_int(value):
        if value in (None, ''):
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

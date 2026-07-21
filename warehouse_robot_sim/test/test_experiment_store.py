import csv
import json
from pathlib import Path

from warehouse_robot_sim.experiment_store import ExperimentStore


def test_store_saves_and_loads_experiment(tmp_path):
    results_root = tmp_path / 'results'
    result_dir = results_root / 'run_001'
    result_dir.mkdir(parents=True)

    config = {
        'layout': 'standard',
        'robot_count': 2,
        'scheduler_policy': 'sjf_aging',
        'job_count': 1,
        'seed': 7,
    }
    summary = {
        'status': 'completed',
        'jobs_generated': 1,
        'jobs_completed': 1,
        'jobs_failed': 0,
        'makespan_sec': 12.5,
        'robot_utilization': {'robot1': 0.8, 'robot2': 0.2},
    }
    (result_dir / 'config.json').write_text(json.dumps(config), encoding='utf-8')
    (result_dir / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')
    (result_dir / 'events.jsonl').write_text(
        json.dumps({
            'event': 'job_completed',
            'sim_time_sec': 12.5,
            'job_id': 1,
            'robot': 'robot1',
            'route': 'A1->B1',
        }) + '\n',
        encoding='utf-8',
    )
    with (result_dir / 'jobs.csv').open('w', newline='', encoding='utf-8') as job_file:
        writer = csv.writer(job_file)
        writer.writerow([
            'job_id', 'pickup', 'dropoff', 'robot', 'status', 'attempts',
            'created_time', 'assigned_time', 'pickup_time', 'completed_time',
            'wait_sec', 'completion_sec', 'failure_reason',
        ])
        writer.writerow([1, 'A1', 'B1', 'robot1', 'completed', 1, 0, 1, 6, 12.5, 1, 11.5, ''])

    store = ExperimentStore(results_root / 'experiments.db')
    assert store.import_existing_results(results_root) == 1

    saved = store.get_experiment('run_001')
    assert saved['config'] == config
    assert saved['summary']['jobs_completed'] == 1
    assert saved['recent_events'][0]['route'] == 'A1->B1'

    history = store.list_experiments()
    assert history[0]['id'] == 'run_001'
    assert history[0]['jobs_completed'] == 1

    with store._database() as connection:
        assert connection.execute('SELECT COUNT(*) FROM jobs').fetchone()[0] == 1
        assert connection.execute('SELECT COUNT(*) FROM robot_metrics').fetchone()[0] == 2
        assert connection.execute('SELECT station FROM station_events').fetchone()[0] == 'B1'

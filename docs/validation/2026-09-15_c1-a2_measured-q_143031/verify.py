#!/usr/bin/env python3
"""Offline verification: run from the package root with PYTHONPATH=. (no ROS)."""
import json
import math
import statistics
from pathlib import Path
from fr3_lab_stack_runtime.timing_evidence import analyze, same_clock

root = Path(__file__).resolve().parent
records = [json.loads(line) for line in (root / 'c1-a2-20-20260915T143031.jsonl').read_text().splitlines()]
summary = json.loads((root / 'c1-a2-20-20260915T143031-summary.json').read_text())
assert analyze(records) == summary
assert next(r['data'] for r in records if r['kind'] == 'summary') == summary
sent = [r for r in records if r['kind'] == 'sent']
forwarded = [r['data'] for r in records if r['kind'] == 'forwarder']
controller = [r['data'] for r in records if r['kind'] == 'controller']
callbacks = [r for r in controller if r['event'] == 'callback']
samples = [s for r in controller if r['event'] == 'samples' for s in r['samples']]
apps = [s['application'] for s in samples if 'application' in s]
assert len(sent) == len(forwarded) == len(callbacks) == len(apps) == 20
assert len({r['source_stamp_ns'] for r in sent}) == 20
assert same_clock(*sent, *forwarded, *controller)
assert all(r['publication_call_outcome'] == 'returned' for r in sent)
assert all(r['outcome'] == 'forwarded' and r['counts']['publication_errors'] == 0 for r in forwarded)
assert all(r['outcome'] == 'accepted' for r in callbacks)
assert all(t['status'] == 'applied' and t['sent_target'] == t['q_desired'] and
           t['forwarder_records'] == t['callback_records'] == t['application_records'] == 1
           for t in summary['targets'])
sequences = [t['sequence'] for t in summary['targets']]
assert sequences == list(range(2, 22))
cycles = [s['cycle'] for s in samples]
assert len(cycles) == 6000 and cycles == list(range(cycles[0], cycles[0] + 6000))
assert not any(summary['evidence_id_gaps'].values())
assert not any(summary['controller_publication_errors_cumulative'].values())
assert all(not any(v.values()) for v in summary['rt_overflow_cumulative'].values())
assert all(summary[k] == 0 for k in ('unmatched_forwarder_records', 'unmatched_callback_records', 'unmatched_application_records'))
intervals = {k: [] for k in ('t0_t1', 't1_t2', 't2_t3', 't3_t4', 't0_t4')}
for target in sent:
    stamp = target['source_stamp_ns']
    f = next(f for f in forwarded if f['source_stamp_sec'] * 10**9 + f['source_stamp_nanosec'] == stamp)
    c = next(c for c in callbacks if c['source_stamp_ns'] == stamp)
    a = next(a for a in apps if a['source_stamp_ns'] == stamp)
    assert f['source_frame_id'] == c['source_frame_id'] == target['run']
    assert c['activation'] == a['activation'] == a['callback_activation']
    assert c['sequence'] == a['sequence'] and c['t3_ns'] == a['t3_ns']
    assert a['q_desired'] == target['q']
    times = [target['t0_ns'], f['server_receipt_monotonic_ns'],
             f['publication_call_start_monotonic_ns'], c['t3_ns'], a['t4_ns']]
    assert times == sorted(times)
    for key, value in zip(intervals, [times[1]-times[0], times[2]-times[1], times[3]-times[2], times[4]-times[3], times[4]-times[0]]):
        intervals[key].append(value)
        assert next(t for t in summary['targets'] if t['source_stamp_ns'] == stamp)['intervals_ns'][key] == value

def stats(values):
    values = sorted(values)
    return dict(count=len(values), mean_ns=statistics.mean(values), median_ns=statistics.median(values),
                p95_ns=values[math.ceil(.95 * len(values))-1], p99_ns=values[math.ceil(.99 * len(values))-1],
                minimum_ns=values[0], maximum_ns=values[-1])

periods = [s['period_ns'] for s in samples]
ps = stats(periods)
for key, value in ps.items():
    if key != 'minimum_ns':
        assert summary['periods'][0][key] == value
assert sum(v > 1_500_000 for v in periods) == sum(v > 2_000_000 for v in periods) == 0
print(json.dumps(dict(verified=True, sequences=sequences, cycle_range=[cycles[0], cycles[-1]],
                     timing={k: stats(v) for k, v in intervals.items()}, periods=ps,
                     actual_t0_spacing=stats([b['t0_ns']-a['t0_ns'] for a,b in zip(sent,sent[1:])])), indent=2))

"""C1-A2 clock provenance and offline evidence analysis (no ROS dependencies)."""
import math
import os
import socket
import statistics
import time
from pathlib import Path


def clock_provenance():
    # Verify monotonic_ns (also used by the forwarder admission gate) against
    # the explicit POSIX clock used by the C++ controller. Linux same boot and
    # time namespace is required; ROS time is never used for these intervals.
    before = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    value = time.monotonic_ns()
    after = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    if not before <= value <= after:
        raise RuntimeError('Python monotonic_ns does not match CLOCK_MONOTONIC')
    return dict(clock='CLOCK_MONOTONIC', hostname=socket.gethostname(),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                time_namespace=os.readlink('/proc/self/ns/time'),
                python_clock_implementation=time.get_clock_info('monotonic').implementation,
                clock_verified=True)


def same_clock(*records):
    keys = ('clock', 'hostname', 'boot_id', 'time_namespace')
    return all(all(r.get(k) and r.get(k) == records[0].get(k) for k in keys)
               for r in records) and records[0].get('clock') == 'CLOCK_MONOTONIC'


def period_statistics(values, thresholds=(1_500_000, 2_000_000)):
    """Percentiles: nearest rank ceil(p*N), one-based; median averages middle pair."""
    values = sorted(values)
    result = dict(count=len(values), nominal_ns=1_000_000,
                  thresholds_ns=list(thresholds),
                  long_period_counts={str(t): sum(v > t for v in values) for t in thresholds},
                  percentile_method='nearest rank ceil(p*N), one-based')
    if not values:
        return dict(result, mean_ns=None, median_ns=None, p95_ns=None, p99_ns=None,
                    maximum_ns=None, mean_deviation_ns=None, maximum_absolute_deviation_ns=None)
    mean = statistics.mean(values)
    return dict(result, mean_ns=mean, median_ns=statistics.median(values),
                p95_ns=values[math.ceil(.95 * len(values)) - 1],
                p99_ns=values[math.ceil(.99 * len(values)) - 1], maximum_ns=values[-1],
                mean_deviation_ns=mean - 1_000_000,
                maximum_absolute_deviation_ns=max(abs(v - 1_000_000) for v in values))


def analyze(records):
    """Conservative joins: ambiguous stamps and incomplete evidence stay unresolved."""
    sent = [r for r in records if r['kind'] == 'sent']
    if len({r['run'] for r in sent}) > 1:
        raise ValueError('analyze one run at a time')
    stamps = [r['source_stamp_ns'] for r in sent]
    if len(stamps) != len(set(stamps)):
        raise ValueError('source stamps must be unique within the run')
    forwards = [r['data'] for r in records if r['kind'] == 'forwarder']
    controller = [r['data'] for r in records if r['kind'] == 'controller']
    callbacks = [r for r in controller if r['event'] == 'callback']
    batches = [r for r in controller if r['event'] == 'samples']
    applications = [dict(s['application'], **{k: b[k] for k in (
        'instance', 'clock', 'hostname', 'boot_id', 'time_namespace')})
        for b in batches for s in b['samples'] if 'application' in s]
    groups = {}
    for b in batches:
        for s in b['samples']:
            key = (b['instance'], s['activation'])
            groups.setdefault(key, []).append(s['period_ns'])
    # IDs expose middle-of-capture DDS losses; boundaries are explicitly unobservable.
    gaps = {}
    for kind, rows, identity, counter in (
        ('controller', controller, 'instance', lambda r: r['evidence_id']),
        ('forwarder', forwards, 'forwarder_instance', lambda r: r['counts']['received'])):
        for instance in {r[identity] for r in rows}:
            ids = sorted(set(counter(r) for r in rows if r[identity] == instance))
            gaps[f'{kind}:{instance}'] = sum(b - a - 1 for a, b in zip(ids, ids[1:]))
    losses = {i: dict(dropped_period_samples=max(b['dropped_period_samples'] for b in batches if b['instance'] == i),
                      dropped_application_records=max(b['dropped_application_records'] for b in batches if b['instance'] == i))
              for i in {b['instance'] for b in batches}}
    results = []
    for target in sent:
        stamp = target['source_stamp_ns']
        fs = [f for f in forwards if f['source_stamp_sec'] * 10**9 + f['source_stamp_nanosec'] == stamp
              and f.get('source_frame_id') == target['run']]
        cs = [c for c in callbacks if c['source_stamp_ns'] == stamp
              and c.get('source_frame_id') == target['run']]
        apps = [a for a in applications if a['source_stamp_ns'] == stamp and any(
            (c['instance'], c['activation'], c['sequence']) ==
            (a['instance'], a['activation'], a['sequence'])
            and a.get('callback_activation', a['activation']) == a['activation'] for c in cs)]
        row = dict(source_stamp_ns=stamp, status='unmatched', sent_target=target['q'],
                   forwarder_records=len(fs), callback_records=len(cs), application_records=len(apps))
        if len(fs) > 1 or len(cs) > 1 or len(apps) > 1:
            row['status'] = 'ambiguous'
        elif apps:
            a = apps[0]
            row.update(status='applied', instance=a['instance'], activation=a['activation'],
                       sequence=a['sequence'], q_desired=a['q_desired'])
            if cs and (cs[0]['instance'], cs[0]['activation'], cs[0]['sequence'], cs[0]['t3_ns']) != (
                    a['instance'], a['activation'], a['sequence'], a['t3_ns']):
                row['status'] = 'ambiguous'
            elif fs and fs[0]['publication_call_start_monotonic_ns'] is not None and same_clock(target, fs[0], a):
                f = fs[0]
                row['intervals_ns'] = dict(t0_t1=f['server_receipt_monotonic_ns'] - target['t0_ns'],
                    t1_t2=f['publication_call_start_monotonic_ns'] - f['server_receipt_monotonic_ns'],
                    t2_t3=a['t3_ns'] - f['publication_call_start_monotonic_ns'],
                    t3_t4=a['t4_ns'] - a['t3_ns'], t0_t4=a['t4_ns'] - target['t0_ns'])
            else:
                row['intervals_unavailable'] = 'missing evidence or different host/boot/clock/time namespace'
        elif (fs and fs[0]['outcome'] == 'rejected') or (cs and cs[0]['outcome'] != 'accepted'):
            row['status'] = 'rejected'
        elif fs and fs[0]['outcome'] == 'publication_error':
            row['status'] = 'publication_error'
        elif cs:
            c = cs[0]
            later = any(a['instance'] == c['instance'] and a['activation'] == c['activation']
                        and a['sequence'] > c['sequence'] for a in applications)
            complete = not gaps.get('controller:' + c['instance'], 0) and not losses.get(
                c['instance'], {}).get('dropped_application_records', 0)
            # Require a batch before T3 to avoid claiming supersession when an
            # application batch at the start boundary may have been lost.
            covered = any(b['instance'] == c['instance'] and b['evidence_id'] < c['evidence_id'] for b in batches)
            row['status'] = 'superseded' if later and complete and covered else 'accepted_application_unresolved'
            if row['status'] != 'superseded':
                row['unresolved_reason'] = ('lost_evidence' if not complete else
                                            'no_proof_of_application_or_supersession')
        results.append(row)
    return dict(targets=results, controller_publication_errors_cumulative={
                i: max(r.get('publication_errors', 0) for r in controller if r['instance'] == i)
                for i in {r['instance'] for r in controller}}, periods=[dict(instance=i, activation=a, **period_statistics(v))
                for (i, a), v in groups.items()], evidence_id_gaps=gaps, rt_overflow_cumulative=losses,
                capture_boundaries='Loss before first/after last evidence ID is unknown; unresolved is not proof of rejection or supersession.',
                unmatched_forwarder_records=sum(not any(
                    f['source_stamp_sec'] * 10**9 + f['source_stamp_nanosec'] == t['source_stamp_ns']
                    and f.get('source_frame_id') == t['run'] for t in sent) for f in forwards),
                unmatched_callback_records=sum(not any(c['source_stamp_ns'] == t['source_stamp_ns']
                    and c.get('source_frame_id') == t['run'] for t in sent) for c in callbacks),
                unmatched_application_records=sum(not any(
                    c['source_stamp_ns'] in stamps and c.get('source_frame_id') in {t['run'] for t in sent}
                    and (c['instance'], c['activation'], c['sequence']) ==
                    (a['instance'], a['activation'], a['sequence']) for c in callbacks) for a in applications))

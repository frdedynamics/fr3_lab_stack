import pytest
from fr3_lab_stack_runtime.timing_evidence import analyze, clock_provenance, period_statistics, same_clock


def test_clock():
    p = clock_provenance()
    assert p['clock_verified'] and 'CLOCK_MONOTONIC' in p['python_clock_implementation']
    assert same_clock(p, p)
    for key in ('clock', 'hostname', 'boot_id', 'time_namespace'):
        assert not same_clock(p, dict(p, **{key: 'different'}))


def test_statistics():
    s = period_statistics([1_000_000, 1_500_000, 2_000_000, 2_500_000])
    assert s['count'] == 4
    assert s['mean_ns'] == s['median_ns'] == 1_750_000
    assert s['p95_ns'] == s['p99_ns'] == s['maximum_ns'] == 2_500_000
    assert s['long_period_counts'] == {'1500000': 2, '2000000': 1}
    assert s['mean_deviation_ns'] == 750_000
    assert s['maximum_absolute_deviation_ns'] == 1_500_000
    s = period_statistics(list(range(1, 101)))
    assert s['p95_ns'] == 95 and s['p99_ns'] == 99
    assert period_statistics([])['mean_ns'] is None
    assert period_statistics([1_000_000])['long_period_counts']['1500000'] == 0


P = dict(clock='CLOCK_MONOTONIC', hostname='host', boot_id='boot', time_namespace='time')


def fixture(activation=7, dropped=0, missing=False):
    sent = dict(kind='sent', run='run', source_stamp_ns=123, q=[0.] * 7, t0_ns=10, **P)
    cb = dict(event='callback', evidence_id=2, instance='i', activation=7, sequence=1,
              t3_ns=30, source_stamp_ns=123, outcome='accepted', source_frame_id='run', **P)
    app = dict(activation=activation, sequence=2, source_stamp_ns=124, t3_ns=40,
               t4_ns=50, q_desired=[0.] * 7)
    batch = dict(event='samples', evidence_id=4 if missing else 3, instance='i',
                 activation=activation, samples=[dict(cycle=1, activation=activation,
                 period_ns=1_000_000, application=app)], dropped_period_samples=dropped,
                 dropped_application_records=dropped, **P)
    pre = dict(batch, evidence_id=1, samples=[])
    return [sent, dict(kind='controller', data=pre), dict(kind='controller', data=cb),
            dict(kind='controller', data=batch)]


def test_superseded_and_lost_evidence():
    assert analyze(fixture())['targets'][0]['status'] == 'superseded'
    for rows in (fixture(activation=8), fixture(dropped=1), fixture(missing=True)):
        assert analyze(rows)['targets'][0]['status'] == 'accepted_application_unresolved'
    assert analyze(fixture(missing=True))['evidence_id_gaps']['controller:i'] == 1
    assert analyze(fixture(dropped=1))['rt_overflow_cumulative']['i']['dropped_application_records'] == 1


def test_applied_start_timestamp_and_activation_guard():
    rows = fixture()
    rows[-1]['data']['samples'][0]['application'].update(source_stamp_ns=123, sequence=1, t3_ns=30)
    f = dict(source_frame_id='run', source_stamp_sec=0, source_stamp_nanosec=123, forwarder_instance='f',
             counts=dict(received=1), outcome='forwarded', server_receipt_monotonic_ns=20,
             publication_call_start_monotonic_ns=25, publication_call_end_monotonic_ns=45, **P)
    rows.append(dict(kind='forwarder', data=f))
    r = analyze(rows)['targets'][0]
    assert r['status'] == 'applied'
    assert r['intervals_ns'] == dict(t0_t1=10, t1_t2=5, t2_t3=5, t3_t4=20, t0_t4=40)
    f['hostname'] = 'remote'
    assert 'intervals_ns' not in analyze(rows)['targets'][0]
    rows[2]['data']['activation'] = 8
    assert analyze(rows)['targets'][0]['status'] == 'accepted_application_unresolved'


def test_duplicates_rejected_unmatched():
    rows = fixture()
    with pytest.raises(ValueError, match='unique'):
        analyze(rows + [rows[0]])
    rows[2]['data']['outcome'] = 'inactive'
    assert analyze(rows)['targets'][0]['status'] == 'rejected'
    assert analyze([rows[0]])['targets'][0]['status'] == 'unmatched'
    assert analyze(rows)['unmatched_application_records'] == 1


def test_run_identity_and_missing_callback_are_not_false_matches():
    rows = fixture()
    rows[-1]['data']['samples'][0]['application'].update(source_stamp_ns=123, sequence=1, t3_ns=30)
    rows[2]['data']['source_frame_id'] = 'previous-run'
    result = analyze(rows)
    assert result['targets'][0]['status'] == 'unmatched'
    assert result['unmatched_callback_records'] == 1
    assert result['unmatched_application_records'] == 1
    with pytest.raises(ValueError, match='one run'):
        analyze(rows + [dict(rows[0], run='other', source_stamp_ns=999)])


def test_callback_crossing_activation_is_unresolved():
    rows = fixture()
    rows[-1]['data']['samples'][0]['application'].update(
        source_stamp_ns=123, sequence=1, t3_ns=30, callback_activation=6)
    assert analyze(rows)['targets'][0]['status'] == 'accepted_application_unresolved'

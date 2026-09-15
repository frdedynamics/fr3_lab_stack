# C1-A2 measured-q characterization — 2026-09-15

**C1-A2 passed: measured-q timing characterization.**

Twenty measured-q commands at nominal 10 Hz, collected at approximately 14:30
Europe/Oslo on 2026-09-15. The operator executed this run; finalization only verified
and archived it. No physical C1-B execution was performed in this task.

## Verified results

| Interval | Mean (ms) | Median (ms) | p95 (ms) | Maximum (ms) |
| --- | ---: | ---: | ---: | ---: |
| T0→T1 client→forwarder | 0.198 | 0.183 | 0.289 | 0.299 |
| T1→T2 forwarder processing | 0.028 | 0.027 | 0.043 | 0.047 |
| T2→T3 forwarder→controller callback | 0.130 | 0.127 | 0.168 | 0.221 |
| T3→T4 callback→RT application | 0.496 | 0.466 | 0.910 | 0.957 |
| **T0→T4 end-to-end** | **0.852** | **0.842** | **1.219** | **1.449** |

These rounded values were independently recomputed from raw T0–T4 timestamps.
`verification.json` retains nanosecond statistics, including minima and p99.
Percentiles use nearest rank `ceil(p*N)`, one-based; median averages the middle
pair for even N. T2 is publish-call start; call ends are separately retained.
T4 is software equilibrium installation, not actuator response or convergence.

- **20/20 applied**, with exactly one forwarder, callback, and application record
  per target; sent target and applied `q_desired` match exactly across all 140 values.
- Twenty unique source stamps and consecutive controller sequences **2–21** in one
  controller instance and activation. Sequence 1 is outside this recorded run.
- **6000 consecutive update samples**, cycle indices **456749–462748**, spanning
  discovery/collection/drain windows; no duplicate or missing cycle in this window.
- Zero client/forwarder/controller-evidence publication errors, interior evidence-ID
  gaps, RT period-sample drops, RT application-record drops, unmatched records,
  rejected targets, superseded targets, ambiguous matches, or unresolved targets.
- Actual T0 spacing over 19 intervals: mean **100.296949 ms**, minimum **100.177053 ms**,
  maximum **100.368110 ms**. The requested interval was 100 ms (nominal 10 Hz).

| Supplied controller period statistic | Result |
| --- | ---: |
| Count | 6000 |
| Mean | 0.9999972675 ms |
| Median | 0.999384 ms |
| p95 | 1.081312 ms |
| p99 | 1.104866 ms |
| Maximum | 1.183844 ms |
| Minimum | 0.826325 ms |
| Mean signed deviation from nominal 1 ms | −2.7325 ns |
| Maximum absolute deviation from nominal 1 ms | 183844 ns |
| Long periods strictly above 1.5 ms | 0 |
| Long periods strictly above 2.0 ms | 0 |

T3→T4 spans **0.043091–0.957371 ms**, consistent with asynchronous callback arrival
within a nominal 1 ms update cycle. There is no observed additional software delay
in this short run. Mean and worst end-to-end latency are about **1.28%** and
**2.17%** of a 66.667 ms DROID action period, respectively.

This validates the timing infrastructure and communication/application path for
**zero-displacement measured-q targets at nominal 10 Hz**. It does not validate
moving 15 Hz execution, a full DROID action sequence, policy inference, physical
tracking accuracy, or worst-case latency under other loads. Evidence outside the
capture boundaries remains unknown. **C1-B is next; moving 15 Hz execution and
policy inference remain unvalidated.**

## Runtime and provenance

Tested base: `67a99139de545f95038625d7dbe2be68d2630bec`, plus the uncommitted C1-A2
implementation now preserved in `implementation.patch`. The finalization commit
contains that implementation, its tests, documentation, and this archive.
The patch includes modified and newly added implementation/test files; it excludes
narrative documentation and evidence to avoid recursive provenance.
`provenance.json` records per-file SHA-256 hashes and installed-artifact hashes at
archival. No code was changed during this documentation finalization. Runtime did
not record its loaded binary hash; archival hashes are not a historical binary
attestation.

- ROS 2 Jazzy, collector `--count 20 --interval 0.1`; fresh measured-q state, no offsets.
- Controller: `fr3`, empty arm prefix, hybrid impedance enabled; joint K
  `[40,30,50,25,35,25,10]`, D `[4,6,5,5,3,2,1]`; Cartesian K
  `[750,750,750,15,15,15]`, D `[37,37,37,2,2,2]`; target age 0.25 s,
  future tolerance 0.05 s, unchanged 39-field state at configured 100 Hz.
  The operator confirmed the checked-in YAML was used **without overrides**.
  The process had stopped at archival; no collection-time controller parameter
  dump is available. The complete YAML is preserved alongside this report.
- Forwarder parameters are directly recorded on every target: max age 0.25 s,
  future tolerance 0.05 s, controller polling 0.1 s, evidence expiry 0.3 s.
- Run UUID: `5dc8b3c3-a130-4ecd-b945-5d9bdf4931c8`.
- Controller instance: `23851-9983292980857`; activation: `9989306644906`.
- Forwarder instance: `bd22a820-6e30-41aa-9d30-de31a88b6191`.
- All recorded processes agree on `CLOCK_MONOTONIC`, hostname `hvl-robotics2404`,
  boot ID `81b6e542-1538-4b24-8f5b-1783bb53c33e`, time namespace
  `time:[4026531834]`. Python records `clock_gettime(CLOCK_MONOTONIC)` and successful
  startup clock verification; C++ explicitly uses the same POSIX clock. ROS/source
  stamps are retained for correlation/freshness, never subtracted from monotonic time.

## Evidence and reproduction

- `c1-a2-20-20260915T143031.jsonl`: byte-for-byte copy of the raw recording.
- `c1-a2-20-20260915T143031-summary.json`: byte-for-byte copy of operator analysis.
- `commands.sh`: exact operator collection/analysis commands and resolved output path.
- `verify.py`, `verification.json`: offline assertion checks and recomputed statistics.
- `implementation.patch`, `provenance.json`: tested base plus implementation identity.
- Both configuration YAMLs: archived checked-in settings.
- `software-test-results.txt`: **112 tests, 0 errors, 0 failures, 0 skipped**.
- `SHA256SUMS`: integrity manifest for every archive file except the manifest itself.

From the package root, these commands perform **offline verification only**:

```bash
cd /home/hvl-robotics2404/franka_ros2_ws/src/fr3_lab_stack
PYTHONPATH=. PYTHONNOUSERSITE=1 /usr/bin/python3 \
  docs/validation/2026-09-15_c1-a2_measured-q_143031/verify.py
cd docs/validation/2026-09-15_c1-a2_measured-q_143031
sha256sum -c SHA256SUMS
```

The implementation was software-validated with:

```bash
cd /home/hvl-robotics2404/franka_ros2_ws
PYTHONNOUSERSITE=1 colcon build --packages-select fr3_lab_stack \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
PYTHONNOUSERSITE=1 colcon test --packages-select fr3_lab_stack
colcon test-result --test-result-base build/fr3_lab_stack --verbose
```

The saved analyzed JSON and the summary embedded in the raw JSONL both match a
fresh offline analysis exactly. No evidence file was edited to produce the match.

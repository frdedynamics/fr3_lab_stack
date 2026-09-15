# Historical collection and analysis commands, supplied by the operator.
# Collection publishes measured-q targets; do not run merely to inspect the archive.
cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT="/tmp/c1-a2-20-$(date +%Y%m%dT%H%M%S).jsonl"
ros2 run fr3_lab_stack fr3_streaming_timing_collector \
  --count 20 \
  --interval 0.1 \
  --output "$OUT"
ros2 run fr3_lab_stack fr3_streaming_timing_collector \
  --analyze "$OUT" \
  | tee "${OUT%.jsonl}-summary.json"
echo "raw:     $OUT"
echo "summary: ${OUT%.jsonl}-summary.json"
# OUT resolved to /tmp/c1-a2-20-20260915T143031.jsonl for this run.

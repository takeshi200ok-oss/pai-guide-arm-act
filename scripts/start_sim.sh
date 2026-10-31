#!/bin/bash
# 案内板付きワールドで CRANE+ V2 を起動し、コントローラが active になるまで待つ
# 使い方: start_sim.sh [gui:=false|true]
# 起動が 90 秒以内に終わらなければ（Gazebo が固まることがある）1回だけ起動し直す。
source ~/guide_arm/scripts/env.sh
GUI_ARG=${1:-gui:=false}
for attempt in 1 2; do
  bash ~/guide_arm/scripts/stop_sim.sh
  nohup ros2 launch ~/guide_arm/launch/guide_sim.launch.py "$GUI_ARG" > "$GUIDE_RUNS/logs/gazebo.log" 2>&1 &
  t0=$(date +%s)
  while [ $(( $(date +%s)-t0 )) -lt 90 ]; do
    out=$(timeout 10 ros2 control list_controllers 2>/dev/null)
    if echo "$out" | grep -q "crane_plus_arm_controller.*active" && echo "$out" | grep -q "crane_plus_gripper_controller.*active"; then
      echo "sim ready in $(( $(date +%s)-t0 ))s ($GUI_ARG, attempt $attempt)"
      exit 0
    fi
    sleep 1
  done
  echo "sim start timeout (attempt $attempt)"
done
exit 1

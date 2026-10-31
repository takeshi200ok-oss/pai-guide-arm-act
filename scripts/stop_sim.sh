#!/bin/bash
# Gazebo と関連ノードを停止する（このスクリプト自身にはマッチしないパターンを使う）
pats=("ros2 launch" "ign gazebo" "parameter_bridge" "robot_state_publisher" "move_group" "rviz2"
      "spawner" "ros2 bag" "infer_node.py" "teacher.py")
for p in "${pats[@]}"; do pkill -INT -f "$p" 2>/dev/null; done
sleep 3
for p in "${pats[@]}"; do pkill -KILL -f "$p" 2>/dev/null; done
exit 0

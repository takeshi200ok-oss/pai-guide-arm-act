#!/bin/bash
# コンテナ内だけにある大きな成果物（rosbag・データセット・重み）と実行記録を、
# リポジトリ外の ../guide_arm_backup/ へコピーする（Git Bash から実行）。
cd "$(dirname "$0")"
dst=../guide_arm_backup
mkdir -p "$dst"
for d in bags datasets checkpoints; do
  MSYS_NO_PATHCONV=1 docker cp pai_ros2:/home/ubuntu/guide_runs/$d "$dst/"
done
du -sh "$dst"/*

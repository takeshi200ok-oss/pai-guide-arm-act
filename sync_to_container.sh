#!/bin/bash
# ホストの guide_arm をコンテナ pai_ros2 の ~/guide_arm へコピーする（Git Bash から実行）
cd "$(dirname "$0")"
MSYS_NO_PATHCONV=1 docker exec -u ubuntu pai_ros2 mkdir -p /home/ubuntu/guide_arm /home/ubuntu/guide_runs
for d in configs launch scripts worlds video; do
  MSYS_NO_PATHCONV=1 docker cp "$d" pai_ros2:/home/ubuntu/guide_arm/
done
MSYS_NO_PATHCONV=1 docker exec -u root pai_ros2 chown -R ubuntu:ubuntu /home/ubuntu/guide_arm

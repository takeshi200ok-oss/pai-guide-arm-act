#!/bin/bash
# Stage B（初期姿勢を変えた教師データ）の収集・変換・学習。手順は Stage A の v7 と同じで、
# 違いは collect_demos.py に --starts-file（start_poses.json の train 一覧）を渡すことだけ。
set -e
source ~/guide_arm/scripts/env.sh
cd ~/guide_arm/scripts
PX="pixi run --manifest-path $PAI_TRAINING/pyproject.toml"
L=$GUIDE_RUNS/logs
python3 -c "
import json
d = json.load(open('$GUIDE_ARM/configs/start_poses.json'))
json.dump([{k: v for k, v in p.items() if k != 'id'} for p in d['lists']['train']], open('$GUIDE_RUNS/starts_train.json', 'w'), indent=1)"
bash start_sim.sh gui:=false
for d in P N; do
  python3 collect_demos.py --profile linear --direction $d --episodes 20 \
    --starts-file $GUIDE_RUNS/starts_train.json --out $GUIDE_RUNS/bags/guide_${d}_B_lin > $L/collect_${d}_B_lin.log 2>&1
  tail -1 $L/collect_${d}_B_lin.log
  $PX python convert.py --trim-start-hold --bags $GUIDE_RUNS/bags/guide_${d}_B_lin \
    --out $GUIDE_RUNS/datasets/guide_${d}_B_lin_trim2 --repo-id guide_${d}_B_lin_trim2 --task guide_$d > $L/convert_${d}_B_lin.log 2>&1
  $PX python check_dataset.py $GUIDE_RUNS/datasets/guide_${d}_B_lin_trim2 > $L/check_${d}_B_lin.log 2>&1
  grep columns $L/check_${d}_B_lin.log
done
bash stop_sim.sh
for d in P N; do
  $PX python train.py --config $GUIDE_ARM/configs/train_${d}_B.yaml > $L/train_${d}_B_chunk.log 2>&1
  tail -1 $L/train_${d}_B_chunk.log
done

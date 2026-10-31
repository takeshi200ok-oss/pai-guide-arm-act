#!/bin/bash
# 比較に使った 4 モデルを、GitHub Releases に添付するアーカイブにまとめる（Git Bash から実行）。
# 入力: ../guide_arm_backup/checkpoints/<model>/（backup_from_container.sh で退避したもの）
# 出力: ../release_assets/<model>.tar.gz と SHA256SUMS
set -e
cd "$(dirname "$0")"
src=../guide_arm_backup/checkpoints
out=../release_assets
mkdir -p "$out"
declare -A COND=([A]="初期姿勢を基準姿勢（全関節0）に固定した教師デモ20本" [B]="初期姿勢を joint1 ∈ [-0.15, +0.15]、joint3 ∈ [0, 0.10] rad で変えた教師デモ20本（configs/start_poses.json の train 一覧）")
declare -A DIRN=([P]="右（P 方向、joint1 = +0.8 rad、橙の板）" [N]="左（N 方向、joint1 = -0.8 rad、青い板）")
: > "$out/SHA256SUMS"
for c in A B; do
  for d in P N; do
    m=act_${d}_${c}_chunk_s2000
    tmp=$(mktemp -d)
    mkdir -p "$tmp/$m"
    cp "$src/$m"/{model.safetensors,config.json,train_config.yaml,train_summary.json,dataset_info.json,loss.csv} "$tmp/$m/"
    sha=$(sha256sum "$src/$m/model.safetensors" | cut -d' ' -f1)
    cat > "$tmp/$m/MODEL_README.md" <<EOF
# $m

CRANE+ V2 の案内動作を学習した ACT（LeRobot）のチェックポイント。pai-guide-arm-act の比較実験（Stage B）で使った 4 モデルの 1 つ。

| 項目 | 内容 |
|---|---|
| 方向 | ${DIRN[$d]} |
| 条件 | $c：${COND[$c]} |
| 学習 | v7 の手順（train_config.yaml）。2000 更新、バッチ 8、seed 0、CPU |
| 入出力 | 入力: 関節状態 5 次元（joint1, joint2, joint4, joint3, gripper）。出力: 関節指令 20 ステップ（0.05 秒間隔）、うち 10 ステップを実行 |
| 正規化 | 学習データの平均・標準偏差（下限 0.05 rad）。値は model.safetensors に含まれる |
| model.safetensors の SHA-256 | $sha |

## 使い方
ACTPolicy.from_pretrained(<このフォルダ>) で読み込める。リポジトリの scripts/infer_node.py と scripts/evaluate.py で Gazebo 上で実行・評価できる（README の「配布モデルで動かす」を参照）。

## 注意
- Gazebo 上の CRANE+ V2 と、リポジトリの案内板付きワールド・目標姿勢でのみ評価したもの。実機や他の環境での動作は確認していない。
- 教師データはスクリプトで生成した実演（人の操作ではない）。
- 利用条件: Apache License 2.0（リポジトリの LICENSE と同じ）。
EOF
    tar -czf "$out/$m.tar.gz" -C "$tmp" "$m"
    rm -rf "$tmp"
    (cd "$out" && sha256sum "$m.tar.gz" >> SHA256SUMS)
    echo "$m: model.safetensors $sha"
  done
done
ls -la "$out"
cat "$out/SHA256SUMS"

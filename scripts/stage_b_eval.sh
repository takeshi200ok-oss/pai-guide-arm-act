#!/bin/bash
# start_poses.json の一覧（dev / eval）の各初期姿勢から、条件 A・B の ACT を P/N それぞれで評価する。
# 使い方: stage_b_eval.sh <dev|eval> <出力ディレクトリ> [使う姿勢の数（省略時は全部）]
# - 姿勢ごとに A→B と B→A の順を交互にする（実行順の偏りを抑える）
# - evaluate.py が異常終了した、または status が start_timeout のときだけ、シミュレータを起動し直して
#   同じ姿勢 ID で再試行する（最大2回）。失敗した試行の記録は *_attemptN として残す
# - 方策が動いたうえでの失敗（status: fail）は再試行しない
source ~/guide_arm/scripts/env.sh
cd ~/guide_arm/scripts
list=$1; out=$2; limit=${3:-999}
mkdir -p "$out"
declare -A MODEL=(
  [A_P]=$GUIDE_RUNS/checkpoints/act_P_A_chunk_s2000 [B_P]=$GUIDE_RUNS/checkpoints/act_P_B_chunk_s2000
  [A_N]=$GUIDE_RUNS/checkpoints/act_N_A_chunk_s2000 [B_N]=$GUIDE_RUNS/checkpoints/act_N_B_chunk_s2000)
mapfile -t POSES < <(python3 -c "
import json
d = json.load(open('$GUIDE_ARM/configs/start_poses.json'))
for p in d['lists']['$list'][:$limit]:
    print(p['id'], ','.join(f'{k}={v}' for k, v in p.items() if k != 'id'))")
bash start_sim.sh gui:=false || exit 1
i=0
for line in "${POSES[@]}"; do
  id=${line%% *}; pose=${line#* }
  if (( i % 2 == 0 )); then order="A B"; else order="B A"; fi
  for d in P N; do
    for c in $order; do
      tag=${c}_${d}_${id}
      for attempt in 1 2 3; do
        python3 evaluate.py --controller policy --policy "${MODEL[${c}_${d}]}" --direction $d \
          --starts "$pose" --reference $GUIDE_ARM/configs/reference.json --out-dir "$out" --tag $tag \
          > "$out/$tag.evaluate.log" 2>&1
        rc=$?
        status=$(python3 -c "import json; print(json.load(open('$out/${tag}_trial0.json'))['status'])" 2>/dev/null)
        if [ $rc -eq 0 ] && [ -n "$status" ] && [ "$status" != "start_timeout" ]; then break; fi
        echo "infra problem: $tag attempt $attempt rc=$rc status=$status -> restart sim"
        for f in "$out/${tag}_trial0.json" "$out/$tag.evaluate.log"; do [ -e "$f" ] && mv "$f" "${f%.*}_attempt$attempt.${f##*.}"; done
        bash start_sim.sh gui:=false
      done
      echo "$tag $status"
    done
  done
  i=$((i + 1))
done

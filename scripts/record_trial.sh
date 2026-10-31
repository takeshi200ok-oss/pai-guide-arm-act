#!/bin/bash
# GUI ありのシミュレータで1試行を録画する（数値評価とは別扱い。結果 JSON には RTF も残る）
# Gazebo GUI 内蔵の動画記録（/gui/record_video）を使う。3D シーンだけが記録され、
# デスクトップ上に重なる表示は映らない（x11grab での画面キャプチャでは映り込んだ）。
# 使い方: record_trial.sh <policy|teacher> <P|N> <出力名> [policy のパス]
source ~/guide_arm/scripts/env.sh
cd ~/guide_arm/scripts
ctrl=$1; dir=$2; name=$3; policy=$4
out=$GUIDE_RUNS/videos; mkdir -p "$out"
rec() {  # $1 = start | stop
  ign service -s /gui/record_video --reqtype ignition.msgs.VideoRecord --reptype ignition.msgs.Boolean \
    --timeout 5000 --req "$1: true, format: \"mp4\", save_filename: \"$out/${name}_tmp.mp4\"" > /dev/null
}
bash set_camera.sh > /dev/null
python3 teacher.py reset > /dev/null
rec start
args=(--controller "$ctrl" --direction "$dir" --reference "$GUIDE_ARM/configs/reference.json" --out-dir "$out" --tag "$name")
[ -n "$policy" ] && args+=(--policy "$policy")
python3 evaluate.py "${args[@]}" 2>&1 | grep trial
rec stop
sleep 3  # 書き出しの完了を待つ
# 次の録画を始めると前の保存ファイルが消えることがあったので、実体を別名にコピーしておく
cp -L "$out/${name}_tmp.mp4" "$out/$name.mp4" && rm -f "$out/${name}_tmp.mp4"
ffmpeg -loglevel error -y -sseof -0.5 -i "$out/$name.mp4" -frames:v 1 "$out/${name}_last.png"

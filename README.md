# pai-guide-arm-act — CPUで動かす模倣学習の案内アーム

Gazebo 上の CRANE+ V2 が、左右に置いた案内板（左＝青、右＝橙）の方向へ腕を傾けて示す「案内動作」を、
**スクリプトで生成した実演データから ACT（Action Chunking with Transformers）で学習**し、**CPU のみ**で実行・評価したものです。
Physical AI 応用編1（東京大学 松尾・岩澤研究室）の最終課題として作成しました。

- 動画: https://youtu.be/RQfaNdsQ9cc （YouTube）
- 学習済みモデル: GitHub Releases v1.0.0（https://github.com/takeshi200ok-oss/pai-guide-arm-act/releases/tag/v1.0.0）
- 説明資料（PDF）: 提出物として別に提出

| | 内容 |
|---|---|
| AI の担当 | ACT が現在の関節状態から、案内動作の関節指令列を生成する |
| 通常の制御の担当 | 左右の選択（起動時に方向ごとのモデルを指定）、初期姿勢への復帰、成功判定 |
| 教師データ | 人のテレオペではなく、スクリプトで生成した実演（等速補間）を Gazebo で実行して記録 |
| 環境 | ROS 2 Humble、Ignition Gazebo 6、LeRobot（ACT）、PyTorch 2.7（CPU） |

## 結果の要約

**比較（Stage B、GUI なし、80 試行）**: 教師デモの初期姿勢を固定した条件 A と、初期姿勢を変えた条件 B を、
事前に固定した未使用の 20 姿勢で比較しました（[comparison.md](results/eval_B_final/comparison.md)）。

| 方向 | 条件 | 成功 | 到達時間 平均 [sim 秒] | 8 sim 秒時点の最大誤差 [rad] |
|---|---|---|---|---|
| 右（P） | A | 20/20 | 2.41 | 0.0008 |
| 右（P） | B | 20/20 | 2.70 | 0.0065 |
| 左（N） | A | 20/20 | 2.59 | 0.0060 |
| 左（N） | B | 20/20 | 3.22 | 0.0102 |

- 今回の範囲（joint1 ±0.15、joint3 0〜0.10 rad）では、B による成功率の改善は確認できませんでした。A と B が一般に同等とは結論しません。
- 学習 seed は各 1 つ、評価は同じ 20 姿勢を 4 モデルに与えたものです。範囲外の初期姿勢は未検証です。
- 開発中に観測した停止・逆向きと、その対策（正規化の下限、動き出しの区間の扱い、チャンク実行）の記録は [configs/](configs/) と [results/eval_A/](results/eval_A/) にあります。
  `train_*_A_v1`〜`v6` は開発版（版によって片方向は成功したものもある）、`train_*_A.yaml` が採用した v7 です。原因はいずれも検査からの推定です。

## フォルダ構成

```
configs/   目標姿勢・カメラ・成功判定・参照姿勢・初期姿勢一覧・学習設定（開発版 v1〜v6 の設定を含む）
launch/    MoveIt を起動せず、GUI なしでも起動できる Gazebo の launch
worlds/    案内板を置いたワールド
scripts/   教師・収集・変換・学習・推論・評価・比較集計・録画のスクリプト
video/     提出動画の生成スクリプト（make_video.py）と字幕の元テキスト（captions.yaml）
results/   評価結果（JSON）、学習の要約、凍結記録、ログ、録画
```

## 1. 環境を用意する

講義（第4〜5回）で使った環境を使います。作者の PC では講義時にセットアップ済みの環境を使ったため、
**以下の手順を新しいコンテナで最初から通しては確認していません**。詳細は講義資料と各リポジトリの README に従ってください。

1. Docker イメージを起動する（講義の起動スクリプトと同じ設定。noVNC は http://localhost:6080 ）
   ```bash
   docker pull airobotbook/ros2-desktop-ai-robot-book-humble:latest
   # 確認した版: sha256:b58ec74c019aacd9ce4818fa25fc5a9e0a1f22f8c9cd1f5260994bf15cd9eb63
   docker run -d --name pai_ros2 -p 6080:80 --shm-size=512m -p 9090:9090 -p 9876:9876 \
     airobotbook/ros2-desktop-ai-robot-book-humble:latest
   ```
2. コンテナ内（ユーザー ubuntu）で、講義の教材を取得する
   ```bash
   # テレオペ用パッケージ（本リポジトリでは使わないが、講義の手順どおりワークスペースを作る）
   mkdir -p ~/pai_ws/src && cd ~/pai_ws/src && git clone https://github.com/matsuolab/physicalai_teleop.git
   cd ~/pai_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install
   # 学習環境（確認した版: pai_training b4d6b545、lerobot submodule 768e3666、pixi 0.81.0）
   cd ~ && git clone --recursive https://github.com/matsuolab/pai_training.git
   cd ~/pai_training && git checkout b4d6b545 && git submodule update --init --recursive
   curl -fsSL https://pixi.sh/install.sh | bash   # pixi（https://pixi.sh）
   cd ~/pai_training && ~/.pixi/bin/pixi install
   ```
3. 本リポジトリをコンテナの `~/guide_arm` に置く
   ```bash
   # ホストで（Windows の Git Bash の場合は sync_to_container.sh でも可）
   docker cp configs launch scripts worlds video pai_ros2:/home/ubuntu/guide_arm/
   ```
   コンテナ内では `~/guide_arm` にコード、`~/guide_runs` に出力を置きます。
   実行した PC: AMD Ryzen 7 7735HS（16 スレッド）、メモリ 32GB、CUDA なし。

## 2. 配布モデルで動かす（再学習なし）

[GitHub Releases v1.0.0](https://github.com/takeshi200ok-oss/pai-guide-arm-act/releases/tag/v1.0.0) から、条件 A の左右のモデル `act_N_A_chunk_s2000.tar.gz`、`act_P_A_chunk_s2000.tar.gz` を取得し、
[SHA256SUMS](#学習済みモデル) で確認してから、コンテナの `~/guide_models/` に展開します。

```bash
mkdir -p ~/guide_models
tar -xzf act_N_A_chunk_s2000.tar.gz -C ~/guide_models
tar -xzf act_P_A_chunk_s2000.tar.gz -C ~/guide_models

source ~/guide_arm/scripts/env.sh
cd ~/guide_arm/scripts
bash start_sim.sh gui:=false          # 画面で見る場合は gui:=true（noVNC で表示。実時間比は下がる）
for d in N P; do
  python3 evaluate.py --controller policy --policy ~/guide_models/act_${d}_A_chunk_s2000 --direction $d \
    --reference ~/guide_arm/configs/reference.json --out-dir ~/guide_runs/demo --tag demo_$d
done
bash stop_sim.sh
```

各試行で初期姿勢へ戻してから推論ノードを起動し、成功判定の結果（`status`、`reach_time` など）を表示します。
作者の環境では、配布アーカイブから展開したモデルで左右とも成功しました（N 2.69 sim 秒、P 2.73 sim 秒）。

## 3. 学習から評価までを再実行する

```bash
source ~/guide_arm/scripts/env.sh
cd ~/guide_arm/scripts
PX="pixi run --manifest-path $PAI_TRAINING/pyproject.toml"

# 条件 A（左右とも）: 収集 → 変換 → 学習
bash start_sim.sh gui:=false
for d in P N; do
  python3 collect_demos.py --profile linear --direction $d --episodes 20 --out ~/guide_runs/bags/guide_${d}_A_lin
  $PX python convert.py --trim-start-hold --bags ~/guide_runs/bags/guide_${d}_A_lin \
      --out ~/guide_runs/datasets/guide_${d}_A_lin_trim2 --repo-id guide_${d}_A_lin_trim2 --task guide_$d
  $PX python check_dataset.py ~/guide_runs/datasets/guide_${d}_A_lin_trim2
done
bash stop_sim.sh                       # 学習中はシミュレータを止める（CPU で 1 モデル約 15 分）
for d in P N; do $PX python train.py --config ~/guide_arm/configs/train_${d}_A.yaml; done

# 条件 B（左右とも）: 収集 → 変換 → 学習（初期姿勢は configs/start_poses.json の train 一覧）
bash stage_b_train.sh

# 80 試行の比較評価（A/B × 左右の 4 モデルが ~/guide_runs/checkpoints/ に必要）と集計
bash stage_b_eval.sh eval ~/guide_runs/eval_B_final
python3 compare_ab.py ~/guide_runs/eval_B_final
```

- 80 試行には `act_{P,N}_{A,B}_chunk_s2000` の 4 モデルが必要です。再学習せずに使う場合は、Releases の 4 アーカイブを
  `~/guide_runs/checkpoints/` に展開してください（`stage_b_eval.sh` はこの場所を参照します）。
- 参照姿勢（`configs/reference.json`）、成功判定（`configs/criteria.yaml`）、初期姿勢一覧（`configs/start_poses.json`）は、
  ACT の評価前に固定したものです。作り直す場合は `make_reference.py`、`make_pose_lists.py` を使います（既存ファイルは上書きしません）。
- 学習は seed を固定していますが、CPU の並列処理などにより、同じ重みが再現されることは確認していません。

## 4. 動画を作り直す

`video/make_video.py` は、評価結果と録画から動画を組み立てます。次がそろっている必要があります。

- `~/guide_runs/eval_A/`（開発版の評価 JSON。失敗の軌跡の図に使う）、`~/guide_runs/eval_B_final/comparison.json`（比較）
- `~/guide_runs/videos/act_{N,P}_A_chunk_rec.mp4` と `_trial0.json`（GUI ありの録画試行。`scripts/record_trial.sh` で撮影）
- コンテナ内の ffmpeg、Noto Sans CJK フォント、pai_training の pixi 環境（Pillow、matplotlib）

本リポジトリの `results/` に評価 JSON と録画を収めていますが、コードだけを取得しても同じ録画は作れません（録画は撮影のたびに変わります）。

```bash
cd ~/guide_arm/video
pixi run --manifest-path ~/pai_training/pyproject.toml python make_video.py --runs ~/guide_runs --out ~/guide_runs/video_out \
    --cover <表紙画像> --bgm <音楽ファイル>   # どちらも省略可（省略すると表紙なし・無音）
```

提出動画の冒頭の表紙画像（講座の運営が受講者向けに提供したもの）と BGM は、本リポジトリに含めていません。
BGM は「遠い港の夜景」（作曲：甘茶、[甘茶の音楽工房](https://amachamusic.chagasi.com/)）で、配布元の利用規約に従って使用しました。
作り直す場合は配布元から各自で取得してください（本リポジトリの Apache License 2.0 は、表紙画像と BGM には適用されません）。
`make_video.py` は曲の前後の無音を除き、曲の終わりが動画の終わりに合うようテンポを合わせて（今回は 0.906 倍）、小さめの音量で重ねます。

## 学習済みモデル

重み（各約 154MB）は Git に含めず、[GitHub Releases v1.0.0](https://github.com/takeshi200ok-oss/pai-guide-arm-act/releases/tag/v1.0.0) に 4 つのアーカイブとして添付しています。
各アーカイブには `model.safetensors`、`config.json`（ACT の設定）、学習設定・学習の要約・データ情報・`MODEL_README.md` が入っています。

| アーカイブ | 方向・条件 | model.safetensors の SHA-256 |
|---|---|---|
| act_P_A_chunk_s2000.tar.gz | 右・A | 5da48a1fa418717d6f48c6bb7b226cf7971100c09365e6958c00d86de7c06367 |
| act_N_A_chunk_s2000.tar.gz | 左・A | f2b7cba00947d942f76c11c83b53fcbb701cd16f1b495eed8061192033f88065 |
| act_P_B_chunk_s2000.tar.gz | 右・B | 7be698ff16f5ce6f0102f21dd5d843b7161520cd33b51f210815995c02537a76 |
| act_N_B_chunk_s2000.tar.gz | 左・B | 7030dfcd40db87be9c07936c945a6c33a3311709eae99acf9c3bca09592cb5f2 |

アーカイブ自体の SHA-256 は Releases に添付した `SHA256SUMS` に記載しています。上の値は比較評価の前に記録した凍結記録
（[freeze_stage_b.txt](results/freeze_stage_b.txt)）と一致します。

## 注意

- 到達・保持・判定の時間はシミュレーション秒です。学習・収集の所要時間は実時間です。
- 比較の数値は GUI なしの評価の値です。動画の実演字幕の値は GUI ありの当該録画試行のもので、比較集計には含めていません。
- 講義の `pai_training` のスクリプトは直接使わず、変換・学習・推論を本リポジトリのスクリプトで行います（関節名での整列、シミュレーション時計、チャンク実行などのため）。

## ライセンス・出典

コードと配布モデルは Apache License 2.0（[LICENSE](LICENSE)）。改変・参考にした第三者の成果物と権利表示は [NOTICE](NOTICE) に記載しています。
設計、実装、実験計画および提出資料の作成に生成 AI アシスタントを利用し、出力を実行ログ・評価結果・成果物と照合して確認しました（ロボットの関節指令を生成する ACT とは別です）。

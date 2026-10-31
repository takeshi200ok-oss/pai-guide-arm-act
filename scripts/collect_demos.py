#!/usr/bin/env python3
"""teacher.py の案内動作を rosbag にエピソードとして連続収集する（1エピソード = 1 rosbag）。

各エピソード: 開始姿勢へリセット（記録外） → rosbag 記録開始（--use-sim-time） → 教師の案内動作 → 記録停止。
rosbag のタイムスタンプはシミュレーション時刻になる。開始姿勢と動作パラメータは episodes.json に保存する。
"""
import argparse
import json
import signal
import subprocess
import time
from pathlib import Path

from guide_common import ARM_TOPIC, GRIPPER_TOPIC, JOINT_STATES_TOPIC, format_pose, load_poses, parse_pose

HERE = Path(__file__).resolve().parent
TOPICS = [JOINT_STATES_TOPIC, ARM_TOPIC, GRIPPER_TOPIC]


def record_episode(out_dir, direction, start, move_time, pre_hold, post_hold, profile):
    subprocess.run(['python3', str(HERE / 'teacher.py'), 'reset', '--start', format_pose(start)],
                   check=True, stdout=subprocess.DEVNULL)
    log_path = out_dir.parent / f'{out_dir.name}_bag.log'
    meta_path = out_dir.parent / f'{out_dir.name}_meta.json'
    with open(log_path, 'w') as bag_log:
        bag = subprocess.Popen(['ros2', 'bag', 'record', '--use-sim-time', '-o', str(out_dir), *TOPICS],
                               stdout=bag_log, stderr=subprocess.STDOUT)
        try:
            # 指令トピックは送信側が現れてから購読される。ここでは /joint_states の購読だけ待ち、
            # 指令トピックの購読確立は teacher.py の --wait-subscribers 2 で待つ。
            t0 = time.monotonic()
            while "Subscribed to topic '/joint_states'" not in log_path.read_text():
                if time.monotonic() - t0 > 15 or bag.poll() is not None:
                    raise RuntimeError(f'rosbag の開始を確認できない: {log_path}')
                time.sleep(0.1)
            subprocess.run(['python3', str(HERE / 'teacher.py'), 'demo', '--direction', direction,
                            '--start', format_pose(start), '--move-time', f'{move_time}', '--profile', profile,
                            '--pre-hold', f'{pre_hold}', '--post-hold', f'{post_hold}',
                            '--meta-out', str(meta_path), '--wait-subscribers', '2'],
                           check=True, stdout=subprocess.DEVNULL)
        finally:
            bag.send_signal(signal.SIGINT)
            bag.wait(timeout=15)
    return json.loads(meta_path.read_text())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--direction', choices=['P', 'N'], required=True)
    ap.add_argument('--out', required=True, help='rosbag の出力先（新規ディレクトリ）')
    ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--starts-file', help='開始姿勢の JSON（関節名→角度 の辞書のリスト）。無ければ全エピソード home')
    ap.add_argument('--pre-hold', type=float, default=0.5)
    ap.add_argument('--move-time', type=float, default=2.0)
    ap.add_argument('--post-hold', type=float, default=3.0)
    ap.add_argument('--profile', choices=['linear', 'min_jerk'], default='linear')
    args = ap.parse_args()

    subprocess.run(['ros2', 'daemon', 'start'], check=True, stdout=subprocess.DEVNULL)
    poses = load_poses()
    if args.starts_file:
        starts = [parse_pose(','.join(f'{k}={v}' for k, v in s.items()))
                  for s in json.loads(Path(args.starts_file).read_text())]
        if len(starts) < args.episodes:
            raise SystemExit(f'開始姿勢が {len(starts)} 個しかない（{args.episodes} 必要）')
    else:
        starts = [dict(poses['home'])] * args.episodes
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=False)  # 既存データを上書きしない
    log = {'args': vars(args), 'target': poses['directions'][args.direction], 'episodes': []}
    t_all = time.monotonic()
    for k in range(args.episodes):
        t = time.monotonic()
        meta = record_episode(out / f'ep{k:03d}', args.direction, starts[k],
                              args.move_time, args.pre_hold, args.post_hold, args.profile)
        meta.pop('joint_states')
        meta['wall_time_s'] = time.monotonic() - t
        log['episodes'].append(meta)
        print(f'ep{k:03d}: start={format_pose(starts[k])} final={meta["final_joint_pos"]} '
              f'({meta["wall_time_s"]:.1f}s)', flush=True)
    log['total_wall_time_s'] = time.monotonic() - t_all
    (out / 'episodes.json').write_text(json.dumps(log, indent=1))
    print(f'done: {args.episodes} episodes in {log["total_wall_time_s"]:.1f}s')


if __name__ == '__main__':
    main()

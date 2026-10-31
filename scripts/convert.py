#!/usr/bin/env python3
"""rosbag（collect_demos.py の出力）を LeRobotDataset に変換する（pai_training の pixi 環境で実行）。

pai_training/scripts/rosbag2lerobot.py（講義教材）の処理（各時刻で直前の値を保持してサンプリングし、
アームとグリッパの指令を結合して action にする）を参考に、次を変更した:
  - 関節は名前で取り出す。必要な名前が欠けていれば変換を中止する（0 で埋めない）
  - 列名を実際の値の並び [joint1, joint2, joint4, joint3, gripper] に合わせる
  - サンプリング時刻は start + k / fps（シミュレーション時刻の等間隔）。末尾のパディングはしない
  - 各エピソードの長さ・時刻範囲を <出力>_conversion.json に保存する
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

from guide_common import ARM_JOINTS, ARM_TOPIC, FEATURE_NAMES, GRIPPER_JOINT, GRIPPER_TOPIC, JOINT_STATES_TOPIC

FEATURES = {key: {'dtype': 'float32', 'shape': (5,), 'names': FEATURE_NAMES}
            for key in ('observation.state', 'observation.environment_state', 'action')}


def by_name(names, positions, wanted, where):
    table = dict(zip(names, positions))
    missing = [n for n in wanted if n not in table]
    if missing:
        raise KeyError(f'{where}: 必要な関節名がない {missing}')
    return [table[n] for n in wanted]


def read_episode(bag_dir):
    series = {JOINT_STATES_TOPIC: [], ARM_TOPIC: [], GRIPPER_TOPIC: []}
    with Reader(bag_dir) as reader:
        conns = [c for c in reader.connections if c.topic in series]
        for conn, ts, raw in reader.messages(connections=conns):
            msg = deserialize_cdr(raw, conn.msgtype)
            if conn.topic == JOINT_STATES_TOPIC:
                val = by_name(msg.name, msg.position, ARM_JOINTS + [GRIPPER_JOINT], f'{bag_dir} joint_states')
            elif conn.topic == ARM_TOPIC:
                val = by_name(msg.joint_names, msg.points[0].positions, ARM_JOINTS, f'{bag_dir} arm cmd')
            else:
                val = by_name(msg.joint_names, msg.points[0].positions, [GRIPPER_JOINT], f'{bag_dir} gripper cmd')
            series[conn.topic].append((ts * 1e-9, val))
    for topic, s in series.items():
        if not s:
            raise ValueError(f'{bag_dir}: {topic} が記録されていない')
    return series


def sample_and_hold(series, times):
    ts = np.array([t for t, _ in series])
    vals = np.array([v for _, v in series], dtype=np.float32)
    idx = np.searchsorted(ts, times, side='right') - 1
    return vals[np.clip(idx, 0, None)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='+', required=True, help='rosbag のエピソードディレクトリを含むディレクトリ（複数可）')
    ap.add_argument('--out', required=True, help='LeRobotDataset の出力先（新規）')
    ap.add_argument('--repo-id', required=True)
    ap.add_argument('--task', required=True)
    ap.add_argument('--fps', type=int, default=20)
    ap.add_argument('--trim-start-hold', action='store_true', help='冒頭の静止区間を除く')
    args = ap.parse_args()
    report_args = {'trim_start_hold': args.trim_start_hold}

    episodes = sorted(p.parent for d in args.bags for p in Path(d).expanduser().rglob('*.db3'))
    if not episodes:
        raise SystemExit('rosbag が見つからない')
    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=args.fps, root=Path(args.out).expanduser(),
                               robot_type='crane_plus', features=FEATURES, use_videos=False)
    report = {'fps': args.fps, 'feature_names': FEATURE_NAMES, 'joint_order': ARM_JOINTS + [GRIPPER_JOINT],
              'args': report_args, 'episodes': []}
    for ep_dir in episodes:
        s = read_episode(ep_dir)
        start = max(v[0][0] for v in s.values())
        end = min(v[-1][0] for v in s.values())
        times = start + np.arange(int(np.floor((end - start) * args.fps)) + 1) / args.fps
        state = sample_and_hold(s[JOINT_STATES_TOPIC], times)
        action = np.hstack([sample_and_hold(s[ARM_TOPIC], times), sample_and_hold(s[GRIPPER_TOPIC], times)])
        trimmed = 0
        if args.trim_start_hold:
            # 冒頭の静止区間（指令が開始姿勢のまま）を除き、指令が動き出した最初のフレームから使う。
            # 開始姿勢に「止まれ（指令 = 開始姿勢）」の正解が1つでも残ると、方策の予測が開始姿勢で
            # 逆向きにはみ出すことがあった（Stage A の開発中に確認）。
            moving = np.flatnonzero(np.abs(action[:, :4] - action[0, :4]).max(axis=1) > 1e-6)
            trimmed = int(moving[0]) if len(moving) else 0
            times, state, action = times[trimmed:], state[trimmed:], action[trimmed:]
        for st, ac in zip(state, action):
            st_t = torch.from_numpy(st)
            ds.add_frame({'observation.state': st_t, 'observation.environment_state': st_t.clone(),
                          'action': torch.from_numpy(ac), 'task': args.task})
        ds.save_episode()
        report['episodes'].append({'bag': str(ep_dir), 'frames': len(times), 'trimmed_start_frames': trimmed,
                                   'sim_start': float(times[0]),
                                   'sim_end': float(end), 'n_msgs': {k: len(v) for k, v in s.items()}})
        print(f'{ep_dir.name}: {len(times)} frames (trimmed {trimmed})', flush=True)
    Path(str(Path(args.out).expanduser()) + '_conversion.json').write_text(json.dumps(report, indent=1))
    print(f'saved {len(episodes)} episodes to {args.out}')


if __name__ == '__main__':
    main()

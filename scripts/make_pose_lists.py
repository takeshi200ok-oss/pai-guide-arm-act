#!/usr/bin/env python3
"""Stage B の初期姿勢一覧（学習・開発・最終評価）を生成して configs/start_poses.json に保存する。

- 各一覧を別々の seed で、範囲内の一様分布から生成する（モデルの成績を見て選別しない）
- 角度は小数第4位に丸め、丸め後に一覧間・一覧内の重複がないことを確認する
- 基準姿勢と、Stage A までに使った開発姿勢は除外する
- P/N で同じ一覧を使う。joint2・joint4 は 0 に固定する
"""
import argparse
import datetime
import hashlib
import json
import random
from pathlib import Path

from guide_common import CONFIG_DIR

RANGES = {'crane_plus_joint1': (-0.15, 0.15), 'crane_plus_joint3': (0.0, 0.10)}
FIXED = {'crane_plus_joint2': 0.0, 'crane_plus_joint4': 0.0}
DIGITS = 4
SEEDS = {'train': 101, 'dev': 202, 'eval': 303}
EXCLUDE = [  # 基準姿勢と、Stage A の参照姿勢測定で使った開発姿勢
    {'crane_plus_joint1': 0.0, 'crane_plus_joint3': 0.0},
    {'crane_plus_joint1': 0.15, 'crane_plus_joint3': 0.05},
    {'crane_plus_joint1': -0.15, 'crane_plus_joint3': 0.10},
]


def key(pose):
    return tuple(round(pose[n], DIGITS) for n in RANGES)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--count', type=int, default=20)
    ap.add_argument('--out', default=str(CONFIG_DIR / 'start_poses.json'))
    args = ap.parse_args()
    if Path(args.out).exists():
        raise SystemExit(f'{args.out} は既にある（固定済みの一覧は上書きしない）')

    used = {key(p) for p in EXCLUDE}
    lists = {}
    for name, seed in SEEDS.items():
        rng = random.Random(seed)
        poses = []
        while len(poses) < args.count:
            pose = {n: round(rng.uniform(lo, hi), DIGITS) for n, (lo, hi) in RANGES.items()}
            if key(pose) in used:
                continue  # 重複・除外姿勢は捨てて次を引く
            used.add(key(pose))
            poses.append({'id': f'{name}{len(poses):02d}', **pose, **FIXED})
        lists[name] = poses
    body = {'created': datetime.datetime.now().isoformat(timespec='seconds'),
            'method': 'random.Random(seed).uniform を関節ごとに順に呼び、小数第4位に丸める。'
                      '重複・除外姿勢は捨てて引き直す',
            'ranges': RANGES, 'fixed': FIXED, 'digits': DIGITS, 'seeds': SEEDS, 'excluded': EXCLUDE,
            'lists': lists}
    text = json.dumps(body, indent=1)
    Path(args.out).write_bytes(text.encode())  # OS によらず LF で保存（ハッシュを一致させる）
    for name, poses in lists.items():
        j1 = [p['crane_plus_joint1'] for p in poses]
        j3 = [p['crane_plus_joint3'] for p in poses]
        print(f'{name}: {len(poses)} poses, j1 [{min(j1):+.4f}, {max(j1):+.4f}], j3 [{min(j3):.4f}, {max(j3):.4f}]')
    print('sha256', hashlib.sha256(text.encode()).hexdigest())


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""変換後の LeRobotDataset を検査する（pai_training の pixi 環境で実行）。

- 列名が想定どおり（[joint1, joint2, joint4, joint3, gripper]）か
- エピソード長、最初と最後の観測・指令
- 指定関節について、実測(observation.state)が指令(action)から何フレーム遅れて追従しているか
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from guide_common import FEATURE_NAMES


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root')
    ap.add_argument('--joint', default='joint1', help='遅れを調べる列名')
    args = ap.parse_args()
    root = Path(args.root).expanduser()
    info = json.loads((root / 'meta' / 'info.json').read_text())
    for key in ('observation.state', 'observation.environment_state', 'action'):
        names = info['features'][key]['names']
        assert names == FEATURE_NAMES, f'{key} の列名が想定と違う: {names}'
    j = FEATURE_NAMES.index(args.joint)
    rows = []
    for f in sorted((root / 'data').rglob('episode_*.parquet')):
        df = pd.read_parquet(f)
        state = np.stack(df['observation.state'].to_numpy())
        action = np.stack(df['action'].to_numpy())
        errs = {k: float(np.mean(np.abs(state[k:, j] - action[:len(action) - k, j]))) for k in range(0, 21)}
        lag = min(errs, key=errs.get)
        rows.append({'episode': f.stem[-3:], 'frames': len(df),
                     'state0': np.round(state[0, :4], 3).tolist(), 'state_end': np.round(state[-1, :4], 3).tolist(),
                     'action_end': np.round(action[-1, :4], 3).tolist(), f'{args.joint}_lag_frames': lag})
    print(f'columns OK: {FEATURE_NAMES}  fps={info["fps"]}  episodes={info["total_episodes"]}  frames={info["total_frames"]}')
    print(pd.DataFrame(rows).to_string(index=False))


if __name__ == '__main__':
    main()

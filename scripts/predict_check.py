#!/usr/bin/env python3
"""学習済み ACT に、データセットの1エピソードの観測を順に与え、予測した指令と教師の指令を並べる
（pai_training の pixi 環境で実行）。シミュレータは使わない。"""
import argparse

import numpy as np
import pandas as pd
import torch
from lerobot.common.policies.act.modeling_act import ACTPolicy

from guide_common import FEATURE_NAMES


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--policy', required=True)
    ap.add_argument('--episode', required=True, help='parquet ファイル')
    ap.add_argument('--frames', default='0,4,8,12,16,20,25,30,40,60,90')
    args = ap.parse_args()
    p = ACTPolicy.from_pretrained(args.policy)
    p.eval()
    df = pd.read_parquet(args.episode)
    S = np.stack(df['observation.state'].to_numpy())
    A = np.stack(df['action'].to_numpy())
    j1, j3 = FEATURE_NAMES.index('joint1'), FEATURE_NAMES.index('joint3')
    n = p.config.n_action_steps
    print(f'frame  state(j1,j3)  teacher(j1,j3)  ACT 1st(j1,j3)  ACT {n}th(j1,j3)')
    for i in [int(f) for f in args.frames.split(',') if int(f) < len(S)]:
        p.reset()
        x = torch.tensor(S[i], dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            a = p.select_action({'observation.state': x, 'observation.environment_state': x}).numpy()[0]
        last = p._action_queue[-1].numpy()[0] if len(p._action_queue) else a  # チャンクで最後に実行する指令
        print(f'{i:5d}  {S[i, j1]:+.3f} {S[i, j3]:+.3f}   {A[i, j1]:+.3f} {A[i, j3]:+.3f}   '
              f'{a[j1]:+.3f} {a[j3]:+.3f}      {last[j1]:+.3f} {last[j3]:+.3f}')


if __name__ == '__main__':
    main()

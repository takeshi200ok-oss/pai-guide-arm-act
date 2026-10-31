#!/usr/bin/env python3
# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Copyright 2024 Tatsuya Kamijo. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# pai_training/scripts/train.py（講義教材, 上記ライセンス）を基に変更:
#   - 設定ファイルを引数で受け取り、seed を固定する
#   - 特徴量の定義はデータセットのメタデータから読む（列名の取り違えを防ぐ）
#   - wandb を使わず、loss を CSV に、使った設定とデータセット情報を重みと同じ場所に保存する
"""ACT を LeRobotDataset で学習する（pai_training の pixi 環境で実行）。

  pixi run --manifest-path ~/pai_training/pyproject.toml python train.py --config <yaml>
"""
import argparse
import json
import random
import shutil
import time
from pathlib import Path

import numpy as np
import torch
import yaml
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.common.datasets.utils import dataset_to_policy_features
from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.configs.types import FeatureType


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', required=True)
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())

    seed = cfg['seed']
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

    out = Path(cfg['output_directory']).expanduser()
    out.mkdir(parents=True, exist_ok=False)  # 既存の重みを上書きしない
    root = Path(cfg['dataset_root']).expanduser()
    meta = LeRobotDatasetMetadata(cfg['repo_id'], root=root)
    keys = ('observation.state', 'observation.environment_state', 'action')
    features = dataset_to_policy_features({k: meta.features[k] for k in keys})
    output_features = {k: ft for k, ft in features.items() if ft.type is FeatureType.ACTION}
    input_features = {k: ft for k, ft in features.items() if k not in output_features}

    policy_cfg = ACTConfig(input_features=input_features, output_features=output_features,
                           chunk_size=cfg['chunk_size'], n_action_steps=cfg['n_action_steps'])
    # ほぼ一定の関節（動かさない joint2/joint4、固定のグリッパ）は標準偏差が 0 に近く、
    # 動作に伴う微小なたわみ（1e-3 rad 程度）が正規化で大きな信号になってしまう。
    # 下限 min_std [rad] を設けて、こうした微小な揺れを手がかりにできないようにする。
    stats = {k: dict(v) for k, v in meta.stats.items()}
    min_std = cfg.get('min_std')
    if min_std:
        for k in keys:
            stats[k]['std'] = np.maximum(np.asarray(stats[k]['std']), min_std)
    policy = ACTPolicy(policy_cfg, dataset_stats=stats)
    policy.train()
    device = torch.device(cfg['device'])
    policy.to(device)

    dataset = LeRobotDataset(cfg['repo_id'], root=root, delta_timestamps=cfg['delta_timestamps'])
    optimizer = torch.optim.Adam(policy.parameters(), lr=cfg['learning_rate'])
    generator = torch.Generator().manual_seed(seed)
    loader = torch.utils.data.DataLoader(dataset, num_workers=cfg['num_workers'], batch_size=cfg['batch_size'],
                                         shuffle=True, pin_memory=False, drop_last=True, generator=generator)

    log = open(out / 'loss.csv', 'w')
    log.write('step,loss,l1_loss,kld_loss,wall_s\n')
    t0 = time.monotonic()
    step = 0
    while step < cfg['training_steps']:
        for batch in loader:
            batch = {k: (v.to(device) if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}
            # delta_timestamps で付いた時間方向の次元を落とす（講義の train.py と同じ）
            batch['observation.state'] = batch['observation.state'][:, -1, :]
            batch['observation.environment_state'] = batch['observation.environment_state'][:, -1, :]
            loss, info = policy.forward(batch)
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()
            log.write(f'{step},{loss.item():.5f},{info.get("l1_loss", float("nan")):.5f},'
                      f'{info.get("kld_loss", float("nan")):.5f},{time.monotonic() - t0:.1f}\n')
            if step % cfg['log_freq'] == 0:
                print(f'step: {step} loss: {loss.item():.3f}', flush=True)
            step += 1
            if step >= cfg['training_steps']:
                break
    log.close()
    policy.save_pretrained(out)
    shutil.copy(args.config, out / 'train_config.yaml')
    (out / 'dataset_info.json').write_text((root / 'meta' / 'info.json').read_text())
    (out / 'train_summary.json').write_text(json.dumps(
        {'steps': step, 'final_loss': loss.item(), 'wall_s': time.monotonic() - t0,
         'torch_threads': torch.get_num_threads(), 'torch': torch.__version__}, indent=1))
    print(f'policy saved to {out} ({time.monotonic() - t0:.0f}s)')


if __name__ == '__main__':
    main()

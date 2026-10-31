#!/usr/bin/env python3
"""教師の直接実行の記録から、各方向の参照姿勢を決めて configs/reference.json に保存する。

ACT の結果を見る前に実行し、以後は変更しない。
- 各試行の静止姿勢 = 記録の最後 settle_window 秒の平均
- 静止の確認   = 最後 settle_window 秒の各関節の変動幅（max - min）が still_tol 以下
- 参照姿勢     = 全試行の静止姿勢の平均（試行間のばらつきも保存）
- 指令目標・参照姿勢・両者の差をすべて保存し、教師の各試行を criteria.yaml で再判定する
"""
import argparse
import datetime
import json
from pathlib import Path

from evaluate import judge
from guide_common import ARM_JOINTS, CONFIG_DIR, load_poses, load_yaml


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs-dir', required=True, help='evaluate.py --controller teacher の出力')
    ap.add_argument('--settle-window', type=float, default=1.0)
    ap.add_argument('--still-tol', type=float, default=0.001)
    ap.add_argument('--out', default=str(CONFIG_DIR / 'reference.json'))
    args = ap.parse_args()

    poses = load_poses()
    crit = load_yaml('criteria.yaml')
    out = {'created': datetime.datetime.now().isoformat(timespec='seconds'),
           'joint_order': ARM_JOINTS, 'settle_window': args.settle_window, 'still_tol': args.still_tol,
           'criteria': crit, 'directions': {}}
    for d in ('P', 'N'):
        target = [poses['directions'][d][n] for n in ARM_JOINTS]
        runs = []
        for f in sorted(Path(args.runs_dir).glob(f'teacher_{d}_*.json')):
            r = json.loads(f.read_text())
            t0 = r['first_command']['header_stamp']
            traj = [(t, arm) for t, arm, _ in r['trajectory'] if t >= 0]
            t_end = traj[-1][0]
            tail = [arm for t, arm in traj if t >= t_end - args.settle_window]
            settled = [sum(a[i] for a in tail) / len(tail) for i in range(4)]
            spread = [max(a[i] for a in tail) - min(a[i] for a in tail) for i in range(4)]
            # 指令目標から 0.01 / 0.002 rad 以内に入った時刻（以後出ない最初の時刻）
            def entered(tol):
                t_in = None
                for t, arm in traj:
                    if max(abs(p - g) for p, g in zip(arm, target)) <= tol:
                        t_in = t if t_in is None else t_in
                    else:
                        t_in = None
                return t_in
            runs.append({'file': f.name, 'start': r['start'], 'record_end': t_end,
                         'settled': dict(zip(ARM_JOINTS, settled)),
                         'tail_spread': dict(zip(ARM_JOINTS, spread)),
                         'still': max(spread) <= args.still_tol,
                         'settled_minus_target': dict(zip(ARM_JOINTS, [s - g for s, g in zip(settled, target)])),
                         'time_within_0.01_of_target': entered(0.01),
                         'time_within_0.002_of_target': entered(0.002),
                         'rtf': r['rtf'], '_t0': t0, '_r': r})
        if not runs:
            raise SystemExit(f'{d} の教師試行が見つからない')
        ref = [sum(run['settled'][n] for run in runs) / len(runs) for n in ARM_JOINTS]
        between = {n: max(run['settled'][n] for run in runs) - min(run['settled'][n] for run in runs)
                   for n in ARM_JOINTS}
        for run in runs:
            r = run.pop('_r')
            t0 = run.pop('_t0')
            samples = [(t + t0, arm, g) for t, arm, g in r['trajectory']]
            ok, t_in, _ = judge(samples, t0, ref, crit)
            run['criteria_check'] = {'success': ok, 'reach_time': t_in}
        out['directions'][d] = {
            'target': dict(zip(ARM_JOINTS, target)),
            'reference': dict(zip(ARM_JOINTS, ref)),
            'reference_minus_target': dict(zip(ARM_JOINTS, [a - b for a, b in zip(ref, target)])),
            'between_run_spread': between,
            'all_still': all(run['still'] for run in runs),
            'runs': runs,
        }
    Path(args.out).write_text(json.dumps(out, indent=1))
    for d, v in out['directions'].items():
        print(d, 'reference-target =', {k[-6:]: round(x, 5) for k, x in v['reference_minus_target'].items()},
              'spread =', round(max(v['between_run_spread'].values()), 5), 'all_still =', v['all_still'])
        for run in v['runs']:
            print('  ', run['file'], 'still', run['still'], 'in0.01@', run['time_within_0.01_of_target'],
                  'in0.002@', run['time_within_0.002_of_target'], 'crit', run['criteria_check'],
                  'rtf', round(run['rtf'], 3))


if __name__ == '__main__':
    main()

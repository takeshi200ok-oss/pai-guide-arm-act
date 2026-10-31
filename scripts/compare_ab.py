#!/usr/bin/env python3
"""stage_b_eval.sh の結果を集計し、条件 A/B を比較する（Markdown と JSON を出力）。

- P/N・A/B ごとの成功数、成功試行の到達時間、timeout（8 sim 秒）時点の最大関節誤差（参照姿勢／指令目標）
- 要求した初期姿勢と、実際の開始姿勢（最初の指令の直前のサンプル）の差
- 同じ姿勢 ID での「両方成功」「Aだけ成功」「Bだけ成功」「両方失敗」の件数と、姿勢ごとの表
"""
import argparse
import json
import statistics
from pathlib import Path

ARM = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint4', 'crane_plus_joint3']


def load(out_dir):
    res = {}
    for f in sorted(Path(out_dir).glob('*_trial0.json')):
        c, d, pid = f.name[:-len('_trial0.json')].split('_', 2)
        r = json.loads(f.read_text())
        before = [arm for t, arm, _ in r.get('trajectory', []) if t < 0]
        actual = dict(zip(ARM, before[-1])) if before else None
        res[(c, d, pid)] = {
            'status': r['status'], 'reach_time': r.get('reach_time'),
            'err_ref': r['at_timeout']['vs_reference']['max_abs_err'] if r.get('at_timeout') else None,
            'err_target': r['at_timeout']['vs_target']['max_abs_err'] if r.get('at_timeout') else None,
            'start_requested': {n: r['start'][n] for n in ARM},
            'start_actual': actual,
            'start_diff': max(abs(actual[n] - r['start'][n]) for n in ARM) if actual else None,
            'rtf': r.get('rtf'), 'publishers': r.get('arm_publishers'),
        }
    return res


def fmt_range(vals, digits=2):
    return f'{statistics.mean(vals):.{digits}f}（{min(vals):.{digits}f}〜{max(vals):.{digits}f}）' if vals else '-'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out_dir')
    args = ap.parse_args()
    res = load(args.out_dir)
    ids = sorted({k[2] for k in res})
    lines = ['## 条件ごとの結果', '',
             '| 方向 | 条件 | 成功 | 成功試行の到達時間 [sim s] 平均（最小〜最大） | 8 sim 秒時点の最大誤差 [rad] 中央値／最大（参照姿勢） | 同（指令目標） |',
             '|---|---|---|---|---|---|']
    summary = {'conditions': {}, 'paired': {}}
    for d in ('P', 'N'):
        for c in ('A', 'B'):
            rs = [res[(c, d, i)] for i in ids if (c, d, i) in res]
            ok = [r for r in rs if r['status'] == 'success']
            er = [r['err_ref'] for r in rs if r['err_ref'] is not None]
            et = [r['err_target'] for r in rs if r['err_target'] is not None]
            rt = [r['reach_time'] for r in ok]
            lines.append(f'| {d} | {c} | {len(ok)}/{len(rs)} | {fmt_range(rt)} | '
                         f'{statistics.median(er):.4f}／{max(er):.4f} | {statistics.median(et):.4f}／{max(et):.4f} |')
            summary['conditions'][f'{d}_{c}'] = {'success': len(ok), 'n': len(rs), 'reach_times': rt,
                                                 'other_status': sorted({r['status'] for r in rs} - {'success', 'fail'})}
    lines += ['', '## 同じ初期姿勢での対応（A と B）', '', '| 方向 | 両方成功 | Aだけ成功 | Bだけ成功 | 両方失敗 |', '|---|---|---|---|---|']
    for d in ('P', 'N'):
        cnt = {'both': [], 'A_only': [], 'B_only': [], 'neither': []}
        for i in ids:
            a, b = res.get(('A', d, i)), res.get(('B', d, i))
            if not (a and b):
                continue
            sa, sb = a['status'] == 'success', b['status'] == 'success'
            cnt['both' if sa and sb else 'A_only' if sa else 'B_only' if sb else 'neither'].append(i)
        summary['paired'][d] = cnt
        lines.append(f'| {d} | {len(cnt["both"])} | {len(cnt["A_only"])} {cnt["A_only"] or ""} | '
                     f'{len(cnt["B_only"])} {cnt["B_only"] or ""} | {len(cnt["neither"])} |')
    diffs = [r['start_diff'] for r in res.values() if r['start_diff'] is not None]
    pubs = sorted({p for r in res.values() for p in (r['publishers'] or [])})
    rtfs = [r['rtf'] for r in res.values() if r['rtf']]
    lines += ['', f'- 試行数 {len(res)}。要求した初期姿勢と実際の開始姿勢の差: 最大 {max(diffs):.4f} rad',
              f'- 指令の送信元: {pubs}。RTF {min(rtfs):.2f}〜{max(rtfs):.2f}',
              '', '## 姿勢ごとの表', '',
              '| 姿勢ID | joint1 | joint3 | P-A | P-B | N-A | N-B |', '|---|---|---|---|---|---|---|']
    for i in ids:
        any_r = next(r for k, r in res.items() if k[2] == i)
        cells = []
        for d in ('P', 'N'):
            for c in ('A', 'B'):
                r = res.get((c, d, i))
                cells.append('-' if not r else (f'成功 {r["reach_time"]:.2f}s' if r['status'] == 'success'
                                                 else f'{r["status"]} 誤差{r["err_ref"]:.3f}' if r['err_ref'] is not None
                                                 else r['status']))
        s = any_r['start_requested']
        lines.append(f'| {i} | {s["crane_plus_joint1"]:+.4f} | {s["crane_plus_joint3"]:.4f} | ' + ' | '.join(cells) + ' |')
    text = '\n'.join(lines)
    Path(args.out_dir, 'comparison.md').write_text(text + '\n')
    Path(args.out_dir, 'comparison.json').write_text(json.dumps({'summary': summary, 'trials': {
        '_'.join(k): v for k, v in res.items()}}, indent=1))
    print(text)


if __name__ == '__main__':
    main()

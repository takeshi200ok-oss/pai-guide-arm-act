#!/usr/bin/env python3
"""提出用動画（1280x720, 30fps, 約2分30秒）を作る（pai_training の pixi 環境で実行。Pillow・matplotlib・ffmpeg を使う）。

  pixi run --manifest-path ~/pai_training/pyproject.toml python make_video.py --runs ~/guide_runs --out <dir> \
      [--cover <表紙画像>] [--bgm <音楽ファイル>]

- 字幕・見出しの文章は captions.yaml、数値は評価結果の JSON（eval_A, eval_B_final, 録画試行）から読む
- 実演は Gazebo GUI 内蔵の録画（scripts/record_trial.sh）を、動き出しの少し前から切り出して等倍で使う
- 表紙画像（講座の運営が提供したもの）と BGM は本リポジトリに含めない。指定しなければ表紙なし・無音で作る
"""
import argparse
import json
import re
import statistics
import subprocess
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
W, H, FPS = 1280, 720, 30
FONT_R = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FONT_B = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'  # index 0 = Noto Sans CJK JP
INK, SUB, BG, BAND = '#1F2933', '#52606D', '#FFFFFF', '#F0F4F8'
COLOR = {'P': '#D9730D', 'N': '#1F5FBF', 'A': '#3E4C59', 'B': '#2F9E77'}
NAME = {'P': '右（P）', 'N': '左（N）'}
plt.rcParams['font.family'] = 'Noto Sans CJK JP'


def font(size, bold=False):
    return ImageFont.truetype(FONT_B if bold else FONT_R, size, index=0)


NO_LINE_START = '）)」』、。，．・ー？！'  # 行頭に置かない文字（前の行の末尾に付ける）


def tokens(text):
    """折り返しの単位に分ける。英数字・記号の連続（例: 0.63, v7, joint1）は途中で切らない。"""
    out = []
    for m in re.finditer(r'[A-Za-z0-9.+\-−_/%×]+|.', text):
        out.append(m.group())
    return out


def wrap(draw, text, f, width):
    """日本語を含む文字列を、描画幅 width に収まるよう折り返す（簡単な行頭禁則つき）。"""
    lines, cur = [], ''
    for tok in tokens(text):
        if draw.textlength(cur + tok, font=f) > width and cur and tok[0] not in NO_LINE_START:
            lines.append(cur)
            cur = '' if tok == ' ' else tok
        else:
            cur += tok
    return lines + ([cur] if cur else [])


def fit_font(draw, text, width, size, bold=False):
    """text が width に収まる最大の文字サイズ（size 以下）のフォント。"""
    while size > 10 and draw.textlength(text, font=font(size, bold)) > width:
        size -= 1
    return font(size, bold)


def text_block(draw, xy, text, f, width, fill=INK, gap=8):
    x, y = xy
    for line in wrap(draw, text, f, width):
        draw.text((x, y), line, font=f, fill=fill)
        y += f.size + gap
    return y


def slide_base(title, kicker=None):
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, W, 86], fill=BAND)
    d.text((48, 22), title, font=font(34, True), fill=INK)
    if kicker:  # 帯の右下に、どの部分の説明かを小さく示す
        f = font(16)
        d.text((W - 24 - d.textlength(kicker, font=f), 62), kicker, font=f, fill=SUB)
    return img, d


def paste_figure(img, path, box):
    fig = Image.open(path).convert('RGB')
    x0, y0, x1, y1 = box
    fig.thumbnail((x1 - x0, y1 - y0))
    img.paste(fig, (x0 + (x1 - x0 - fig.width) // 2, y0 + (y1 - y0 - fig.height) // 2))


# ---------- 図 ----------
def load_json(p):
    return json.loads(Path(p).read_text())


def fig_fail_traj(runs, out):
    """Stage A の開発中の版と最終版（v7）の joint1 の軌跡（基準姿勢から、GUIなしの評価 JSON）。"""
    cases = {'P': [('act_P_A_s2000_home', '第1版（停止）', ':'), ('act_P_A_s2000_minstd_trim_home', '第3版（逆向き）', '--'),
                   ('act_P_A_chunk_s2000_home', '第7版（最終版）', '-')],
             'N': [('act_N_A_s2000_minstd_home', '第2版（逆向き）', '--'), ('act_N_A_lin2_s2000_home', '第6版（停止）', ':'),
                   ('act_N_A_chunk_s2000_home', '第7版（最終版）', '-')]}
    fig, axes = plt.subplots(2, 1, figsize=(6.0, 5.6), dpi=110, sharex=True)
    for ax, d in zip(axes, ('P', 'N')):
        target = 0.8 if d == 'P' else -0.8
        ax.axhline(target, color=COLOR[d], lw=1, alpha=0.5)
        ax.text(7.9, target, '目標', ha='right', va='bottom' if d == 'P' else 'top', color=COLOR[d], fontsize=9)
        for tag, label, ls in cases[d]:
            r = load_json(Path(runs, 'eval_A', f'{tag}_trial0.json'))
            t = [p[0] for p in r['trajectory'] if 0 <= p[0] <= 8]
            j1 = [p[1][0] for p in r['trajectory'] if 0 <= p[0] <= 8]
            ax.plot(t, j1, ls, color=COLOR[d] if '最終版' in label else '#7B8794', lw=2 if '最終版' in label else 1.6, label=label)
        ax.set_title(f'{NAME[d]}方向（基準姿勢から）', fontsize=11)
        ax.set_xlim(0, 8)
        ax.set_ylim(-0.95, 0.95)
        ax.set_ylabel('joint1 [rad]')
        ax.legend(fontsize=9, loc='center right')
        ax.grid(alpha=0.3)
    axes[1].set_xlabel('最初の指令からの時間 [sim秒]')
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def fig_poses(out):
    poses = load_json(ROOT / 'configs' / 'start_poses.json')['lists']
    fig, ax = plt.subplots(figsize=(5.4, 4.2), dpi=110)
    style = {'train': ('B の学習用（20）', COLOR['B'], 'o'), 'dev': ('開発確認用（20、うち5を使用）', '#9AA5B1', 'x'),
             'eval': ('最終評価用（20）', COLOR['P'], 's')}
    for k, (label, c, m) in style.items():
        ax.scatter([p['crane_plus_joint1'] for p in poses[k]], [p['crane_plus_joint3'] for p in poses[k]],
                   label=label, color=c, marker=m, s=36, facecolors='none' if m == 's' else None)
    ax.scatter([0], [0], marker='*', s=220, color=COLOR['A'], label='A の学習（基準姿勢のみ）', zorder=5)
    ax.set_xlabel('初期の joint1 [rad]')
    ax.set_ylabel('初期の joint3 [rad]')
    ax.set_xlim(-0.17, 0.17)
    ax.set_ylim(-0.01, 0.11)
    ax.legend(fontsize=8, loc='upper center', bbox_to_anchor=(0.5, -0.17), ncol=2)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def comparison(runs):
    trials = load_json(Path(runs, 'eval_B_final', 'comparison.json'))['trials']
    rows, diff, paired = {}, {}, {'a_only': 0, 'b_only': 0}
    for d in ('P', 'N'):
        ids = sorted({k.split('_', 2)[2] for k in trials if k.startswith(f'A_{d}_')})
        for c in ('A', 'B'):
            rs = [trials[f'{c}_{d}_{i}'] for i in ids]
            ok = [r for r in rs if r['status'] == 'success']
            rows[(d, c)] = (len(ok), len(rs), statistics.mean(r['reach_time'] for r in ok),
                            max(r['err_ref'] for r in rs))
        both = [i for i in ids if trials[f'A_{d}_{i}']['status'] == trials[f'B_{d}_{i}']['status'] == 'success']
        diff[d] = statistics.mean(trials[f'B_{d}_{i}']['reach_time'] - trials[f'A_{d}_{i}']['reach_time'] for i in both)
        for i in ids:
            a, b = (trials[f'{c}_{d}_{i}']['status'] == 'success' for c in 'AB')
            paired['a_only'] += a and not b
            paired['b_only'] += b and not a
    return trials, rows, diff, paired


def fig_reach(trials, out):
    fig, axes = plt.subplots(1, 2, figsize=(6.2, 3.9), dpi=110, sharey=True)
    for ax, d in zip(axes, ('P', 'N')):
        ids = sorted({k.split('_', 2)[2] for k in trials if k.startswith(f'A_{d}_')})
        x = np.arange(len(ids))
        for c, off in (('A', -0.15), ('B', 0.15)):
            ax.scatter(x + off, [trials[f'{c}_{d}_{i}']['reach_time'] for i in ids], s=16, color=COLOR[c], label=f'条件{c}')
        ax.set_title(f'{NAME[d]}：評価姿勢ごとの到達時間', fontsize=10)
        ax.set_xlabel('評価姿勢（eval00〜19）')
        ax.set_xticks([])
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('到達時間 [sim秒]')
    axes[0].legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


# ---------- スライド ----------
def make_cover(c, cover, out):
    """講座の表紙画像の左下の余白（「FINAL PROJECT PRESENTATION」の下）に作品名と副題を重ねる。"""
    img = Image.open(cover).convert('RGB').resize((W, H), Image.LANCZOS)
    d = ImageDraw.Draw(img)
    x, y, width = 60, 452, 520  # 右側の斜めの帯にかからない幅
    d.text((x, y), c['title'], font=fit_font(d, c['title'], width, 32, True), fill=INK)
    y += 52
    for line in c['subtitle']:
        d.text((x, y), line, font=font(21), fill=SUB)
        y += 32
    img.save(out)


def make_pipeline(c, out):
    img, d = slide_base(c['title'])
    n = len(c['boxes'])
    bw, bh, gap, y0 = 172, 110, 32, 150
    x0 = (W - (n * bw + (n - 1) * gap)) // 2
    for i, label in enumerate(c['boxes']):
        x = x0 + i * (bw + gap)
        fill = '#FFF4E5' if 'ACT' in label or '推論' in label else BAND
        d.rounded_rectangle([x, y0, x + bw, y0 + bh], 14, fill=fill, outline='#9AA5B1', width=2)
        for k, line in enumerate(label.split('\n')):
            f = fit_font(d, line, bw - 14, 21 if k == 0 else 17, k == 0)
            d.text((x + (bw - d.textlength(line, font=f)) / 2, y0 + 24 + k * 38), line, font=f, fill=INK if k == 0 else SUB)
        if i < n - 1:
            ax = x + bw + 4
            d.line([ax, y0 + bh / 2, ax + gap - 8, y0 + bh / 2], fill=INK, width=3)
            d.polygon([(ax + gap - 8, y0 + bh / 2 - 7), (ax + gap, y0 + bh / 2), (ax + gap - 8, y0 + bh / 2 + 7)], fill=INK)
    # 推論ノード → Gazebo へ戻る矢印
    xs, xe = x0 + (n - 1) * (bw + gap) + bw / 2, x0 + 1 * (bw + gap) + bw / 2
    yb = y0 + bh + 34
    d.line([xs, y0 + bh, xs, yb, xe, yb, xe, y0 + bh + 8], fill=COLOR['P'], width=3)
    d.polygon([(xe - 7, y0 + bh + 10), (xe, y0 + bh), (xe + 7, y0 + bh + 10)], fill=COLOR['P'])
    d.text(((xs + xe) / 2 - 150, yb + 6), '学習後は ACT の指令で動かす（教師は停止）', font=font(18), fill=COLOR['P'])
    y = 380
    for note in c['notes']:
        d.ellipse([70, y + 12, 80, y + 22], fill=INK)
        y = text_block(d, (96, y), note, font(25), W - 170) + 14
    img.save(out)


def make_items(c, out, fig=None):
    img, d = slide_base(c['title'], c.get('kicker'))
    width = 540 if fig else W - 140
    y = 120
    for head, body in c['items']:
        d.text((60, y), head, font=font(22, True), fill=COLOR['P'] if head == '対策' else COLOR['B'] if head == '結果' else SUB)
        y = text_block(d, (60, y + 32), body, font(24), width) + 20
    if fig:
        paste_figure(img, fig, (620, 96, W - 10, H - 6))
    img.save(out)


def make_compare_setup(c, out, fig):
    img, d = slide_base(c['title'])
    y = 130
    for line in c['lines']:
        d.ellipse([60, y + 12, 70, y + 22], fill=INK)
        y = text_block(d, (84, y), line, font(24), 600) + 18
    paste_figure(img, fig, (720, 100, W - 20, H - 20))
    img.save(out)


def make_compare_result(c, out, fig, rows, diff, paired):
    img, d = slide_base(c['title'])
    cols = [('方向', 110), ('条件', 90), ('成功', 120), ('平均到達時間 [sim秒]', 260), ('8秒時点の誤差の最大 [rad]', 300)]
    x0, y0, rh = 60, 112, 40
    x = x0
    for name, w in cols:
        d.text((x + 8, y0), name, font=font(20, True), fill=SUB)
        x += w
    for k, (dd, cc) in enumerate([('P', 'A'), ('P', 'B'), ('N', 'A'), ('N', 'B')]):
        ok, n, rt, err = rows[(dd, cc)]
        y = y0 + rh * (k + 1)
        if k % 2 == 0:
            d.rectangle([x0, y - 4, x0 + sum(w for _, w in cols), y + rh - 6], fill=BAND)
        vals = [NAME[dd], cc, f'{ok}/{n}', f'{rt:.2f}', f'{err:.4f}']
        x = x0
        for (name, w), v in zip(cols, vals):
            d.text((x + 8, y), v, font=font(22, name == '成功'), fill=COLOR[dd] if name == '方向' else INK)
            x += w
    y = y0 + rh * 5 + 14
    for note in c['notes']:
        txt = note.format(a_only=paired['a_only'], b_only=paired['b_only'], dP=diff['P'], dN=diff['N'])
        y = text_block(d, (60, y), '・' + txt, font(20), 560) + 8
    paste_figure(img, fig, (620, 300, W - 6, H - 4))
    img.save(out)


def make_bullets(c, out, footer=None):
    img, d = slide_base(c['title'])
    y = 124
    for b in c['bullets']:
        d.ellipse([60, y + 13, 72, y + 25], fill=INK)
        y = text_block(d, (90, y), b, font(27), W - 160) + 20
    if footer:
        d.text((60, H - 44), footer, font=font(17), fill=SUB)
    img.save(out)


def make_sections(c, out, footer=None):
    """見出しつきのまとまりを縦に並べる（結論用）。行頭の ! は強調、small は小さめの注記。"""
    img, d = slide_base(c['title'])
    y = 118
    for sec in c['sections']:
        small = sec.get('small', False)
        d.text((60, y), sec['head'], font=font(20 if small else 23, True), fill=SUB if small else COLOR['B'])
        y += 34 if small else 40
        for line in sec['lines']:
            strong = line.startswith('!')
            f = font(21 if small else (31 if strong else 27), strong)
            y = text_block(d, (84, y), line.lstrip('!'), f, W - 160, fill=COLOR['P'] if strong else INK) + 12
        y += 26
    if footer:
        d.text((60, H - 44), footer, font=font(17), fill=SUB)
    img.save(out)


# 実演映像の切り出し範囲（Gazebo 内蔵録画 582x798 のうち、ロボット全体と左右の板が入る範囲）と表示位置
DEMO_CROP = (390, 236, 100, 306)  # 幅, 高さ, x, y
DEMO_BOX = (20, 104, 760)         # 表示位置 x, y と表示幅（高さは縦横比を保って決まる）


def make_demo_panel(c, out, result):
    """実演区間の背景（上に見出し、左に映像を重ねる余白、右に説明）。"""
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    col = COLOR[c['direction']]
    d.rectangle([0, 0, W, 86], fill=BAND)
    d.rectangle([0, 0, 14, 86], fill=col)
    d.text((40, 22), c['header'], font=font(32, True), fill=col)
    x0 = DEMO_BOX[0] + DEMO_BOX[2] + 24
    y = 112
    for line in c['lines']:
        txt = line.format(reach=result['reach_time'], rtf=result['rtf'])
        d.ellipse([x0, y + 11, x0 + 10, y + 21], fill=INK)
        y = text_block(d, (x0 + 22, y), txt, font(21), W - x0 - 40) + 14
    vh = round(DEMO_BOX[2] * DEMO_CROP[1] / DEMO_CROP[0])
    d.text((DEMO_BOX[0], DEMO_BOX[1] + vh + 14), c['caption'], font=font(18), fill=SUB)
    img.save(out)


# ---------- 動画 ----------
def ffmpeg(*args):
    subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', *args], check=True)


def still_segment(png, dur, out):
    fade = f'fade=in:st=0:d=0.3,fade=out:st={dur - 0.3}:d=0.3'
    ffmpeg('-loop', '1', '-t', str(dur), '-i', str(png), '-vf', f'fps={FPS},format=yuv420p,{fade}',
           '-c:v', 'libx264', '-preset', 'medium', '-crf', '20', str(out))


def add_bgm(video, bgm, dur, gain_db, out):
    """BGM を重ねる。曲の前後の無音を除き、曲の終わりが動画の終わり（1秒前）に来るようテンポを合わせる。
    合わせるのに 0.9 倍より遅くなる場合は、テンポを変えずに繰り返す。最後は 3 秒でフェードアウトする。"""
    sil = subprocess.run(['ffmpeg', '-hide_banner', '-i', str(bgm), '-af', 'silencedetect=n=-50dB:d=0.5', '-f', 'null',
                          '-'], capture_output=True, text=True).stderr
    total = float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0',
                                  str(bgm)], capture_output=True, text=True).stdout)
    starts = [float(v) for v in re.findall(r'silence_start: ([\d.]+)', sil)]
    ends = [float(v) for v in re.findall(r'silence_end: ([\d.]+)', sil)]
    head = ends[0] if starts and starts[0] < 0.05 and ends else 0.0
    tail = total  # 末尾の無音（ファイルの終わりまで続くもの）があれば、その手前まで
    if starts and starts[-1] > head and (len(ends) < len(starts) or ends[-1] >= total - 0.05):
        tail = starts[-1]
    tempo = (tail - head) / (dur - 1.0)
    if 0.9 <= tempo <= 1.0:
        af = f'atrim={head:.3f}:{tail:.3f},asetpts=PTS-STARTPTS,atempo={tempo:.4f}'
    else:
        tempo = 1.0
        af = f'atrim={head:.3f}:{tail:.3f},asetpts=PTS-STARTPTS,aloop=loop=-1:size=2e9'
    af += (f',atrim=0:{dur:.3f},volume={gain_db}dB,afade=in:st=0:d=1.5,afade=out:st={dur - 3:.3f}:d=3,'
           f'aformat=sample_rates=48000:channel_layouts=stereo')
    ffmpeg('-i', str(video), '-i', str(bgm), '-filter_complex', f'[1:a]{af}[a]', '-map', '0:v', '-map', '[a]',
           '-c:v', 'copy', '-c:a', 'aac', '-b:a', '160k', '-t', f'{dur:.3f}', '-movflags', '+faststart', str(out))
    return {'tempo': round(tempo, 4), 'trim': [round(head, 3), round(tail, 3)], 'gain_db': gain_db}


def motion_start(clip):
    """録画の中でロボットが動き出す時刻（フレーム差分が閾値を超える最初の時刻、冒頭1秒は除く）。"""
    w, h = 146, 200
    raw = subprocess.run(['ffmpeg', '-loglevel', 'error', '-i', str(clip), '-vf', f'scale={w}:{h},format=gray',
                          '-f', 'rawvideo', '-'], capture_output=True, check=True).stdout
    fr = np.frombuffer(raw, np.uint8).reshape(-1, h, w).astype(np.int16)
    rate = json.loads(subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v', '-show_entries',
                                      'stream=r_frame_rate', '-of', 'json', str(clip)], capture_output=True,
                                     text=True).stdout)['streams'][0]['r_frame_rate']
    fps = eval(rate)
    diff = np.abs(np.diff(fr, axis=0)).mean(axis=(1, 2))
    idx = [i for i in np.flatnonzero(diff > 0.05) if i / fps > 1.0]
    return idx[0] / fps


def demo_segment(c, panel, clip, dur, out):
    start = max(motion_start(clip) - c['lead'], 0)
    label = c['label']
    cw, ch, cx, cy = DEMO_CROP
    bx, by, bw = DEMO_BOX
    lx = bx + 16 if c['direction'] == 'N' else bx + bw - 150
    vf = (f'[0:v]crop={cw}:{ch}:{cx}:{cy},scale={bw}:-2,setsar=1[v];[1:v][v]overlay={bx}:{by},'
          f"drawtext=fontfile={FONT_B}:text='{label}':x={lx}:y={by + 14}:fontsize=40:fontcolor={COLOR[c['direction']]}:"
          f'box=1:boxcolor=white@0.8:boxborderw=8,'
          f'fps={FPS},format=yuv420p,fade=in:st=0:d=0.3,fade=out:st={dur - 0.3}:d=0.3')
    ffmpeg('-ss', f'{start:.2f}', '-t', str(dur), '-i', str(clip), '-loop', '1', '-t', str(dur), '-i', str(panel),
           '-filter_complex', vf, '-c:v', 'libx264', '-preset', 'medium', '-crf', '20', '-an', str(out))
    return start


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', required=True, help='guide_runs（eval_A, eval_B_final, videos を含む）')
    ap.add_argument('--out', required=True)
    ap.add_argument('--cover', help='冒頭に置く表紙画像（講座の運営が提供したもの。本リポジトリには含めない）')
    ap.add_argument('--bgm', help='BGM の音楽ファイル（本リポジトリには含めない。captions.yaml の bgm_credit を結論に表示）')
    ap.add_argument('--bgm-gain-db', type=float, default=-8.0)
    args = ap.parse_args()
    cap = yaml.safe_load((HERE / 'captions.yaml').read_text())
    out = Path(args.out).expanduser()
    (out / 'parts').mkdir(parents=True, exist_ok=True)
    P = out / 'parts'
    runs = Path(args.runs).expanduser()

    fig_fail_traj(runs, P / 'fig_fail_traj.png')
    fig_poses(P / 'fig_poses.png')
    trials, rows, diff, paired = comparison(runs)
    fig_reach(trials, P / 'fig_reach.png')

    segs, log = [], []

    def add_still(name, dur):
        seg = P / f'{len(segs):02d}_{name}.mp4'
        still_segment(P / f'{name}.png', dur, seg)
        segs.append(seg)
        log.append({'segment': seg.name, 'duration': dur})

    if args.cover:
        make_cover(cap['cover'], Path(args.cover).expanduser(), P / 'cover.png')
        add_still('cover', cap['cover']['duration'])
    make_bullets(cap['intro'], P / 'intro.png')
    add_still('intro', cap['intro']['duration'])
    for c in cap['demos']:
        result = load_json(runs / 'videos' / c['result'])
        make_demo_panel(c, P / f'demo_{c["direction"]}.png', result)
        seg = P / f'{len(segs):02d}_demo_{c["direction"]}.mp4'
        start = demo_segment(c, P / f'demo_{c["direction"]}.png', runs / 'videos' / c['clip'], c['duration'], seg)
        segs.append(seg)
        log.append({'segment': seg.name, 'duration': c['duration'], 'clip': c['clip'], 'clip_start_s': round(start, 2),
                    'trial_status': result['status'], 'reach_time': result['reach_time'], 'rtf': result['rtf']})
    make_pipeline(cap['pipeline'], P / 'pipeline.png')
    add_still('pipeline', cap['pipeline']['duration'])
    for k, c in enumerate(cap['failures']):
        make_items(c, P / f'fail{k}.png', P / 'fig_fail_traj.png' if c.get('figure') else None)
        add_still(f'fail{k}', c['duration'])
    make_compare_setup(cap['comparison'][0], P / 'cmp0.png', P / 'fig_poses.png')
    add_still('cmp0', cap['comparison'][0]['duration'])
    make_compare_result(cap['comparison'][1], P / 'cmp1.png', P / 'fig_reach.png', rows, diff, paired)
    add_still('cmp1', cap['comparison'][1]['duration'])
    make_sections(cap['conclusion'], P / 'conclusion.png', cap['bgm_credit'] if args.bgm else None)
    add_still('conclusion', cap['conclusion']['duration'])

    lst = P / 'concat.txt'
    lst.write_text(''.join(f"file '{s.name}'\n" for s in segs))
    final = out / 'guide_arm_final.mp4'
    silent = P / 'silent.mp4' if args.bgm else final
    ffmpeg('-f', 'concat', '-safe', '0', '-i', str(lst), '-c', 'copy', '-movflags', '+faststart', str(silent))
    dur = float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0',
                                str(silent)], capture_output=True, text=True).stdout)
    bgm = add_bgm(silent, Path(args.bgm).expanduser(), dur, args.bgm_gain_db, final) if args.bgm else None
    (out / 'video_log.json').write_text(json.dumps({'segments': log, 'total_s': dur, 'cover': bool(args.cover),
                                                    'bgm': bgm,
                                                    'comparison_rows': {f'{k[0]}_{k[1]}': v for k, v in rows.items()},
                                                    'paired': paired, 'reach_diff_B_minus_A': diff}, indent=1,
                                                   ensure_ascii=False))
    print(f'{final} ({dur:.1f}s)')


if __name__ == '__main__':
    main()

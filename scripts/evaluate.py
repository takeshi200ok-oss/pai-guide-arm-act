#!/usr/bin/env python3
"""案内動作を実行して記録し、成功判定する（教師スクリプトと ACT を同じ手順・同じ判定で評価する）。

1試行の流れ:
  1. teacher.py reset で開始姿勢へ戻して静止させる（この間は記録しない）
  2. 制御器を起動する（--controller teacher: teacher.py demo / policy: infer_node.py）
  3. 腕の指令トピックで **最初の指令メッセージを受信した時刻** を t0 とする（シミュレーション時刻）
  4. t0 から criteria.yaml の record 秒（シミュレーション秒）だけ /joint_states を記録し、制御器を止める
  5. 参照姿勢（reference.json）と指令目標の両方に対する誤差を計算し、成功判定する
"""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from guide_common import (ARM_JOINTS, ARM_TOPIC, GRIPPER_JOINT, JOINT_STATES_TOPIC, ROOT, format_pose,
                          load_poses, load_yaml, parse_pose, positions_by_name, sim_now,
                          spin_until_sim, wait_for_clock)

SCRIPTS = ROOT / 'scripts'
PAI_TRAINING = Path(os.environ.get('PAI_TRAINING', Path.home() / 'pai_training'))


class Recorder(Node):
    def __init__(self):
        super().__init__('guide_eval_recorder',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.recording = False
        self.samples = []         # (stamp, arm positions in ARM_JOINTS order, gripper)
        self.first_cmd = None     # (header stamp, receive sim time)
        self.n_cmd = 0
        self.min_stamp = float('inf')  # これより前の時刻の指令（リセット時の残り）は数えない
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_js, 50)
        self.create_subscription(JointTrajectory, ARM_TOPIC, self._on_cmd, 50)

    def _on_js(self, msg):
        if self.recording:
            pos = positions_by_name(msg, ARM_JOINTS + [GRIPPER_JOINT])
            self.samples.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, pos[:4], pos[4]))

    def _on_cmd(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        recv = sim_now(self)
        if (stamp if stamp > 0 else recv) < self.min_stamp:
            return
        self.n_cmd += 1
        if self.first_cmd is None:
            self.first_cmd = (stamp, recv)

    def arm_publishers(self):
        return sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                      for i in self.get_publishers_info_by_topic(ARM_TOPIC) if i.node_name != '_NODE_NAME_UNKNOWN_')


def judge(samples, t0, ref, crit):
    """samples: [(t, arm)]。腕4関節すべてが ref から tol 以内を hold 秒保った最初の区間を探す。"""
    tol, hold, timeout = crit['tol'], crit['hold'], crit['timeout']
    win = [(t - t0, max(abs(p - r) for p, r in zip(arm, ref))) for t, arm, _ in samples if t >= t0]
    t_in = None
    for t, err in win:
        if t > timeout:
            break
        if err <= tol:
            t_in = t if t_in is None else t_in
            if t - t_in >= hold:
                return True, t_in, t
        else:
            t_in = None
    return False, None, None


def errors_at(samples, t0, t_rel, target):
    """t0 + t_rel 以前で最後のサンプルの、関節ごとの誤差（ARM_JOINTS の順）。"""
    cand = [s for s in samples if s[0] - t0 <= t_rel]
    if not cand:
        return None
    _, arm, grip = cand[-1]
    return {'pos': dict(zip(ARM_JOINTS, arm)), 'gripper': grip,
            'err': {n: p - r for n, p, r in zip(ARM_JOINTS, arm, target)},
            'max_abs_err': max(abs(p - r) for p, r in zip(arm, target))}


def run_trial(args, rec, crit, start, target, ref):
    subprocess.run(['python3', str(SCRIPTS / 'teacher.py'), 'reset', '--start', format_pose(start)],
                   check=True, stdout=subprocess.DEVNULL)
    # リセット中に溜まった受信を捨ててから、起動時刻より前の指令を無視する設定にする
    rec.min_stamp = float('inf')
    spin_until_sim(rec, sim_now(rec) + 0.2)
    rec.samples.clear()
    rec.first_cmd = None
    rec.n_cmd = 0
    rec.min_stamp = sim_now(rec)
    log = open(Path(args.out_dir) / f'{args.tag}_trial{args.k}.controller.log', 'w')
    if args.controller == 'teacher':
        cmd = ['python3', str(SCRIPTS / 'teacher.py'), 'demo', '--direction', args.direction,
               '--start', format_pose(start), '--post-hold', str(args.teacher_post_hold),
               '--profile', args.teacher_profile,
               '--wait-subscribers', '1']
        cwd = SCRIPTS
    else:
        cmd = ['pixi', 'run', '--manifest-path', str(PAI_TRAINING / 'pyproject.toml'),
               'python', str(SCRIPTS / 'infer_node.py'), '--model-path', args.policy]
        cwd = SCRIPTS
    rec.recording = True
    t_launch = time.monotonic()
    proc = subprocess.Popen(cmd, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    result = {'controller': args.controller, 'policy': args.policy, 'direction': args.direction,
              'start': start, 'target': dict(zip(ARM_JOINTS, target)),
              'reference': dict(zip(ARM_JOINTS, ref)) if ref else None, 'criteria': crit}
    while rec.first_cmd is None and time.monotonic() - t_launch < args.start_timeout:
        rclpy.spin_once(rec, timeout_sec=0.05)
    if rec.first_cmd is None:
        result['status'] = 'start_timeout'  # 動作の失敗とは区別する
    else:
        stamp, recv = rec.first_cmd
        t0 = stamp if stamp > 0 else recv
        result['first_command'] = {'header_stamp': stamp, 'received_sim': recv,
                                   'wall_since_launch': time.monotonic() - t_launch}
        w0, s0 = time.monotonic(), sim_now(rec)
        pubs = set(rec.arm_publishers())
        while sim_now(rec) < t0 + crit['record']:
            spin_until_sim(rec, min(sim_now(rec) + 0.5, t0 + crit['record']))
            pubs.update(rec.arm_publishers())
        result['rtf'] = (sim_now(rec) - s0) / (time.monotonic() - w0)
        result['arm_publishers'] = sorted(pubs)
        result['n_commands'] = rec.n_cmd
        ok, t_in, t_done = judge(rec.samples, t0, ref or target, crit)
        result['status'] = 'success' if ok else 'fail'
        result['reach_time'] = t_in           # 許容範囲に入った時刻（成功時のみ）[sim s]
        result['success_confirmed_at'] = t_done
        result['at_timeout'] = {'vs_reference': errors_at(rec.samples, t0, crit['timeout'], ref) if ref else None,
                                'vs_target': errors_at(rec.samples, t0, crit['timeout'], target)}
        result['at_end'] = {'vs_reference': errors_at(rec.samples, t0, crit['record'], ref) if ref else None,
                            'vs_target': errors_at(rec.samples, t0, crit['record'], target)}
        result['trajectory'] = [[round(t - t0, 3), [round(p, 5) for p in arm], round(g, 5)]
                                for t, arm, g in rec.samples[::4]]
    rec.recording = False
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=15)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        os.killpg(proc.pid, signal.SIGKILL)
    log.close()
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--controller', choices=['teacher', 'policy'], required=True)
    ap.add_argument('--policy', help='ACT のチェックポイント（--controller policy）')
    ap.add_argument('--direction', choices=['P', 'N'], required=True)
    ap.add_argument('--starts', default='', help="開始姿勢を ';' 区切りで並べる（例: 'j1=0;j1=0.1,j3=0.05'）。既定は home")
    ap.add_argument('--trials', type=int, default=1, help='各開始姿勢での試行回数')
    ap.add_argument('--reference', help='参照姿勢 JSON（make_reference.py の出力）。無ければ指令目標で判定')
    ap.add_argument('--criteria', default='criteria.yaml')
    ap.add_argument('--record', type=float, help='記録の長さ [sim s] を上書きする（参照姿勢の測定用）')
    ap.add_argument('--teacher-post-hold', type=float, default=10.0)
    ap.add_argument('--teacher-profile', choices=['linear', 'min_jerk'], default='linear')
    ap.add_argument('--start-timeout', type=float, default=120.0, help='最初の指令を待つ実時間 [s]')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--tag', required=True)
    args = ap.parse_args()

    crit = load_yaml(args.criteria)
    if args.record:
        crit['record'] = args.record
    poses = load_poses()
    target = [poses['directions'][args.direction][n] for n in ARM_JOINTS]
    ref = None
    if args.reference:
        ref_all = json.loads(Path(args.reference).read_text())
        ref = [ref_all['directions'][args.direction]['reference'][n] for n in ARM_JOINTS]
    starts = [parse_pose(s) for s in args.starts.split(';')] if args.starts else [dict(poses['home'])]
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    rclpy.init()
    rec = Recorder()
    wait_for_clock(rec)
    results = []
    args.k = 0
    for start in starts:
        for _ in range(args.trials):
            r = run_trial(args, rec, crit, start, target, ref)
            r['trial'] = args.k
            (Path(args.out_dir) / f'{args.tag}_trial{args.k}.json').write_text(json.dumps(r, indent=1))
            summary = {k: r.get(k) for k in ('trial', 'status', 'reach_time', 'rtf', 'arm_publishers')}
            if r.get('at_timeout'):
                summary['err_ref'] = r['at_timeout']['vs_reference'] and round(r['at_timeout']['vs_reference']['max_abs_err'], 4)
                summary['err_target'] = round(r['at_timeout']['vs_target']['max_abs_err'], 4)
            print(json.dumps(summary), flush=True)
            results.append(r)
            args.k += 1
    rec.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

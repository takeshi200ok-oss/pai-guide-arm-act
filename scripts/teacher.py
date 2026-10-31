#!/usr/bin/env python3
"""CRANE+ V2 の教師（お手本）動作をスクリプトで生成するノード。

physicalai_teleop の teleop_keyboard と同じトピック・メッセージ形式・周期（20Hz）で関節指令を送る。
時間はすべてシミュレーション時計（/clock）で測る。GUI の有無で実時間比が変わっても、
軌道の長さ・保持時間・指令周期はシミュレーション秒で同じになる。

モード:
  demo  : 開始姿勢で保持 → 補間（既定は等速）で目標姿勢へ移動 → 目標姿勢で保持
  reset : 指定姿勢へ戻し、腕の全関節の誤差が許容値以内で静止したら終了（記録には使わない）
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from guide_common import (ARM_JOINTS, ARM_TOPIC, GRIPPER_JOINT, GRIPPER_TOPIC, JOINT_STATES_TOPIC,
                          PERIOD, load_poses, parse_pose, positions_by_name, sim_now,
                          spin_until_sim, wait_for_clock)


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def linear(s):
    return min(max(s, 0.0), 1.0)


# 補間の形。linear（等速）は講義のキーボード操作のように動き出しから一定の速さで指令を進める。
# min_jerk は動き出しの速度が 0 のため、開始姿勢付近の指令が極めて小さくなり、
# 状態だけを見る方策には「どちら向きに動き出すか」の手がかりが弱い（Stage A の開発中に確認）。
PROFILES = {'linear': linear, 'min_jerk': min_jerk}


class Teacher(Node):
    def __init__(self, gripper):
        super().__init__('crane_plus_teacher',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.gripper = gripper
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, 50)
        self.joint_pos = None  # ARM_JOINTS の順
        self.samples = []      # (stamp [sim s], ARM_JOINTS の順の位置)
        self.recording = False

    def _on_joint_state(self, msg):
        self.joint_pos = positions_by_name(msg, ARM_JOINTS)
        if self.recording:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.samples.append((stamp, self.joint_pos))

    def publish(self, arm):
        stamp = self.get_clock().now().to_msg()
        for pub, names, positions in ((self.arm_pub, ARM_JOINTS, arm),
                                      (self.gripper_pub, [GRIPPER_JOINT], [self.gripper])):
            jt = JointTrajectory()
            jt.header.stamp = stamp
            jt.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in positions]
            pt.time_from_start.nanosec = int(0.1 * 1e9)
            jt.points.append(pt)
            pub.publish(jt)

    def wait_subscribers(self, count, wall_timeout=15.0):
        """指令トピックの購読者（コントローラ＋rosbag）が count 以上になるまで待つ。
        待っている間は指令を送らない（コントローラは直前の姿勢を保持する）。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < wall_timeout:
            if min(self.arm_pub.get_subscription_count(), self.gripper_pub.get_subscription_count()) >= count:
                return True
            spin_until_sim(self, sim_now(self) + PERIOD)
        return False

    def run_demo(self, start, goal, pre_hold, move_time, post_hold, profile=linear):
        t0 = sim_now(self)
        total = pre_hold + move_time + post_hold
        n = 0
        while True:
            t = sim_now(self) - t0
            if t > total:
                break
            s = profile((t - pre_hold) / move_time)
            self.publish([a + (b - a) * s for a, b in zip(start, goal)])
            n += 1
            spin_until_sim(self, t0 + n * PERIOD)
        return t0

    def run_reset(self, pose, tol, settle, timeout):
        """pose を送り続け、誤差 tol 以内が settle シミュレーション秒続いたら終了。"""
        t0 = sim_now(self)
        t_in = None
        while sim_now(self) - t0 < timeout:
            self.publish(pose)
            spin_until_sim(self, sim_now(self) + PERIOD)
            if self.joint_pos is None:
                continue
            if max(abs(a - b) for a, b in zip(self.joint_pos, pose)) < tol:
                t_in = t_in or sim_now(self)
                if sim_now(self) - t_in >= settle:
                    return True
            else:
                t_in = None
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=['demo', 'reset'])
    ap.add_argument('--start', default='', help="開始姿勢 例: 'j1=0.1,j3=0.05'（省略した関節は 0）")
    ap.add_argument('--goal', default='', help='目標姿勢（demo のみ）')
    ap.add_argument('--direction', choices=['P', 'N'], help='guide_poses.yaml の目標を goal に使う')
    ap.add_argument('--pre-hold', type=float, default=0.5)
    ap.add_argument('--move-time', type=float, default=2.0)
    ap.add_argument('--profile', choices=list(PROFILES), default='linear')
    ap.add_argument('--post-hold', type=float, default=2.0)
    ap.add_argument('--tol', type=float, default=0.005, help='reset の許容誤差 [rad]')
    ap.add_argument('--settle', type=float, default=0.5, help='reset で許容誤差内に留まる時間 [sim s]')
    ap.add_argument('--timeout', type=float, default=20.0, help='reset のタイムアウト [sim s]')
    ap.add_argument('--wait-subscribers', type=int, default=1,
                    help='demo 開始前に待つ指令トピックの購読者数（rosbag 記録時は 2）')
    ap.add_argument('--meta-out', help='demo のパラメータと実測した関節状態を書き出す JSON')
    args = ap.parse_args()

    poses = load_poses()
    start = parse_pose(args.start)
    goal = dict(poses['directions'][args.direction]) if args.direction else parse_pose(args.goal)
    start_v = [start[n] for n in ARM_JOINTS]
    goal_v = [goal[n] for n in ARM_JOINTS]

    rclpy.init()
    node = Teacher(poses['gripper'])
    ok = True
    try:
        wait_for_clock(node)
        spin_until_sim(node, sim_now(node) + 0.3)
        if args.mode == 'reset':
            ok = node.run_reset(start_v, args.tol, args.settle, args.timeout)
            print(f'reset {"ok" if ok else "TIMEOUT"}: joint={node.joint_pos}')
        else:
            if not node.wait_subscribers(args.wait_subscribers):
                raise RuntimeError(f'指令トピックの購読者が {args.wait_subscribers} に達しない')
            # rosbag が購読を始めた直後の取りこぼしを避けるため少し待つ（指令は送らない）
            spin_until_sim(node, sim_now(node) + 0.3)
            node.recording = True
            t0 = node.run_demo(start_v, goal_v, args.pre_hold, args.move_time, args.post_hold,
                               PROFILES[args.profile])
            node.recording = False
            if args.meta_out:
                meta = {
                    'joint_order': ARM_JOINTS,
                    'start': start, 'goal': goal, 'gripper': poses['gripper'],
                    'pre_hold': args.pre_hold, 'move_time': args.move_time, 'post_hold': args.post_hold,
                    'profile': args.profile,
                    'first_command_sim_time': t0,
                    'final_joint_pos': dict(zip(ARM_JOINTS, node.joint_pos)),
                    'joint_states': [[round(t - t0, 4), [round(p, 5) for p in pos]] for t, pos in node.samples],
                }
                with open(args.meta_out, 'w') as f:
                    json.dump(meta, f, indent=1)
            print(f'demo done: final joint={dict(zip(ARM_JOINTS, node.joint_pos))}')
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

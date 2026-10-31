#!/usr/bin/env python3
"""学習済み ACT で CRANE+ V2 に関節指令を出す推論ノード（pai_training の pixi 環境で実行）。

pai_training/scripts/infer.py（講義教材）を参考に、次を変更した:
  - シミュレーション時計（use_sim_time）で 20Hz の推論周期を刻む
  - /joint_states は関節名で並べ替え、必要な名前が欠けていれば指令を出さない（0 で埋めない）
  - 起動時に policy.reset() で行動キューを空にする
  - 指令メッセージの header.stamp にシミュレーション時刻を入れる（評価の開始時刻に使う）

実行例:
  pixi run --manifest-path ~/pai_training/pyproject.toml \
    python ~/guide_arm/scripts/infer_node.py --model-path <checkpoint>
"""
import argparse

import numpy as np
import rclpy
import torch
from lerobot.common.policies.act.modeling_act import ACTPolicy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from guide_common import (ARM_JOINTS, ARM_TOPIC, GRIPPER_JOINT, GRIPPER_TOPIC, JOINT_STATES_TOPIC,
                          PERIOD, STATE_JOINTS, positions_by_name)


class InferenceNode(Node):
    def __init__(self, model_path):
        super().__init__('crane_plus_act_infer',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.policy = ACTPolicy.from_pretrained(model_path)
        self.policy.eval()
        self.policy.to('cpu')
        self.policy.reset()
        self.input_keys = list(self.policy.config.input_features)
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10)
        self.latest = None
        self.missing_logged = False
        self.steps = 0
        self.create_timer(PERIOD, self._step)  # use_sim_time なのでシミュレーション時間の周期
        self.get_logger().info(f'loaded {model_path}; inputs={self.input_keys}')

    def _on_joint_state(self, msg):
        try:
            self.latest = np.array(positions_by_name(msg, STATE_JOINTS), dtype=np.float32)
        except KeyError as e:
            self.latest = None
            if not self.missing_logged:
                self.get_logger().error(str(e))
                self.missing_logged = True

    def _step(self):
        if self.latest is None:
            return
        state = torch.from_numpy(self.latest).unsqueeze(0)
        obs = {key: state for key in self.input_keys}  # 講義と同じく environment_state にも関節状態を入れる
        with torch.no_grad():
            action = self.policy.select_action(obs).cpu().numpy().flatten()
        stamp = self.get_clock().now().to_msg()
        for pub, names, positions in ((self.arm_pub, ARM_JOINTS, action[:4]),
                                      (self.gripper_pub, [GRIPPER_JOINT], action[4:5])):
            jt = JointTrajectory()
            jt.header.stamp = stamp
            jt.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in positions]
            pt.time_from_start.nanosec = int(0.1 * 1e9)
            jt.points.append(pt)
            pub.publish(jt)
        self.steps += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model-path', required=True)
    args = ap.parse_args()
    rclpy.init()
    node = InferenceNode(args.model_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'stopped after {node.steps} steps')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

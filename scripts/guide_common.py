"""案内アームの各スクリプトで共有する定義と、シミュレーション時計のヘルパー。"""
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / 'configs'

# 値の並び。講義（physicalai_teleop / pai_training）と同じ順にそろえ、列名もこの順で付ける。
ARM_JOINTS = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint4', 'crane_plus_joint3']
GRIPPER_JOINT = 'crane_plus_joint_hand'
STATE_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
FEATURE_NAMES = ['joint1', 'joint2', 'joint4', 'joint3', 'gripper']

ARM_TOPIC = '/crane_plus_arm_controller/joint_trajectory'
GRIPPER_TOPIC = '/crane_plus_gripper_controller/joint_trajectory'
JOINT_STATES_TOPIC = '/joint_states'
PERIOD = 0.05  # 指令周期 [シミュレーション秒]（20Hz）

SHORT_NAMES = {'j1': 'crane_plus_joint1', 'j2': 'crane_plus_joint2',
               'j3': 'crane_plus_joint3', 'j4': 'crane_plus_joint4'}


def load_yaml(name):
    with open(CONFIG_DIR / name) as f:
        return yaml.safe_load(f)


def load_poses():
    return load_yaml('guide_poses.yaml')


def parse_pose(text):
    """'j1=0.8,j3=0.6' 形式を関節名の辞書にする。書かれていない腕関節は 0。"""
    pose = {name: 0.0 for name in ARM_JOINTS}
    if text:
        for item in text.split(','):
            key, val = item.split('=')
            name = SHORT_NAMES.get(key.strip(), key.strip())
            if name not in pose:
                raise ValueError(f'未知の関節名: {key}')
            pose[name] = float(val)
    return pose


def format_pose(pose):
    rev = {v: k for k, v in SHORT_NAMES.items()}
    return ','.join(f'{rev[n]}={pose[n]:.4f}' for n in ARM_JOINTS)


def positions_by_name(msg, names):
    """JointState などから names の順に位置を取り出す。名前が欠けていれば KeyError。"""
    table = dict(zip(msg.name, msg.position))
    missing = [n for n in names if n not in table]
    if missing:
        raise KeyError(f'関節メッセージに必要な名前がない: {missing}')
    return [table[n] for n in names]


def sim_now(node):
    return node.get_clock().now().nanoseconds * 1e-9


def wait_for_clock(node, timeout=10.0):
    """use_sim_time のノードで /clock を受け取るまで待つ。"""
    import rclpy
    t0 = time.monotonic()
    while sim_now(node) == 0.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() - t0 > timeout:
            raise RuntimeError('/clock を受信できない（シミュレータが起動していない？）')


def spin_until_sim(node, t_sim, wall_timeout=60.0):
    """シミュレーション時刻 t_sim までコールバックを処理しながら待つ。"""
    import rclpy
    t0 = time.monotonic()
    while sim_now(node) < t_sim:
        rclpy.spin_once(node, timeout_sec=0.002)
        if time.monotonic() - t0 > wall_timeout:
            raise RuntimeError('シミュレーション時間が進まない')

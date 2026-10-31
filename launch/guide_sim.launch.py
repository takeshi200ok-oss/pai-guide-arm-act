# Copyright 2022 RT Corporation
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
# 変更の記録（このリポジトリでの変更）:
# crane_plus_gazebo/launch/crane_plus_with_table.launch.py（rt-net/crane_plus, commit 19c1d606）を基に変更:
#  - world:=<sdf> でワールドを指定できるようにする（既定は guide_arm/worlds/guide_table.sdf）
#  - MoveIt (move_group, RViz) を起動しない（学習・推論に不要で CPU を消費するため）
#  - gui:=false でサーバのみ起動できるようにする（CPU 描画の負荷で RTF が下がるのを避ける）
import os

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('crane_plus_description'))}
    default_world = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'guide_table.sdf')
    world_file = LaunchConfiguration('world')
    gui_config = os.path.join(
        get_package_share_directory('crane_plus_gazebo'), 'gui', 'gui.config')
    gui = LaunchConfiguration('gui')

    ign_gazebo_gui = ExecuteProcess(
        cmd=['ign gazebo -r', world_file, '--gui-config', gui_config],
        output='screen', additional_env=env, shell=True, condition=IfCondition(gui))
    ign_gazebo_server = ExecuteProcess(
        cmd=['ign gazebo -r -s', world_file],
        output='screen', additional_env=env, shell=True, condition=UnlessCondition(gui))

    gazebo_spawn_entity = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/robot_description', '-name', 'crane_plus',
                   '-z', '1.015', '-allow_renaming', 'true'])

    description_loader = RobotDescriptionLoader()
    description_loader.use_gazebo = 'true'
    description_loader.use_ignition = 'true'
    description_loader.gz_control_config_package = 'crane_plus_control'
    description_loader.gz_control_config_file_path = 'config/crane_plus_controllers.yaml'
    description = description_loader.load()

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': description}])

    spawners = [
        ExecuteProcess(cmd=[f'ros2 run controller_manager spawner {name}'], shell=True, output='screen')
        for name in ('joint_state_broadcaster', 'crane_plus_arm_controller', 'crane_plus_gripper_controller')
    ]

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        SetParameter(name='use_sim_time', value=True),
        ign_gazebo_gui,
        ign_gazebo_server,
        gazebo_spawn_entity,
        robot_state_publisher,
        *spawners,
        bridge,
    ])

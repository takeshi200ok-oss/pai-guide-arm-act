# 案内アーム共通環境（講義端末の ~/.bashrc と同じ ROS 設定を明示的に読み込む）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source ~/airobot_ws/install/setup.bash
source ~/pai_ws/install/setup.bash
export PATH="$HOME/.pixi/bin:$PATH"
export DISPLAY=:0
export GUIDE_ARM=~/guide_arm
export GUIDE_RUNS=~/guide_runs
export PAI_TRAINING=~/pai_training
mkdir -p "$GUIDE_RUNS/logs"

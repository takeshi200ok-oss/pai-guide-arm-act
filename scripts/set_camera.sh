#!/bin/bash
# configs/camera.yaml の位置に Gazebo GUI のカメラを動かす
source ~/guide_arm/scripts/env.sh
req=$(python3 - <<'PY'
import yaml
c = yaml.safe_load(open('/home/ubuntu/guide_arm/configs/camera.yaml'))
p, o = c['position'], c['orientation']
print(f"pose: {{position: {{x: {p['x']}, y: {p['y']}, z: {p['z']}}}, orientation: {{w: {o['w']}, x: {o['x']}, y: {o['y']}, z: {o['z']}}}}}")
PY
)
ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean --timeout 3000 --req "$req"

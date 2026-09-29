#!/bin/bash
# Replay the wafer transfer in RViz (e.g. The Construct's Graphical Tools).
# Usage: ros2/tools/replay.sh [delay_s] [launch args...]
#   e.g. ros2/tools/replay.sh 0 place_theta_deg:=170.0
#
# Assumes the package is built in ~/ros2_ws and the display is :1 (The
# Construct). A minimised xterm keeps the display alive while RViz restarts,
# otherwise the browser viewer disconnects every time RViz closes.

source ~/ros2_ws/install/setup.bash
export DISPLAY=${DISPLAY:-:1}

pgrep -f 'title keepalive' > /dev/null || (setsid xterm -iconic -title keepalive > /dev/null 2>&1 &)

"$(dirname "$0")/stop_robot.sh" > /dev/null
sleep "${1:-0}"

setsid ros2 launch wafer_transfer_robot wafer_transfer.launch.py "${@:2}" > /tmp/replay.txt 2>&1 &
echo "replay started (log: /tmp/replay.txt)"

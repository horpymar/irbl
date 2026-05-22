#!/bin/bash
set -e

UAV_NAME=${UAV_NAME:-uav1}
BENCHMARK_GOAL_X=${BENCHMARK_GOAL_X:-97.0}
BENCHMARK_GOAL_Y=${BENCHMARK_GOAL_Y:-2.0}
BENCHMARK_GOAL_Z=${BENCHMARK_GOAL_Z:-2.0}
BENCHMARK_GOAL_HEADING=${BENCHMARK_GOAL_HEADING:-0.0}
BENCHMARK_GOAL_DELAY=${BENCHMARK_GOAL_DELAY:-10}

echo "[benchmark_goal] waiting for ROS"
until ros2 node list >/dev/null 2>&1; do sleep 1; done

echo "[benchmark_goal] waiting for RBL services"
until ros2 service list | grep -q "^/${UAV_NAME}/rbl_controller/activation$"; do sleep 1; done
until ros2 service list | grep -q "^/${UAV_NAME}/rbl_controller/goto$"; do sleep 1; done

echo "[benchmark_goal] activating RBL controller"
ros2 service call "/${UAV_NAME}/rbl_controller/activation" std_srvs/srv/Trigger "{}" || true

echo "[benchmark_goal] sleeping ${BENCHMARK_GOAL_DELAY}s before benchmark goal"
sleep "$BENCHMARK_GOAL_DELAY"

echo "[benchmark_goal] sending goal [${BENCHMARK_GOAL_X}, ${BENCHMARK_GOAL_Y}, ${BENCHMARK_GOAL_Z}, ${BENCHMARK_GOAL_HEADING}]"
ros2 service call "/${UAV_NAME}/rbl_controller/goto" mrs_msgs/srv/Vec4 "{goal: [${BENCHMARK_GOAL_X}, ${BENCHMARK_GOAL_Y}, ${BENCHMARK_GOAL_Z}, ${BENCHMARK_GOAL_HEADING}]}"

#!/bin/bash
set -euo pipefail

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

COUNT_PER_MODE=${COUNT_PER_MODE:-15}
BENCHMARK_TIMEOUT=${BENCHMARK_TIMEOUT:-240}
RUN_NO_REPLANNER=${RUN_NO_REPLANNER:-1}
RUN_ASTAR=${RUN_ASTAR:-1}
TMUX_SESSION_NAME=${TMUX_SESSION_NAME:-simulation}
TMUX_SOCKET_NAME=${TMUX_SOCKET_NAME:-mrs}
TMUXINATOR_CONFIG=${TMUXINATOR_CONFIG:-./session.yml}
BETWEEN_RUN_SLEEP=${BETWEEN_RUN_SLEEP:-5}

UAV_NAME=${UAV_NAME:-uav1}
BENCHMARK_GOAL_X=${BENCHMARK_GOAL_X:-97.0}
BENCHMARK_GOAL_Y=${BENCHMARK_GOAL_Y:-2.0}
BENCHMARK_GOAL_Z=${BENCHMARK_GOAL_Z:-2.0}
BENCHMARK_GOAL_HEADING=${BENCHMARK_GOAL_HEADING:-0.0}
BENCHMARK_GOAL_DELAY=${BENCHMARK_GOAL_DELAY:-10}

kill_existing_session() {
  if tmux -L "$TMUX_SOCKET_NAME" has-session -t "$TMUX_SESSION_NAME" >/dev/null 2>&1; then
    echo "[benchmark_batch] killing existing tmux session ${TMUX_SESSION_NAME}"
    tmux -L "$TMUX_SOCKET_NAME" kill-session -t "$TMUX_SESSION_NAME" || true
    sleep 2
  fi
}

run_one() {
  local mode="$1"
  local run_idx="$2"
  local config="$3"
  local stamp
  local bag_dir
  local done_file
  local start_wall
  local max_wait

  stamp=$(date +%Y%m%d_%H%M%S)
  bag_dir="./benchmark_bags/${mode}_${stamp}_run$(printf "%02d" "$run_idx")"
  done_file="${bag_dir}/benchmark_done.txt"
  max_wait=$((BENCHMARK_TIMEOUT + 180))

  echo
  echo "[benchmark_batch] starting ${mode} run ${run_idx}/${COUNT_PER_MODE}"
  echo "[benchmark_batch] bag_dir=${bag_dir}"

  kill_existing_session

  export UAV_NAME
  export RBL_CONTROLLER_CONFIG="$config"
  export BENCHMARK_MODE="$mode"
  export BENCHMARK_TIMEOUT
  export BENCHMARK_BAG_DIR="$bag_dir"
  export BENCHMARK_DONE_FILE="$done_file"
  export BENCHMARK_GOAL_X
  export BENCHMARK_GOAL_Y
  export BENCHMARK_GOAL_Z
  export BENCHMARK_GOAL_HEADING
  export BENCHMARK_GOAL_DELAY

  tmuxinator start -p "$TMUXINATOR_CONFIG"

  start_wall=$(date +%s)
  while [[ ! -f "$done_file" ]]; do
    if (( $(date +%s) - start_wall > max_wait )); then
      echo "[benchmark_batch] run timed out while waiting for recorder marker"
      break
    fi
    sleep 2
  done

  if [[ -f "$done_file" ]]; then
    echo "[benchmark_batch] recorder finished: $(tr -d '\n' < "$done_file")"
  fi

  kill_existing_session
  sleep "$BETWEEN_RUN_SLEEP"
}

run_mode() {
  local mode="$1"
  local config="$2"
  local run_idx

  for run_idx in $(seq 1 "$COUNT_PER_MODE"); do
    run_one "$mode" "$run_idx" "$config"
  done
}

if [[ "$RUN_NO_REPLANNER" == "1" ]]; then
  run_mode "no_replanner" "./config/rbl_controller.yaml"
fi

if [[ "$RUN_ASTAR" == "1" ]]; then
  run_mode "astar" "./config/rbl_controller_astar.yaml"
fi

echo
echo "[benchmark_batch] finished batch"
echo "[benchmark_batch] analyze with:"
echo "  ./analyze_benchmark_bag.py benchmark_bags --plots"

#!/bin/bash

set -euo pipefail

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
WORKSPACE_SRC=$(readlink -f "$SCRIPTPATH/../../..")

COUNT=${COUNT:-20}
RANDOM_GOAL=${RANDOM_GOAL:-0}
BUILD_BEFORE_RUN=${BUILD_BEFORE_RUN:-0}
BENCHMARK_TIMEOUT=${BENCHMARK_TIMEOUT:-240}
BENCHMARK_GOAL_DELAY=${BENCHMARK_GOAL_DELAY:-10}
BETWEEN_RUN_SLEEP=${BETWEEN_RUN_SLEEP:-5}
UAV_NAME=${UAV_NAME:-uav1}

RBL_CONTROLLER_CONFIG=${RBL_CONTROLLER_CONFIG:-./config/rbl_controller.yaml}
TMUX_SESSION_NAME=${TMUX_SESSION_NAME:-simulation}
TMUX_SOCKET_NAME=${TMUX_SOCKET_NAME:-mrs}
TMUXINATOR_CONFIG=${TMUXINATOR_CONFIG:-./session.yml}

FIXED_GOAL_X=${FIXED_GOAL_X:-97.0}
FIXED_GOAL_Y=${FIXED_GOAL_Y:-2.0}
FIXED_GOAL_Z=${FIXED_GOAL_Z:-2.0}
FIXED_GOAL_HEADING=${FIXED_GOAL_HEADING:-0.0}

RANDOM_GOAL_X_MIN=${RANDOM_GOAL_X_MIN:-0.0}
RANDOM_GOAL_X_MAX=${RANDOM_GOAL_X_MAX:-100.0}
RANDOM_GOAL_Y_MIN=${RANDOM_GOAL_Y_MIN:--20.0}
RANDOM_GOAL_Y_MAX=${RANDOM_GOAL_Y_MAX:-20.0}
RANDOM_GOAL_Z=${RANDOM_GOAL_Z:-2.0}
RANDOM_GOAL_HEADING=${RANDOM_GOAL_HEADING:-0.0}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --random-goal)
      RANDOM_GOAL=1
      shift
      ;;
    --fixed-goal)
      RANDOM_GOAL=0
      shift
      ;;
    --count)
      COUNT="$2"
      shift 2
      ;;
    --timeout)
      BENCHMARK_TIMEOUT="$2"
      shift 2
      ;;
    --build)
      BUILD_BEFORE_RUN=1
      shift
      ;;
    --no-build)
      BUILD_BEFORE_RUN=0
      shift
      ;;
    --config)
      RBL_CONTROLLER_CONFIG="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

kill_existing_session() {
  if tmux -L "$TMUX_SOCKET_NAME" has-session -t "$TMUX_SESSION_NAME" >/dev/null 2>&1; then
    echo "[rebuild_batch] killing existing tmux session ${TMUX_SESSION_NAME}"
    tmux -L "$TMUX_SOCKET_NAME" kill-session -t "$TMUX_SESSION_NAME" || true
    sleep 2
  fi
}

sample_uniform() {
  local min_value="$1"
  local max_value="$2"
  awk -v min="$min_value" -v max="$max_value" -v seed="$RANDOM$RANDOM" \
    'BEGIN { srand(seed); printf "%.3f", min + rand() * (max - min) }'
}

select_goal() {
  if [[ "$RANDOM_GOAL" == "1" ]]; then
    BENCHMARK_GOAL_X=$(sample_uniform "$RANDOM_GOAL_X_MIN" "$RANDOM_GOAL_X_MAX")
    BENCHMARK_GOAL_Y=$(sample_uniform "$RANDOM_GOAL_Y_MIN" "$RANDOM_GOAL_Y_MAX")
    BENCHMARK_GOAL_Z="$RANDOM_GOAL_Z"
    BENCHMARK_GOAL_HEADING="$RANDOM_GOAL_HEADING"
  else
    BENCHMARK_GOAL_X="$FIXED_GOAL_X"
    BENCHMARK_GOAL_Y="$FIXED_GOAL_Y"
    BENCHMARK_GOAL_Z="$FIXED_GOAL_Z"
    BENCHMARK_GOAL_HEADING="$FIXED_GOAL_HEADING"
  fi
}

cd "$SCRIPTPATH"
kill_existing_session

cd "$WORKSPACE_SRC"

if [[ "$BUILD_BEFORE_RUN" == "1" ]]; then
  colcon build --symlink-install --packages-select ciri rbl_controller_core rbl_replanner rbl_controller_node
fi

if [[ -f install/setup.bash ]]; then
  # Batch launch note: colcon setup scripts may read unset variables such as COLCON_TRACE.
  # Temporarily disabling nounset keeps this runner robust while preserving strict checks elsewhere.
  set +u
  source install/setup.bash
  set -u
fi

cd "$SCRIPTPATH"

batch_stamp=$(date +%Y%m%d_%H%M%S)
results_dir="./benchmark_results/rebuild_batch_${batch_stamp}"
mkdir -p "$results_dir"
results_csv="${results_dir}/results.csv"

echo "run,goal_x,goal_y,goal_z,goal_heading,result,bag_dir,started_at,finished_at" > "$results_csv"

echo "[rebuild_batch] count=${COUNT}"
echo "[rebuild_batch] random_goal=${RANDOM_GOAL}"
echo "[rebuild_batch] build_before_run=${BUILD_BEFORE_RUN}"
echo "[rebuild_batch] config=${RBL_CONTROLLER_CONFIG}"
echo "[rebuild_batch] result topic=/benchmark/event via benchmark_done.txt"
echo "[rebuild_batch] path topic recorded=/${UAV_NAME}/rbl_controller/path"
echo "[rebuild_batch] results=${results_csv}"

for run_idx in $(seq 1 "$COUNT"); do
  select_goal

  stamp=$(date +%Y%m%d_%H%M%S)
  bag_dir="./benchmark_bags/rebuild_batch_${stamp}_run$(printf "%02d" "$run_idx")"
  done_file="${bag_dir}/benchmark_done.txt"
  max_wait=$((BENCHMARK_TIMEOUT + 180))
  started_at=$(date -Iseconds)

  echo
  echo "[rebuild_batch] run ${run_idx}/${COUNT}"
  echo "[rebuild_batch] goal=[${BENCHMARK_GOAL_X}, ${BENCHMARK_GOAL_Y}, ${BENCHMARK_GOAL_Z}, ${BENCHMARK_GOAL_HEADING}]"
  echo "[rebuild_batch] bag_dir=${bag_dir}"

  kill_existing_session

  export UAV_NAME
  export RBL_CONTROLLER_CONFIG
  export BENCHMARK_MODE="rebuild_batch"
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
      echo "[rebuild_batch] run timed out while waiting for benchmark_done.txt"
      break
    fi
    sleep 2
  done

  if [[ -f "$done_file" ]]; then
    result=$(tr -d '\n' < "$done_file")
  else
    result="runner_timeout"
  fi
  finished_at=$(date -Iseconds)

  echo "[rebuild_batch] result=${result}"
  echo "${run_idx},${BENCHMARK_GOAL_X},${BENCHMARK_GOAL_Y},${BENCHMARK_GOAL_Z},${BENCHMARK_GOAL_HEADING},${result},${bag_dir},${started_at},${finished_at}" >> "$results_csv"

  kill_existing_session
  sleep "$BETWEEN_RUN_SLEEP"
done

echo
echo "[rebuild_batch] finished"
echo "[rebuild_batch] results=${results_csv}"
echo "[rebuild_batch] analyze bags with:"
echo "  ./analyze_benchmark_bag.py benchmark_bags --plots"

#!/bin/bash
set -euo pipefail

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

# A*-only benchmark helper.
# This script intentionally skips the no_replanner baseline and repeatedly reuses
# run_benchmark_batch.sh so recording, timeout handling, tmux cleanup, and bag
# layout stay identical to the normal benchmark pipeline.

COUNT=${COUNT:-20}
BENCHMARK_TIMEOUT=${BENCHMARK_TIMEOUT:-240}
BENCHMARK_GOAL_DELAY=${BENCHMARK_GOAL_DELAY:-10}
BETWEEN_RUN_SLEEP=${BETWEEN_RUN_SLEEP:-5}

# Fixed goal mode by default: [90, 0, 2, 0].
# Set RANDOM_GOALS=1 to sample a new XY goal for each run.
RANDOM_GOALS=${RANDOM_GOALS:-0}
BENCHMARK_GOAL_X=${BENCHMARK_GOAL_X:-90.0}
BENCHMARK_GOAL_Y=${BENCHMARK_GOAL_Y:-0.0}
BENCHMARK_GOAL_Z=${BENCHMARK_GOAL_Z:-2.0}
BENCHMARK_GOAL_HEADING=${BENCHMARK_GOAL_HEADING:-0.0}

# Conservative default random bounds for the current forest_real map/rosbag.
RANDOM_GOAL_X_MIN=${RANDOM_GOAL_X_MIN:-20.0}
RANDOM_GOAL_X_MAX=${RANDOM_GOAL_X_MAX:-97.0}
RANDOM_GOAL_Y_MIN=${RANDOM_GOAL_Y_MIN:--10.0}
RANDOM_GOAL_Y_MAX=${RANDOM_GOAL_Y_MAX:-10.0}
RANDOM_GOAL_Z=${RANDOM_GOAL_Z:-2.0}
RANDOM_GOAL_HEADING=${RANDOM_GOAL_HEADING:-0.0}

sample_uniform() {
  local min_value="$1"
  local max_value="$2"
  awk -v min="$min_value" -v max="$max_value" -v seed="$RANDOM$RANDOM" \
    'BEGIN { srand(seed); printf "%.3f", min + rand() * (max - min) }'
}

for run_idx in $(seq 1 "$COUNT"); do
  if [[ "$RANDOM_GOALS" == "1" ]]; then
    export BENCHMARK_GOAL_X
    export BENCHMARK_GOAL_Y
    export BENCHMARK_GOAL_Z
    export BENCHMARK_GOAL_HEADING
    BENCHMARK_GOAL_X=$(sample_uniform "$RANDOM_GOAL_X_MIN" "$RANDOM_GOAL_X_MAX")
    BENCHMARK_GOAL_Y=$(sample_uniform "$RANDOM_GOAL_Y_MIN" "$RANDOM_GOAL_Y_MAX")
    BENCHMARK_GOAL_Z="$RANDOM_GOAL_Z"
    BENCHMARK_GOAL_HEADING="$RANDOM_GOAL_HEADING"
  fi

  echo
  echo "[astar_20] run ${run_idx}/${COUNT}, goal=[${BENCHMARK_GOAL_X}, ${BENCHMARK_GOAL_Y}, ${BENCHMARK_GOAL_Z}, ${BENCHMARK_GOAL_HEADING}]"

  RUN_NO_REPLANNER=0 \
  RUN_ASTAR=1 \
  COUNT_PER_MODE=1 \
  BENCHMARK_TIMEOUT="$BENCHMARK_TIMEOUT" \
  BENCHMARK_GOAL_DELAY="$BENCHMARK_GOAL_DELAY" \
  BETWEEN_RUN_SLEEP="$BETWEEN_RUN_SLEEP" \
  BENCHMARK_GOAL_X="$BENCHMARK_GOAL_X" \
  BENCHMARK_GOAL_Y="$BENCHMARK_GOAL_Y" \
  BENCHMARK_GOAL_Z="$BENCHMARK_GOAL_Z" \
  BENCHMARK_GOAL_HEADING="$BENCHMARK_GOAL_HEADING" \
  ./run_benchmark_batch.sh
done

echo
echo "[astar_20] finished"
echo "[astar_20] analyze with:"
echo "  ./analyze_benchmark_bag.py benchmark_bags --plots"

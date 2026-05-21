#!/bin/bash

set -e

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
WORKSPACE_SRC=$(readlink -f "$SCRIPTPATH/../../..")

cd "$SCRIPTPATH"

if tmux -L mrs has-session -t simulation 2>/dev/null; then
  tmux -L mrs kill-session -t simulation
fi

cd "$WORKSPACE_SRC"

colcon build --symlink-install --packages-select rbl_controller_core rbl_replanner rbl_controller_node

if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

cd "$SCRIPTPATH"
exec ./start.sh

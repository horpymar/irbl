#!/bin/bash
set -e

echo "[benchmark_record] waiting for ROS"
until ros2 node list >/dev/null 2>&1; do sleep 1; done

exec python3 ./benchmark_record_until_done.py

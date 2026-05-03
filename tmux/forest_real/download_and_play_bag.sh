#!/bin/bash

URL="https://nasmrs.fel.cvut.cz:3923/PERSONAL_DIRECTORIES/manuel/BAGS4GITHUB/rosbag2_2026_04_29-11_59_36_0.mcap"
OUT_FILE="rosbag.mcap"
HOST="nasmrs.fel.cvut.cz"
rm -rf $OUT_FILE
echo "[INFO] Checking internet connectivity..."

# Try to reach the host (fast + reliable check)
if ! wget --spider --quiet --timeout=3 "$URL"; then
  echo "[ERROR] Cannot reach the download server!"
  exit 1
fi

echo "[INFO] Internet OK"

echo "[INFO] Ensuring bag is available..."

# Download or resume
if ! wget -c -O "$OUT_FILE" "$URL"; then
  echo "[ERROR] Download failed (network issue?)"
  exit 1
fi

# Validate file
if [ ! -s "$OUT_FILE" ]; then
  echo "[ERROR] File is empty or corrupted!"
  exit 1
fi

echo "[INFO] Bag ready: $OUT_FILE"

echo "[INFO] Playing bag..."
ros2 bag play "$OUT_FILE" -r 0.1 -l

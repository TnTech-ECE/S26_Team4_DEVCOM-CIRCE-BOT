#!/usr/bin/env bash
# Stop the CIRCE sim and verify it died; a surviving gz server makes the next launch
# spawn a second rover. Harmonic runs as `ruby gz sim ...`, so match "gz sim", not
# "gz-sim". The [x] bracket stops pkill matching this script itself.
set -u

echo "stopping CIRCE sim..."
pkill -INT -f "[c]irce_sim"            2>/dev/null
pkill -INT -f "[g]z sim"               2>/dev/null
pkill -INT -f "[o]pt/ros/jazzy/lib"    2>/dev/null
sleep 3

pkill -KILL -f "[c]irce_sim"           2>/dev/null
pkill -KILL -f "[g]z sim"              2>/dev/null
pkill -KILL -f "[o]pt/ros/jazzy/lib"   2>/dev/null
pkill -KILL -f "[c]irce_perception"    2>/dev/null
sleep 2

# pgrep -c exits 1 on no match; don't let that abort the script.
count() { pgrep -c -f "$1" 2>/dev/null || true; }

ros=$(count "[o]pt/ros/jazzy/lib")
gz=$(count "[g]z sim")
ros=${ros:-0}
gz=${gz:-0}

echo "remaining: ros=${ros} gazebo=${gz}"
if [ "${ros}" -ne 0 ] || [ "${gz}" -ne 0 ]; then
  echo "STILL RUNNING - do not relaunch, you will get a second rover:"
  pgrep -a -f "[o]pt/ros/jazzy/lib" || true
  pgrep -a -f "[g]z sim" || true
  exit 1
fi
echo "clean."

#!/usr/bin/env bash
# Stop everything the CIRCE simulation starts, and verify it actually died.
#
# *** WHY THIS SCRIPT EXISTS ***
# Ctrl-C in the launch terminal does not always take the Gazebo server with it. If the
# server survives and you launch again, `ros_gz_sim create` spawns a SECOND robot into
# the still-running world. Two rovers then share one /robot_description, one /scan and
# one /odom: the lidar sees the twin as a solid obstacle a few centimetres away, the
# costmap fills with phantoms, and the new spawn can drop on top of the old one and
# flip it. Every symptom looks like a perception or controller bug, and none of it is.
#
# *** THE TRAP THIS SCRIPT AVOIDS ***
# On Gazebo Harmonic the server is NOT called anything containing "gz-sim". `gz` is a
# ruby wrapper, so the processes are:
#
#     comm    args
#     ruby    gz sim -r <world>.sdf
#     ruby    gz sim server
#     ruby    gz sim gui
#
# `pkill -f gz-sim` matches none of them and silently leaves the server running, which
# is exactly how the duplicate spawn happens. Match on "gz sim" instead.
#
# The [g] bracket trick keeps the pattern from matching this script's own command line
# in ps output - without it, pkill can kill the shell running the pkill.
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

# pgrep -c prints 0 AND exits 1 when it matches nothing, so it must never be the last
# command in a pipeline the shell judges, and its output must be taken as data. Getting
# this wrong made an earlier version of this script abort before it printed anything.
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

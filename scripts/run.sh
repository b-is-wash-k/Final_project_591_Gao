#!/usr/bin/env bash
# run.sh: start the whole simulation with one command.
#   1. Gazebo with worlds/escape_maze.sdf
#   2. one TurtleBot3 per robot in config/scenario.yaml (robot_red, robot_blue), at its start_pose
#   3. the switch manager (its output is shown here and saved to a log)
#
# Usage (from anywhere):
#   ./scripts/run.sh              # with the Gazebo window
#   ./scripts/run.sh --headless   # no window (server only)
# Ctrl+C stops everything this script started.
# Logs: logs/run_<date>_<time>/  (gazebo.log, spawn_<robot>.log, switch_manager.log)
#
# Then, in a second terminal:  source scripts/test_helpers.sh

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  echo "Run this script, do not source it:  ./scripts/run.sh"
  return 1
fi

cd "$(dirname "$(readlink -f "$0")")/.." || exit 1   # the maze_gazebo_environment folder

WORLD_FILE=worlds/escape_maze.sdf
WORLD_NAME=escape_maze                 # must match <world name="..."> in the SDF
SCENARIO=config/scenario.yaml
PY=/usr/bin/python3                    # system Python: has the Gazebo (gz) Python bindings
LOG_DIR=logs/run_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG_DIR"

GZ_ARGS=(-r)
[[ "${1:-}" == "--headless" ]] && GZ_ARGS=(-r -s --headless-rendering)

say() { echo "[run] $*"; }

source /opt/ros/jazzy/setup.bash
# Keep discovery on this computer; automatic interface selection can leave the
# GUI and server waiting for each other on machines with VPN/virtual interfaces.
# An explicit GZ_IP still takes precedence for multi-computer setups.
export GZ_IP=${GZ_IP:-127.0.0.1}
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}

# Each background program gets its own session. Gazebo creates separate process
# groups for its GUI and server, so cleanup must stop the whole session.
PIDS=()
start_bg() {   # start_bg <logfile> <command...>
  local log=$1; shift
  setsid "$@" > "$log" 2>&1 &
  PIDS+=($!)
}
cleanup() {
  local status=$?
  trap - EXIT
  trap '' INT TERM   # let cleanup finish even if Ctrl+C is pressed again
  echo
  say "stopping Gazebo, robots and switch manager..."
  # SIGTERM also stops shell/Python jobs that inherited ignored SIGINT when
  # started in the background. Gazebo and ROS launch handle it for shutdown.
  for pid in "${PIDS[@]}"; do pkill -TERM -s "$pid" 2>/dev/null; done
  # Allow the Gazebo wrapper's GUI/server shutdown timeouts to finish before
  # forcing any remaining processes to exit. A second Ctrl+C cannot skip this.
  local deadline=$((SECONDS + 12)) running
  while ((SECONDS < deadline)); do
    running=0
    for pid in "${PIDS[@]}"; do
      if pgrep -s "$pid" >/dev/null; then running=1; break; fi
    done
    ((running)) || break
    sleep 0.1
  done
  for pid in "${PIDS[@]}"; do pkill -KILL -s "$pid" 2>/dev/null; done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null; done
  say "stopped. Logs are in $LOG_DIR"
  exit "$status"
}
trap cleanup INT TERM EXIT

wait_for() {   # wait_for <seconds> <description> <check-function>
  local secs=$1 what=$2 check=$3
  for ((i = 0; i < secs; i++)); do
    $check && return 0
    sleep 1
  done
  say "timed out after ${secs}s waiting for $what. Check the logs in $LOG_DIR"
  exit 1
}
world_up()  { gz service -l 2>/dev/null | grep -q "^/world/$WORLD_NAME/create$"; }
robots_up() {
  local models; models=$(timeout 5 gz model --list 2>/dev/null)
  for r in "${ROBOTS[@]}"; do grep -q -- "- $r$" <<< "$models" || return 1; done
}

# 0. Refuse to start a second copy of the same world (two servers would confuse spawning and topics).
if world_up; then
  say "a Gazebo world named '$WORLD_NAME' is already running. Close it first (or: pkill -f 'gz sim')."
  trap - EXIT; exit 1
fi

# Robot ids and start poses come from the scenario file, so it stays the single source of truth.
mapfile -t ROBOT_LINES < <($PY -c "
import yaml
for r in yaml.safe_load(open('$SCENARIO'))['robots']:
    print(r['id'], r['start_pose'][0], r['start_pose'][1])")
ROBOTS=()
for line in "${ROBOT_LINES[@]}"; do ROBOTS+=("${line%% *}"); done
[[ ${#ROBOTS[@]} -gt 0 ]] || { say "no robots found in $SCENARIO"; exit 1; }

# 1. Gazebo
say "Gazebo  ($WORLD_FILE ${GZ_ARGS[*]})"
say "GZ_IP=$GZ_IP; logs: $LOG_DIR"
start_bg "$LOG_DIR/gazebo.log" gz sim -v 3 "${GZ_ARGS[@]}" "$WORLD_FILE"
wait_for 60 "Gazebo to load the world" world_up

# 2. One TurtleBot3 per robot in the scenario
for line in "${ROBOT_LINES[@]}"; do
  read -r id x y <<< "$line"
  say "spawn $id at ($x, $y)"
  start_bg "$LOG_DIR/spawn_$id.log" ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
    namespace:="$id" robot_name:="$id" x_pose:="$x" y_pose:="$y"
done
wait_for 90 "robots ${ROBOTS[*]} to appear in Gazebo" robots_up
say "robots are in Gazebo: ${ROBOTS[*]}"

# 3. Switch manager. Its messages appear in this terminal and in the log.
# It runs in the background and the script waits on it, so Ctrl+C (or closing the terminal) always cleans up.
say "switch manager: it prints only when a switch or door changes. Ctrl+C stops everything."
say "tests (second terminal):  source scripts/test_helpers.sh"
setsid bash -c "$PY scripts/switch_manager.py '$SCENARIO' 2>&1 | tee '$LOG_DIR/switch_manager.log'" &
MANAGER=$!
PIDS+=($MANAGER)
wait "$MANAGER"

# Only reached if the manager stopped on its own (for example, an error in a rule function).
say "switch manager stopped. Gazebo and the robots are still running."
say "Fix the error, then restart only the manager in another terminal:"
say "    python3 scripts/switch_manager.py $SCENARIO"
say "Press Ctrl+C here to stop everything."
wait "${PIDS[0]}"   # keeps running until Ctrl+C or the Gazebo window is closed

#!/usr/bin/env bash
# run.sh: start the whole simulation with one command.
#   1. Gazebo with worlds/escape_maze.sdf
#   2. one TurtleBot3 per robot in config/scenario.yaml (robot1, robot2, ...), at its start_pose
#   3. the switch manager, which also publishes the world state (/team/world_state)
#   4. with --nav or --planner: Nav2 for every robot, and the executor (/team/plan -> Nav2 goals)
#   5. with --planner: the planner that publishes /team/plan
#
# Usage (from anywhere):
#   ./scripts/run.sh                           # with the Gazebo window; move robots by hand
#   ./scripts/run.sh --headless                # no window (server only)
#   ./scripts/run.sh --planner conventional    # Nav2 + executor + the baseline (rule-based) planner
#   ./scripts/run.sh --planner smart_llm       # Nav2 + executor + the SMART-LLM planner (Gemini;
#                                              #   needs: export GEMINI_API_KEY=<your key>)
#   ./scripts/run.sh --nav                     # Nav2 + executor, no planner (send plans by hand)
# Options can be combined, e.g.  ./scripts/run.sh --headless --planner conventional
# Ctrl+C stops everything this script started.
# Logs: logs/run_<date>_<time>/  (gazebo.log, spawn_<robot>.log, switch_manager.log,
#       navigation.log, executor.log, planner.log)
#
# Then, in a second terminal:  source scripts/test_helpers.sh

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  echo "Run this script, do not source it:  ./scripts/run.sh"
  return 1
fi

cd "$(dirname "$(readlink -f "$0")")/.." || exit 1   # the repository root

WORLD_FILE=worlds/escape_maze.sdf
WORLD_NAME=escape_maze                 # must match <world name="..."> in the SDF
SCENARIO=config/scenario.yaml
PY=/usr/bin/python3                    # system Python: has the Gazebo (gz) Python bindings

GZ_ARGS=(-r); NAV=0; PLANNER=none
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) GZ_ARGS=(-r -s --headless-rendering); shift ;;
    --nav) NAV=1; shift ;;
    --planner) PLANNER=${2:-}; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see ./scripts/run.sh --help)"; exit 1 ;;
  esac
done
case "$PLANNER" in
  none) ;;
  conventional|smart_llm) NAV=1 ;;     # a planner needs robots that can drive
  *) echo "--planner must be none, conventional or smart_llm"; exit 1 ;;
esac

say() { echo "[run] $*"; }

# The SMART-LLM planner calls Gemini: check its key and library before starting anything.
if [[ "$PLANNER" == smart_llm ]]; then
  if [[ -z "${GEMINI_API_KEY:-}" || "$GEMINI_API_KEY" == YOUR_GEMINI_API_KEY_HERE ]]; then
    say "--planner smart_llm needs a Gemini API key:  export GEMINI_API_KEY=<your key>"
    say "(never write the key into a file in this repository)"
    exit 1
  fi
  if ! $PY -c "import google.genai" 2>/dev/null; then
    say "--planner smart_llm needs the Gemini library for $PY (it installs into ~/.local only):"
    say "    $PY -m pip install --user --break-system-packages google-genai"
    exit 1
  fi
fi

LOG_DIR=logs/run_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG_DIR"

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
start_shown() {   # start_shown <logfile> <prefix> <command string>: output to the log and this terminal
  local log=$1 prefix=$2 cmd=$3
  setsid bash -c "$cmd 2>&1 | tee '$log' | sed -u 's/^/$prefix/'" &
  PIDS+=($!)
}
cleanup() {
  local status=$?
  trap - EXIT
  trap '' INT TERM   # let cleanup finish even if Ctrl+C is pressed again
  echo
  say "stopping everything this script started..."
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
# The Gazebo window learns about a new model only from the one "state" message sent when the
# model is created. A robot spawned before the window subscribes is never shown in it.
gui_listening() { timeout 5 gz topic -i -t "/world/$WORLD_NAME/state" 2>/dev/null | sed -n '/Subscribers/,$p' | grep -q "tcp://"; }
robots_up() {
  local models; models=$(timeout 5 gz model --list 2>/dev/null)
  for r in "${ROBOTS[@]}"; do grep -q -- "- $r$" <<< "$models" || return 1; done
}
nav_up() { [[ $(grep -c "Managed nodes are active" "$LOG_DIR/navigation.log" 2>/dev/null) -ge ${#ROBOTS[@]} ]]; }
executor_up() { grep -q "executor up" "$LOG_DIR/executor.log" 2>/dev/null; }

# 0. Refuse to start a second copy of the same world (two servers would confuse spawning and topics).
if world_up; then
  say "a Gazebo world named '$WORLD_NAME' is already running. Close it first (or: pkill -f 'gz sim')."
  trap - EXIT; exit 1
fi

# Robot ids and start poses come from the scenario file, so it stays the single source of truth.
mapfile -t ROBOT_LINES < <($PY -c "
import yaml
for r in yaml.safe_load(open('$SCENARIO'))['robots']:
    x, y, yaw = (list(r['start_pose']) + [0.0, 0.0, 0.0])[:3]
    print(r['id'], x, y, yaw)")
ROBOTS=()
for line in "${ROBOT_LINES[@]}"; do ROBOTS+=("${line%% *}"); done
[[ ${#ROBOTS[@]} -gt 0 ]] || { say "no robots found in $SCENARIO"; exit 1; }

# 1. Gazebo
say "Gazebo  ($WORLD_FILE ${GZ_ARGS[*]})"
say "GZ_IP=$GZ_IP; logs: $LOG_DIR"
start_bg "$LOG_DIR/gazebo.log" gz sim -v 3 "${GZ_ARGS[@]}" "$WORLD_FILE"
wait_for 60 "Gazebo to load the world" world_up
if [[ " ${GZ_ARGS[*]} " != *" -s "* ]]; then
  say "waiting for the Gazebo window to connect before spawning (otherwise it never shows the robots)"
  wait_for 90 "the Gazebo window to connect" gui_listening
  sleep 2
fi

# 2. One TurtleBot3 per robot in the scenario. The first one also bridges /clock (exactly one
#    clock bridge: two of them make Nav2 see time jump back).
clock=true
for line in "${ROBOT_LINES[@]}"; do
  read -r id x y yaw <<< "$line"
  say "spawn $id at ($x, $y)"
  start_bg "$LOG_DIR/spawn_$id.log" ros2 launch launch/spawn_robot.launch.py \
    namespace:="$id" robot_name:="$id" x_pose:="$x" y_pose:="$y" yaw:="$yaw" bridge_clock:="$clock"
  clock=false
done
wait_for 90 "robots ${ROBOTS[*]} to appear in Gazebo" robots_up
say "robots are in Gazebo: ${ROBOTS[*]}"

# 3. Switch manager + world state. Its messages appear in this terminal and in the log.
say "switch manager: it prints when a switch, a door, the known rooms or the tasks change."
setsid bash -c "$PY scripts/switch_manager.py '$SCENARIO' 2>&1 | tee '$LOG_DIR/switch_manager.log'" &
MANAGER=$!
PIDS+=($MANAGER)

# 4. Nav2 for every robot, then the executor
if ((NAV)); then
  say "Nav2 for ${ROBOTS[*]} (log: $LOG_DIR/navigation.log)"
  start_bg "$LOG_DIR/navigation.log" ros2 launch launch/navigation.launch.py scenario:="$SCENARIO" \
    params_dir:="$LOG_DIR/nav2_params"
  wait_for 120 "Nav2 to become active for ${ROBOTS[*]}" nav_up
  say "Nav2 is active for ${ROBOTS[*]}"
  start_shown "$LOG_DIR/executor.log" "[executor] " "$PY scripts/executor.py '$SCENARIO'"
  # /team/plan is volatile: start the planner only once the executor listens, so the first
  # plan isn't lost.
  wait_for 30 "the executor to start" executor_up
fi

# 5. The planner
case "$PLANNER" in
  conventional)
    say "planner: baseline (rule-based) planner, scripts/baseline_planner.py"
    start_shown "$LOG_DIR/planner.log" "[planner] " "$PY scripts/baseline_planner.py" ;;
  smart_llm)
    say "planner: SMART-LLM (Gemini), smart_llm_planner/"
    start_shown "$LOG_DIR/planner.log" "[planner] " \
      "PYTHONPATH='$PWD/smart_llm_planner'\${PYTHONPATH:+:\$PYTHONPATH} $PY -m smart_llm_planner.planner_node \
       --ros-args --params-file smart_llm_planner/config/smart_llm_planner.yaml -p use_sim_time:=true" ;;
esac

say "Ctrl+C stops everything. Second terminal:  source scripts/test_helpers.sh"
((NAV)) && [[ "$PLANNER" == none ]] && say "no planner: send plans by hand, e.g.  send_plan robot1=HOLD:S1 robot2=CROSS:D1"
# The script waits on the manager, so Ctrl+C (or closing the terminal) always cleans up.
wait "$MANAGER"

# Only reached if the manager stopped on its own (for example, an error in a rule function).
say "switch manager stopped. Gazebo and the robots are still running."
say "Fix the error, then restart only the manager in another terminal:"
say "    python3 scripts/switch_manager.py $SCENARIO"
say "Press Ctrl+C here to stop everything."
wait "${PIDS[0]}"   # keeps running until Ctrl+C or the Gazebo window is closed

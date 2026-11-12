# test_helpers.sh: test commands for the running simulation (see README, "Testing").
# SOURCE it (don't run it), after ./scripts/run.sh is up in another terminal:
#     source scripts/test_helpers.sh
# Then type one of:
#     test_a          one switch + the color check (README Test A)
#     test_b          the hold-and-handoff         (README Test B)
#     reset_robots    put both robots back on their start circles
#     setpose <robot> <x> <y>   teleport one robot, e.g.  setpose robot_red -1.1 -1.45
#     drive_red / drive_blue    drive a robot with the keyboard (Ctrl+C to stop)
# Watch the run.sh terminal and the gate in Gazebo while these run.

source /opt/ros/jazzy/setup.bash

setpose() {   # teleport: setpose <robot> <x> <y>
  gz service -s /world/escape_maze/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 2000 --req "name: \"$1\", position: {x: $2, y: $3, z: 0.01}" | grep -q "data: true" \
    || echo "  setpose $1 failed (wrong robot name, or Gazebo not running?)"
}

step() {      # step <message> <robot> <x> <y>: print, teleport, pause so you can watch
  echo "-> $1"
  setpose "$2" "$3" "$4"
  sleep 3
}

reset_robots() {
  setpose robot_red  -4.8 -0.65
  setpose robot_blue -4.8  0.65
  echo "robots back on their start circles"
}

test_a() {
  echo "== Test A: one switch, and the color check =="
  reset_robots; sleep 2
  step "red onto the red pad: expect S_R ON, D1 OPEN"                 robot_red  -1.1 -1.45
  step "red steps off: expect S_R OFF, D1 CLOSED"                     robot_red  -3.0 -1.45
  step "blue onto the RED pad: expect nothing (wrong color)"          robot_blue -1.1 -1.45
  step "blue back to start: expect nothing"                           robot_blue -4.8  0.65
  echo "== Test A done =="
}

test_b() {
  echo "== Test B: hold-and-handoff =="
  reset_robots; sleep 2
  step "red holds S_R: expect S_R ON, D1 OPEN"                        robot_red  -1.1 -1.45
  step "blue holds S_B (far side): expect S_B ON"                     robot_blue  1.1  1.45
  step "red leaves S_R and crosses: expect S_R OFF and NO door line"  robot_red   3.0 -1.0
  step "blue leaves S_B: expect S_B OFF, D1 CLOSED"                   robot_blue  4.0  1.0
  echo "== Test B done =="
}

drive_red()  { ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_red/cmd_vel; }
drive_blue() { ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_blue/cmd_vel; }

echo "test helpers loaded: test_a, test_b, reset_robots, setpose <robot> <x> <y>, drive_red, drive_blue"

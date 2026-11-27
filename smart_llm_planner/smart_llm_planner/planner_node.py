"""ROS 2 node: world state in -> SMART-LLM -> one task per robot out.

    /team/world_state  (std_msgs/String, planner-input JSON)  ->  planner_node
    planner_node  ->  /team/plan         (std_msgs/String, {"robot1": "HOLD:S1", ...})
    planner_node  ->  /team/planner_log  (std_msgs/String, one JSON record per planning call)

Replanning loop (simple 1-to-1 version): a plan maps every robot to exactly one task.
A new plan is made when the world state shows something new (room, switch, door or task),
a switch or door changes, a robot's status changes, or a robot finishes its task; and every
`replan_period_s` as a safety net. Only one plan is computed at a time, in a worker thread,
so the robots keep moving while the LLM thinks. Triggers that arrive during planning are
merged into one follow-up plan on the newest state. If no valid plan comes back, nothing is
published and the robots keep their previous tasks.

Run:
    export GEMINI_API_KEY=<your key>
    ros2 launch smart_llm_planner smart_llm_planner.launch.py
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from smart_llm_planner.llm_client import API_KEY_PLACEHOLDER, LLMClient
from smart_llm_planner.replan import replan_triggers
from smart_llm_planner.smart_llm import DEFAULT_INSTRUCTION, SmartLLM


class PlannerNode(Node):
    def __init__(self):
        super().__init__('smart_llm_planner')
        p = {
            'model': 'gemini-2.5-flash',
            'api_key': API_KEY_PLACEHOLDER,
            'temperature': 0.0,
            'request_timeout_s': 60.0,
            'instruction': DEFAULT_INSTRUCTION,
            'replan_period_s': 20.0,
            'world_state_topic': '/team/world_state',
            'plan_topic': '/team/plan',
            'log_topic': '/team/planner_log',
        }
        for name, default in p.items():
            p[name] = self.declare_parameter(name, default).value

        self.planner = SmartLLM(
            LLMClient(p['model'], p['api_key'], p['temperature'], p['request_timeout_s']),
            p['instruction'])
        self.replan_period_s = p['replan_period_s']

        self.plan_pub = self.create_publisher(String, p['plan_topic'], 10)
        self.log_pub = self.create_publisher(String, p['log_topic'], 10)
        # Transient local: the world-state publisher keeps its latest message, so a planner
        # started after it still receives the current state at once.
        world_state_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, p['world_state_topic'], self.on_world_state, world_state_qos)
        self.create_timer(1.0, self.on_timer)

        self._lock = threading.Lock()
        self._state = None
        self._pending = []            # triggers not yet planned for
        self._busy = False
        self._last_plan_end = time.monotonic()
        self.get_logger().info(
            f'SMART-LLM planner up; model={p["model"]}; waiting for {p["world_state_topic"]}')

    # --------------------------------------------------------------- triggers --

    def on_world_state(self, msg):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'ignoring world state that is not JSON: {e}')
            return
        with self._lock:
            triggers = replan_triggers(self._state, state)
            self._state = state
        if triggers:
            self.request_plan(triggers)

    def on_timer(self):
        with self._lock:
            due = (self._state is not None and not self._busy
                   and time.monotonic() - self._last_plan_end >= self.replan_period_s)
        if due:
            self.request_plan([f'periodic ({self.replan_period_s:.0f} s)'])

    def request_plan(self, triggers):
        with self._lock:
            self._pending.extend(triggers)
            if self._busy:
                return
            self._busy = True
        threading.Thread(target=self._worker, daemon=True).start()

    # ---------------------------------------------------------------- planning --

    def _worker(self):
        while True:
            with self._lock:
                if not self._pending:
                    self._busy = False
                    self._last_plan_end = time.monotonic()
                    return
                triggers, self._pending = self._pending, []
                state = self._state
            self._plan_once(state, triggers)

    def _plan_once(self, state, triggers):
        log = self.get_logger()
        log.info(f'planning; triggers: {triggers}')
        record = {'event': 'plan', 'stamp': self.get_clock().now().nanoseconds * 1e-9,
                  'triggers': triggers}
        t0 = time.monotonic()
        try:
            result = self.planner.plan(state)
        except Exception as e:  # API/network failure: keep the previous tasks
            record.update(valid=False, planning_time_s=time.monotonic() - t0,
                          errors=[f'{type(e).__name__}: {e}'])
            log.error(f'planning failed, robots keep their tasks: {e}')
            self.log_pub.publish(String(data=json.dumps(record)))
            return

        record.update(
            valid=result.valid, attempts=result.attempts, errors=result.errors,
            planning_time_s=time.monotonic() - t0, stage_times_s=result.stage_times_s,
            assignments=result.assignments, reasons=result.reasons,
            decomposition=result.decomposition, coalition=result.coalition,
            raw_allocations=result.raw_allocations)
        if result.valid:
            self.plan_pub.publish(String(data=json.dumps(result.assignments)))
            log.info(f'plan ({record["planning_time_s"]:.1f} s, {result.attempts} attempt(s)): '
                     f'{result.assignments}')
        else:
            log.warn(f'invalid plan after {result.attempts} attempts, robots keep their tasks: '
                     f'{result.errors}')
        self.log_pub.publish(String(data=json.dumps(record)))


def main():
    rclpy.init()
    try:
        node = PlannerNode()
    except RuntimeError as e:     # e.g. no API key
        print(f'[smart_llm_planner] {e}')
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()

"""SMART-LLM (Kannan et al., IROS 2024) for the escape maze. No ROS in this file.

The three LLM stages of the paper, each a few-shot Pythonic prompt:
    1. Task decomposition   LLM_DEC(I, E, O, skills)  -> sub-tasks + order      (prompts/train_task_decompose.py)
    2. Coalition formation  LLM_COL(T, robots)        -> coalition policy       (prompts/train_task_coalition.py)
    3. Task allocation      LLM_ALO(T, P, robots)     -> {robot: task} JSON     (prompts/train_task_allocation.py)
Stage 4 (execution) is the executor node, which receives the allocation on /team/plan.

Changes from the paper for replanning in an unknown maze:
  - The environment (E, O) is the current world state, not a fixed AI2-THOR scene.
  - Stage 3 outputs one task per robot for *now* (JSON) instead of threaded Python code;
    the whole pipeline is re-run whenever the world state changes.
  - The allocation goes through the plan validator; on rejection the errors are sent
    back to the model for one retry (proposal section 3.2).

Offline use (no ROS):
    python3 -m smart_llm_planner.smart_llm config/sample_world_state.json --prompts-only
    python3 -m smart_llm_planner.smart_llm config/sample_world_state.json   # needs GEMINI_API_KEY
"""
import argparse
import json
import pprint
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from smart_llm_planner.validator import validate_plan

PROMPT_DIR = Path(__file__).parent / 'prompts'
DEFAULT_INSTRUCTION = 'All robots must reach the exit.'
SYSTEM_PROMPT = ('You are SMART-LLM, the task planner of a team of ground robots escaping a maze. '
                 'Continue the Python-style prompt in exactly the format of the examples.')
CURRENT = '# ' + '=' * 62 + ' CURRENT TASK =='


@dataclass
class PlanResult:
    assignments: dict = None          # {"robot1": "HOLD:S1", ...}; None if no valid plan
    reasons: dict = field(default_factory=dict)
    attempts: int = 0
    errors: list = field(default_factory=list)          # validator errors of the last attempt
    stage_times_s: dict = field(default_factory=dict)
    decomposition: str = ''
    coalition: str = ''
    raw_allocations: list = field(default_factory=list)

    @property
    def valid(self):
        return self.assignments is not None


def _load(name):
    return (PROMPT_DIR / name).read_text()


def _py(name, value):
    return f'{name} = {pprint.pformat(value, width=110, sort_dicts=False)}'


def _drop_none(d):
    return {k: v for k, v in d.items() if v is not None}


def _robots_for_prompt(state):
    return [{**r, 'status': _drop_none(r.get('status') or {'type': 'ok'})} for r in state.get('robots', [])]


def _strip_fences(text):
    return re.sub(r'^```[a-zA-Z]*\s*|\s*```$', '', text.strip())


def _with_heading(text, heading):
    text = _strip_fences(text)
    return text if text.startswith(heading) else f'{heading}\n{text}'


def parse_allocation(text):
    """Return (assignments, reasons) from the model's JSON, or raise ValueError."""
    try:
        data = json.loads(_strip_fences(text))
    except json.JSONDecodeError as e:
        raise ValueError(f'the output is not valid JSON ({e})') from None
    if not isinstance(data, dict):
        raise ValueError('the output must be a JSON object')
    assignments = data.get('assignments', data)
    reasons = data.get('reasons', {}) if 'assignments' in data else {}
    if not isinstance(assignments, dict) or not all(
            isinstance(k, str) and isinstance(v, str) for k, v in assignments.items()):
        raise ValueError('"assignments" must map every robot id to one task id string')
    return assignments, reasons if isinstance(reasons, dict) else {}


class SmartLLM:
    def __init__(self, client, instruction=DEFAULT_INSTRUCTION):
        self.client = client
        self.instruction = instruction
        self.skills = _load('skills.py')
        self.decompose_examples = _load('train_task_decompose.py')
        self.coalition_examples = _load('train_task_coalition.py')
        self.allocation_examples = _load('train_task_allocation.py')

    # ---------------------------------------------------------------- prompts --

    def decomposition_prompt(self, state):
        tasks = [_drop_none(t) for t in state.get('tasks', [])]
        return '\n'.join([
            'import threading', self.skills, self.decompose_examples, CURRENT,
            f'# Task Description: {self.instruction}',
            _py('rooms', state.get('rooms', [])),
            _py('switches', state.get('switches', [])),
            _py('doors', state.get('doors', [])),
            _py('tasks', tasks),
            '', '# GENERAL TASK DECOMPOSITION',
        ])

    def coalition_prompt(self, state, decomposition):
        return '\n'.join([
            self.skills, self.coalition_examples, CURRENT,
            _py('robots', _robots_for_prompt(state)),
            _py('costs', state.get('costs', {})),
            decomposition,
            '', '# SOLUTION',
        ])

    def allocation_prompt(self, state, coalition):
        return '\n'.join([
            self.allocation_examples, CURRENT,
            _py('robots', [r['id'] for r in state.get('robots', [])]),
            _py('tasks', [t['id'] for t in state.get('tasks', [])]),
            coalition,
            '', '# ALLOCATION',
        ])

    # --------------------------------------------------------------- pipeline --

    def plan(self, state):
        result = PlanResult()

        t = time.monotonic()
        result.decomposition = _with_heading(
            self.client.chat([('user', self.decomposition_prompt(state))], system=SYSTEM_PROMPT),
            '# GENERAL TASK DECOMPOSITION')
        result.stage_times_s['decomposition'] = time.monotonic() - t

        t = time.monotonic()
        result.coalition = _with_heading(
            self.client.chat([('user', self.coalition_prompt(state, result.decomposition))],
                             system=SYSTEM_PROMPT),
            '# SOLUTION')
        result.stage_times_s['coalition'] = time.monotonic() - t

        t = time.monotonic()
        messages = [('user', self.allocation_prompt(state, result.coalition))]
        for attempt in (1, 2):                       # first try + one retry (proposal 3.2)
            result.attempts = attempt
            text = self.client.chat(messages, system=SYSTEM_PROMPT, json_output=True)
            result.raw_allocations.append(text)
            try:
                assignments, reasons = parse_allocation(text)
                result.errors = validate_plan(assignments, state)
            except ValueError as e:
                result.errors = [str(e)]
            if not result.errors:
                result.assignments, result.reasons = assignments, reasons
                break
            messages += [('model', text), ('user', '# The plan validator rejected this ALLOCATION:\n'
                          + '\n'.join(f'# - {e}' for e in result.errors)
                          + '\n# Fix every problem and output the corrected ALLOCATION JSON only.')]
        result.stage_times_s['allocation'] = time.monotonic() - t
        return result


def main(argv=None):
    ap = argparse.ArgumentParser(description='Run SMART-LLM once on a world-state JSON file.')
    ap.add_argument('state', help='planner-input JSON (see docs/communication_standards.md)')
    ap.add_argument('--instruction', default=DEFAULT_INSTRUCTION)
    ap.add_argument('--model', default='gemini-2.5-flash')
    ap.add_argument('--prompts-only', action='store_true',
                    help='print the stage-1 prompt and exit; no API call')
    args = ap.parse_args(argv)

    state = json.loads(Path(args.state).read_text())
    if args.prompts_only:
        print(SmartLLM(client=None, instruction=args.instruction).decomposition_prompt(state))
        return 0

    from smart_llm_planner.llm_client import LLMClient
    result = SmartLLM(LLMClient(args.model), args.instruction).plan(state)
    print(result.decomposition, '\n')
    print(result.coalition, '\n')
    print('# ALLOCATION (attempts: %d)' % result.attempts)
    print(json.dumps({'assignments': result.assignments, 'reasons': result.reasons,
                      'errors': result.errors, 'stage_times_s': result.stage_times_s}, indent=2))
    return 0 if result.valid else 1


if __name__ == '__main__':
    sys.exit(main())

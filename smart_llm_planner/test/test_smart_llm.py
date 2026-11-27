import copy
import json
from pathlib import Path

from smart_llm_planner.replan import replan_triggers
from smart_llm_planner.smart_llm import SmartLLM, parse_allocation
from smart_llm_planner.validator import validate_plan

STATE = json.loads((Path(__file__).parents[1] / 'config' / 'sample_world_state.json').read_text())


class ScriptedLLM:
    """Returns canned replies in order and records every call."""

    def __init__(self, *replies):
        self.replies = list(replies)
        self.calls = []

    def chat(self, messages, system=None, json_output=False):
        self.calls.append((messages, json_output))
        return self.replies.pop(0)


# ------------------------------------------------------------------ validator --

def test_valid_hold_and_cross():
    assert validate_plan({'robot1': 'HOLD:S1', 'robot2': 'CROSS:D1'}, STATE) == []


def test_color_mismatch():
    errors = validate_plan({'robot1': 'CROSS:D1', 'robot2': 'HOLD:S1'}, STATE)
    assert any('needs a red robot' in e for e in errors)


def test_cross_closed_door_without_holder():
    errors = validate_plan({'robot1': 'WAIT', 'robot2': 'CROSS:D1'}, STATE)
    assert any('door D1 is closed' in e for e in errors)


def test_missing_robot_unknown_task_duplicate():
    assert any('robot2 has no task' in e for e in validate_plan({'robot1': 'WAIT'}, STATE))
    assert any('unknown task' in e
               for e in validate_plan({'robot1': 'EXIT', 'robot2': 'WAIT'}, STATE))
    assert any('assigned to 2 robots' in e
               for e in validate_plan({'robot1': 'CROSS:D1', 'robot2': 'CROSS:D1'}, STATE))


def test_unreachable():
    state = copy.deepcopy(STATE)
    del state['costs']['robot2']['CROSS:D1']
    errors = validate_plan({'robot1': 'HOLD:S1', 'robot2': 'CROSS:D1'}, state)
    assert any('unreachable' in e for e in errors)


def test_latched_switch_keeps_door_open():
    state = copy.deepcopy(STATE)
    state['switches'].append({'id': 'S2', 'color': 'blue', 'mode': 'latch', 'active': True})
    state['doors'][0]['open_if'] = {'any_of': ['S1', 'S2']}   # S2 is known now, so it is listed
    state['doors'][0]['open'] = True
    assert validate_plan({'robot1': 'CROSS:D1', 'robot2': 'WAIT'}, state) == []


# ------------------------------------------------------------------- triggers --

def test_triggers():
    assert replan_triggers(None, STATE) == ['initial world state']
    assert replan_triggers(STATE, STATE) == []

    assigned = copy.deepcopy(STATE)
    assigned['robots'][0]['task'] = 'HOLD:S1'
    assert replan_triggers(STATE, assigned) == []                  # our own assignment
    assert replan_triggers(assigned, STATE) == ['robot1 finished HOLD:S1']

    opened = copy.deepcopy(STATE)
    opened['doors'][0]['open'] = True
    opened['switches'].append({'id': 'S2', 'color': 'blue', 'mode': 'hold', 'active': False})
    assert replan_triggers(STATE, opened) == ['found switch S2', 'door D1 open -> True']


# ------------------------------------------------------------------- pipeline --

def test_parse_allocation_flat_and_fenced():
    assert parse_allocation('```json\n{"robot1": "WAIT"}\n```') == ({'robot1': 'WAIT'}, {})


def test_three_stages_then_retry_once():
    bad = json.dumps({'assignments': {'robot1': 'CROSS:D1', 'robot2': 'HOLD:S1'}})
    good = json.dumps({'assignments': {'robot1': 'HOLD:S1', 'robot2': 'CROSS:D1'},
                       'reasons': {'robot1': 'red', 'robot2': 'crosses'}})
    llm = ScriptedLLM('# Independent subtasks: ...', '# For SubTask 1 assign robot1.', bad, good)
    result = SmartLLM(llm).plan(STATE)

    assert result.valid and result.attempts == 2
    assert result.assignments == {'robot1': 'HOLD:S1', 'robot2': 'CROSS:D1'}
    assert result.decomposition.startswith('# GENERAL TASK DECOMPOSITION')
    assert '# For SubTask 1 assign robot1.' in llm.calls[2][0][0][1]       # stage 2 -> stage 3
    retry_messages = llm.calls[3][0]
    assert [role for role, _ in retry_messages] == ['user', 'model', 'user']
    assert 'needs a red robot' in retry_messages[-1][1]


def test_two_failures_give_no_plan():
    llm = ScriptedLLM('dec', 'col', 'not json', '{"robot1": "WAIT"}')
    result = SmartLLM(llm).plan(STATE)
    assert not result.valid and result.attempts == 2
    assert any('robot2 has no task' in e for e in result.errors)

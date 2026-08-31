from __future__ import annotations


CONSUMED_COMMAND_STATE = -1


def command_states_compatible(
    command_states,
    required_state: int,
) -> bool:
    """接受目标状态或控制器已消费命令后的 -1 哨兵值。"""
    try:
        states = tuple(int(value) for value in command_states)
    except (TypeError, ValueError):
        return False
    if len(states) != 2:
        return False
    allowed = {int(required_state), CONSUMED_COMMAND_STATE}
    return all(state in allowed for state in states)

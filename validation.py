"""Independent path validation for public and instructor tests."""
from sokoban import SokobanState, sokoban_goal_state


def signature(state):
    return (state.width, state.height, state.robots, state.boxes,
            state.storage, state.obstacles)


def validate_solution(initial, final):
    """Return (valid, message); replay the parent chain, not just the goal/cost."""
    if not isinstance(final, SokobanState):
        return False, "Expected a SokobanState solution"
    chain, seen = [], set()
    current = final
    while current is not None:
        if not isinstance(current, SokobanState) or id(current) in seen:
            return False, "Invalid or cyclic parent chain"
        seen.add(id(current))
        chain.append(current)
        current = current.parent
    chain.reverse()
    if signature(chain[0]) != signature(initial) or chain[0].gval != initial.gval:
        return False, "Path does not start at the supplied initial state"
    for previous, following in zip(chain, chain[1:]):
        if following.gval != previous.gval + 1:
            return False, "Incorrect path cost"
        if not any(candidate.action == following.action and
                   signature(candidate) == signature(following)
                   for candidate in previous.successors()):
            return False, "Illegal transition"
    if not sokoban_goal_state(final):
        return False, "Final state is not a goal"
    return True, "Valid solution"

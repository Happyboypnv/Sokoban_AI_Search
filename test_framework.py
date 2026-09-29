"""Tests of supplied game mechanics only; no student algorithms needed."""
import unittest
from sokoban import SokobanState, sokoban_goal_state, PROBLEMS
from validation import validate_solution, signature


def board(robots=((0, 1),), boxes=((1, 1),), storage=((2, 1),), obstacles=()):
    return SokobanState("START", 0, None, 4, 3, robots, frozenset(boxes),
                        frozenset(storage), frozenset(obstacles))


class FrameworkTests(unittest.TestCase):
    def test_push_and_cost(self):
        start = board()
        before = signature(start)
        goal = next(s for s in start.successors() if s.action == "0 right")
        self.assertTrue(sokoban_goal_state(goal))
        self.assertEqual(goal.gval, 1)
        self.assertIs(goal.parent, start)
        self.assertEqual(signature(start), before)
        self.assertTrue(validate_solution(start, goal)[0])

    def test_no_double_push(self):
        self.assertNotIn("0 right", [s.action for s in board(boxes=((1, 1), (2, 1))).successors()])

    def test_robot_blocks_push(self):
        start = board(robots=((0, 1), (2, 1)))
        self.assertNotIn("0 right", [s.action for s in start.successors()])

    def test_obstacle_blocks_push(self):
        self.assertNotIn("0 right", [s.action for s in board(obstacles=((2, 1),)).successors()])

    def test_multiple_robots_move_one_at_a_time(self):
        start = board(robots=((0, 1), (3, 2)))
        for successor in start.successors():
            self.assertEqual(sum(a != b for a, b in zip(start.robots, successor.robots)), 1)
            self.assertEqual(successor.gval, 1)

    def test_state_key_and_storage(self):
        start = board(boxes=((2, 1),), storage=((2, 1), (3, 2)))
        self.assertIsInstance(start.hashable_state(), tuple)
        self.assertTrue(sokoban_goal_state(start))
        self.assertEqual(len(PROBLEMS), 20)

    def test_validator_rejects_forgery(self):
        initial, forged = board(), board(boxes=((2, 1),))
        self.assertFalse(validate_solution(initial, forged)[0])
        forged.parent, forged.gval, forged.action = initial, 1, "illegal"
        self.assertFalse(validate_solution(initial, forged)[0])
        forged.parent = forged
        self.assertFalse(validate_solution(initial, forged)[0])


if __name__ == "__main__":
    unittest.main()

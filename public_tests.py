"""Small API examples only; no prescribed search algorithm."""
import unittest
import solution
from sokoban import SokobanState
from validation import signature, validate_solution


def simple_state(solved=False):
    return SokobanState(
        "START", 0, None, 4, 3, ((0, 1),),
        frozenset([(2, 1) if solved else (1, 1)]),
        frozenset([(2, 1)]), frozenset())


class PublicTests(unittest.TestCase):
    def test_one_box(self):
        initial = simple_state()
        before = (signature(initial), initial.action, initial.gval, initial.parent)
        final = solution.solve(initial, timebound=120)
        self.assertEqual(
            (signature(initial), initial.action, initial.gval, initial.parent), before)
        valid, detail = validate_solution(simple_state(), final)
        self.assertTrue(valid, detail)

    def test_initial_goal(self):
        initial = simple_state(solved=True)
        final = solution.solve(initial, timebound=120)
        valid, detail = validate_solution(simple_state(solved=True), final)
        self.assertTrue(valid, detail)

    def test_zero_budget(self):
        self.assertIs(solution.solve(simple_state(), timebound=0), False)


if __name__ == "__main__":
    unittest.main()

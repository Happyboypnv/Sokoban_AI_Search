"""State interface only. No search algorithm is provided."""


class StateSpace:
    n = 0

    def __init__(self, action, gval, parent):
        self.action = action
        self.gval = gval
        self.parent = parent
        self.index = StateSpace.n
        StateSpace.n += 1

    def successors(self):
        """Return states with cumulative gval, parent=self and action filled in."""
        raise NotImplementedError

    def hashable_state(self):
        """Return an immutable key identifying the configuration within this problem."""
        raise NotImplementedError

    def print_state(self):
        raise NotImplementedError

    def print_path(self):
        path = []
        current = self
        while current is not None:
            path.append(current)
            current = current.parent
        for item in reversed(path):
            item.print_state()

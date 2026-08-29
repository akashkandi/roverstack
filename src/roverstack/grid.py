"""Basic grid position used by the rest of the project."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Point:
    """A single (x, y) location on our grid."""

    x: int
    y: int

    def manhattan_distance(self, other: "Point") -> int:
        """How many grid steps away 'other' is, moving only up/down/left/right."""
        return abs(self.x - other.x) + abs(self.y - other.y)

    def neighbors(self):
        """The 4 adjacent cells (no diagonals — keeps the planner simple)."""
        return [
            Point(self.x + 1, self.y),
            Point(self.x - 1, self.y),
            Point(self.x, self.y + 1),
            Point(self.x, self.y - 1),
        ]


@dataclass(frozen=True)
class GridBounds:
    """The size of the world grid: valid x is [0, width), valid y is [0, height)."""

    width: int
    height: int

    def contains(self, point: "Point") -> bool:
        return 0 <= point.x < self.width and 0 <= point.y < self.height

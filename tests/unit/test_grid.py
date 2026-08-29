from roverstack.grid import GridBounds, Point


def test_manhattan_distance_between_two_points():
    a = Point(0, 0)
    b = Point(3, 4)
    assert a.manhattan_distance(b) == 7


def test_manhattan_distance_to_self_is_zero():
    a = Point(2, 2)
    assert a.manhattan_distance(a) == 0


def test_neighbors_returns_four_adjacent_cells():
    p = Point(2, 2)
    assert set(p.neighbors()) == {Point(3, 2), Point(1, 2), Point(2, 3), Point(2, 1)}


def test_bounds_contains_checks_the_edges_correctly():
    bounds = GridBounds(width=5, height=5)
    assert bounds.contains(Point(0, 0))
    assert bounds.contains(Point(4, 4))
    assert not bounds.contains(Point(5, 0))
    assert not bounds.contains(Point(-1, 0))

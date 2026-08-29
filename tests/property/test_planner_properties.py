"""Property-based tests for the A* planner.

Instead of hand-picking a handful of grids, Hypothesis generates hundreds of
random grids/obstacle layouts/start-goal pairs and checks that certain rules
ALWAYS hold — exactly the kind of testing real path-planning teams rely on to
catch an edge case a human wouldn't think to write by hand.
"""

from hypothesis import given, settings
from hypothesis import strategies as st

from roverstack.grid import GridBounds, Point
from roverstack.perception import ObstacleMap
from roverstack.planner import PathNotFoundError, a_star

GRID_SIZE = 8


@st.composite
def grid_scenario(draw):
    """A recipe for building one random (but valid) test case: a grid, some
    random obstacles, and a start/goal pulled from the cells that aren't
    blocked."""
    bounds = GridBounds(width=GRID_SIZE, height=GRID_SIZE)
    all_points = [Point(x, y) for x in range(GRID_SIZE) for y in range(GRID_SIZE)]

    obstacles = frozenset(
        draw(
            st.sets(
                st.builds(
                    Point, st.integers(0, GRID_SIZE - 1), st.integers(0, GRID_SIZE - 1)
                ),
                max_size=GRID_SIZE * GRID_SIZE // 3,
            )
        )
    )
    free_points = [p for p in all_points if p not in obstacles]

    start, goal = draw(
        st.tuples(st.sampled_from(free_points), st.sampled_from(free_points)).filter(
            lambda pair: pair[0] != pair[1]
        )
    )
    return bounds, ObstacleMap(obstacles=obstacles), start, goal


@given(scenario=grid_scenario())
@settings(max_examples=100)
def test_a_star_path_is_always_valid_when_one_exists(scenario):
    bounds, obstacle_map, start, goal = scenario

    try:
        path = a_star(start, goal, bounds, obstacle_map)
    except PathNotFoundError:
        return  # no path exists in this random layout — nothing to check here

    assert path[0] == start
    assert path[-1] == goal

    for i, point in enumerate(path):
        assert bounds.contains(point)
        assert not obstacle_map.is_occupied(point)
        if i > 0:
            assert point in path[i - 1].neighbors()

    # A shortest path can never be shorter than the straight-line distance.
    assert len(path) >= start.manhattan_distance(goal) + 1

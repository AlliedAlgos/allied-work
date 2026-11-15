import pytest

from pc.simulator import SimConfig, SimWorld, reset_robot_modules


@pytest.fixture
def make_world():
    """Factory: make_world(**SimConfig fields) -> installed SimWorld."""
    worlds = []

    def make(**kw):
        reset_robot_modules()
        w = SimWorld(SimConfig(**kw)).install()
        worlds.append(w)
        return w

    yield make
    for w in worlds:
        w.uninstall()
    reset_robot_modules()

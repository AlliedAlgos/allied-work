"""The hub menu: arrows choose a run, center starts it."""
import asyncio
import os
import sys
import tempfile

import pytest

import run as runpy  # repo-root run.py


class Done(BaseException):
    pass


def press_script(world, script, end_ms):
    """script: [(start_ms, button_name)]; each press lasts 150 ms. Ends the test at end_ms."""
    from pybricks.parameters import Button

    def hook(dt):
        t = world.t_ms
        if t > end_ms:
            raise Done()
        world.pressed = {getattr(Button, b) for s, b in script if s <= t < s + 150}
    world.hooks.append(hook)


def track_display(world):
    shown = []

    def hook(dt):
        if not shown or shown[-1] != world.display_text:
            shown.append(world.display_text)
    world.hooks.append(hook)
    return shown


def test_arrows_select_and_center_starts(make_world):
    w = make_world()
    import alliedwork as aw
    ran = []
    missions = [(n, "run%d" % n, (lambda n=n: ran.append(n))) for n in (1, 2, 3)]
    shown = track_display(w)
    press_script(w, [(200, "RIGHT"), (600, "RIGHT"), (1000, "LEFT"), (1400, "CENTER"),
                     (4000, "LEFT"), (4400, "LEFT"), (4800, "CENTER")], end_ms=7500)
    with pytest.raises(Done):
        aw.run_menu(missions)
    assert shown == [1, 2, 3, 2, 1, 3]   # right, right, left | after run 2: left, left (wraps)
    assert ran == [2, 3]


def test_menu_orders_by_number_and_survives_a_crashing_mission(make_world, capsys):
    w = make_world()
    import alliedwork as aw
    ran = []

    def boom():
        raise RuntimeError("oops")
    shown = track_display(w)
    press_script(w, [(200, "CENTER"), (4000, "RIGHT"), (4500, "CENTER")], end_ms=8000)
    with pytest.raises(Done):
        aw.run_menu([(5, "run5", lambda: ran.append(5)), (2, "run2", boom)])
    assert shown[0] == 2                      # lowest number first
    assert ran == [5]                         # menu kept working after run2 crashed
    assert "ERROR,run2: oops" in capsys.readouterr().out


def test_empty_menu_message(make_world):
    make_world()
    import alliedwork as aw
    with pytest.raises(ValueError, match="no missions found"):
        aw.run_menu([])


def test_generated_main_runs_menu_in_sim(make_world):
    w = make_world(start_x_in=30, start_y_in=8)
    shown = track_display(w)
    with tempfile.TemporaryDirectory() as tmp:
        main = runpy.make_bundle(tmp)
        src = open(main).read()
        assert "run_menu([(1," in src and "(2," in src and "test" not in src
        sys.path.insert(0, tmp)
        try:
            press_script(w, [(200, "RIGHT"), (600, "CENTER")], end_ms=20000)  # run 2
            with pytest.raises(Done):
                exec(compile(src, main, "exec"), {"__name__": "__main__"})
        finally:
            sys.path.remove(tmp)
            for m in ("run1", "run2", "test"):
                sys.modules.pop(m, None)
    assert shown[:2] == [1, 2]
    assert abs(w.y - 8) < 0.5 and abs(w.x - 30) < 0.5   # run2 came back to its start


def test_single_mission_main():
    assert runpy.build_main("run1") == "import run1\n\nrun1.run()\n"


def test_pybricksdev_bundles_all_imports():
    """The compile step the hub upload uses must find alliedwork + every run."""
    pytest.importorskip("pybricksdev")
    from pybricksdev.compile import compile_multi_file
    with tempfile.TemporaryDirectory() as tmp:
        main = runpy.make_bundle(tmp)
        try:
            blob = asyncio.run(compile_multi_file(main, (6, 1)))
        except Exception as e:  # mpy-cross not available on this machine
            pytest.skip("mpy-cross unavailable: %r" % (e,))
    for name in (b"alliedwork", b"run1", b"run2"):
        assert name in blob

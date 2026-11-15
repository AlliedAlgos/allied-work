"""Put missions on the robot, or try them in the simulator.

    python run.py                 upload ALL runs + the menu to the hub and start it
    python run.py --no-start      upload only; start later with the hub's center button
    python run.py run1            upload and run just hardware/missions/run1.py
    python run.py run1 --sim      try it on the fake robot (no hub needed)
    python run.py list            list the missions
    python run.py work            open the live dashboard (add --sim for a fake robot)
    python run.py docs            open the documentation website (http://localhost:8001)
    python run.py changelog       same website, opened on the changelog
    python run.py docs --build    build a static copy of the site into site/ (works offline)
    python run.py -n NAME ...     pick a hub by Bluetooth name

On the hub: the light matrix shows a run number. LEFT / RIGHT buttons change it,
the CENTER button starts it. CENTER during a run stops the robot.
Missions are the files named run1.py, run2.py ... in hardware/missions/.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import webbrowser

ROOT = os.path.dirname(os.path.abspath(__file__))
HARDWARE = os.path.join(ROOT, "hardware")
MISSIONS = os.path.join(HARDWARE, "missions")


def all_missions():
    """Every mission file (module names), e.g. ['run1', 'run2', 'test']."""
    return sorted(f[:-3] for f in os.listdir(MISSIONS) if f.endswith(".py") and not f.startswith("_"))


def menu_missions():
    """(number, module name) for files named runN.py, in number order."""
    found = []
    for name in all_missions():
        m = re.fullmatch(r"run(\d+)", name)
        if m:
            found.append((int(m.group(1)), name))
    return sorted(found)


def build_main(mission=None):
    """Source of the generated main.py: a menu of all runs, or one mission."""
    if mission:
        return "import %s\n\n%s.run()\n" % (mission, mission)
    runs = menu_missions()
    if not runs:
        raise SystemExit("No runs found. Create hardware/missions/run1.py (copy _template.py).")
    lines = ["from alliedwork import run_menu"]
    lines += ["import %s" % name for _, name in runs]
    items = ", ".join('(%d, "%s", %s.run)' % (n, name, name) for n, name in runs)
    lines += ["", "run_menu([%s])" % items, ""]
    return "\n".join(lines)


def make_bundle(folder, mission=None):
    """Copy alliedwork + missions into folder and write main.py. Returns main.py path."""
    shutil.copy(os.path.join(HARDWARE, "alliedwork.py"), folder)
    for name in all_missions():
        shutil.copy(os.path.join(MISSIONS, name + ".py"), folder)
    main = os.path.join(folder, "main.py")
    with open(main, "w") as f:
        f.write(build_main(mission))
    return main


DOCS_PORT = 8001


def docs_site(page, build=False):
    """Serve (or build) the documentation website made from work/docs/ with MkDocs."""
    env = dict(os.environ, NO_MKDOCS_2_WARNING="1")
    base = [sys.executable, "-m", "mkdocs"]
    if subprocess.call(base + ["--version"], cwd=ROOT, env=env,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
        print("The docs website needs MkDocs:  pip install -r requirements.txt")
        print("(Meanwhile the docs are plain files in work/docs/ : open README.md)")
        return 1
    if build:
        code = subprocess.call(base + ["build", "--strict"], cwd=ROOT, env=env)
        if code == 0:
            print("Built site/ : open site/index.html in a browser (no server needed).")
        return code
    url = "http://localhost:%d/%s" % (DOCS_PORT, page)
    threading.Timer(2.0, webbrowser.open, args=(url,)).start()
    print("Docs website:", url, "  (Ctrl+C to stop)")
    try:
        return subprocess.call(base + ["serve", "--dev-addr", "127.0.0.1:%d" % DOCS_PORT],
                               cwd=ROOT, env=env)
    except KeyboardInterrupt:
        return 0


def run_on_hub(mission, name, start):
    # pybricksdev bundles local imports that sit next to main.py, so build a temp folder.
    with tempfile.TemporaryDirectory() as tmp:
        main = make_bundle(tmp, mission)
        cmd = [sys.executable, "-u", "-m", "pybricksdev", "run", "ble"]
        if name:
            cmd += ["-n", name]
        cmd += ["--start" if start else "--no-start", main]
        return subprocess.call(cmd)


def run_in_sim(mission):
    sys.path[:0] = [ROOT, HARDWARE, MISSIONS]
    from work.pc.simulator import SimConfig, SimWorld
    path = os.path.join(MISSIONS, mission + ".py")
    src = open(path).read()
    m = re.search(r"set_pose\(\s*(?:x\s*=\s*)?(-?[\d.]+)\s*,\s*(?:y\s*=\s*)?(-?[\d.]+)"
                  r"(?:\s*,\s*(?:heading\s*=\s*)?(-?[\d.]+))?", src)
    x, y, h = (float(v) if v else 0.0 for v in m.groups()) if m else (12.0, 12.0, 0.0)
    world = SimWorld(SimConfig(start_x_in=x, start_y_in=y, start_heading_deg=h)).install()
    try:
        ns = {"__name__": mission}
        exec(compile(src, path, "exec"), ns)
        ns["run"]()
    finally:
        px, py, ph = world.pose
        print("\nSimulator finished after %.1f s. True pose: x=%.2f y=%.2f heading=%.1f"
              % (world.t_ms / 1000, px, py, ph))
        world.uninstall()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("mission", nargs="?", help="a mission file name without .py, 'list', 'work', 'docs' or 'changelog'")
    ap.add_argument("--sim", action="store_true", help="run on the fake robot")
    ap.add_argument("--no-start", action="store_true", help="upload but don't start")
    ap.add_argument("-n", "--name", help="hub Bluetooth name")
    ap.add_argument("--build", action="store_true", help="with 'docs': build a static site/ folder")
    ap.add_argument("--version", action="store_true", help="show the alliedwork version")
    args = ap.parse_args()
    if args.version:
        text = open(os.path.join(HARDWARE, "alliedwork.py")).read()
        print("alliedwork " + re.search(r'VERSION = "([^"]+)"', text).group(1))
        return 0
    if args.mission == "work":  # the dashboard (localhost:8000)
        cmd = [sys.executable, "-m", "work.pc.server"] + (["--sim"] if args.sim else [])
        if args.name:
            cmd += ["--hub", args.name]
        try:
            return subprocess.call(cmd, cwd=ROOT)
        except KeyboardInterrupt:
            return 0
    if args.mission in ("docs", "changelog"):
        return docs_site("CHANGELOG.html" if args.mission == "changelog" else "", args.build)
    if args.mission == "list":
        print("Menu runs:", ", ".join("%d = %s" % r for r in menu_missions()) or "(none)")
        print("All files:", ", ".join(all_missions()))
        return 0
    if args.mission and args.mission not in all_missions():
        print("No mission called '%s'. Available: %s" % (args.mission, ", ".join(all_missions())))
        return 1
    if args.sim:
        if not args.mission:
            print("Pick a mission to simulate, e.g.  python run.py run1 --sim")
            return 1
        return run_in_sim(args.mission)
    return run_on_hub(args.mission, args.name, not args.no_start)


if __name__ == "__main__":
    sys.exit(main())

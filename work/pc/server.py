"""Dashboard server: python -m work.pc.server [--sim] [--hub NAME] [--port 8000]

Serves work/dashboard/ and pushes live sensor readings to the browser over a
WebSocket. Hub mode runs `pybricksdev run ble` on hardware/sensor_stream.py and
reconnects automatically; --sim runs the same program on a fake robot.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import webbrowser
from typing import Any, Optional

from aiohttp import WSMsgType, web

from work.pc.protocol import parse_line

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DASHBOARD = os.path.join(ROOT, "work", "dashboard")
STREAM_FILE = os.path.join(ROOT, "hardware", "sensor_stream.py")
RETRY_S = 3


class Hub:
    """Holds the latest reading + connection status and fans out to browsers."""

    def __init__(self) -> None:
        self.clients: set[web.WebSocketResponse] = set()
        self.status = {"state": "starting", "message": "Starting..."}
        self.latest: Optional[dict[str, Any]] = None

    async def broadcast(self, msg: dict[str, Any]) -> None:
        text = json.dumps(msg)
        for ws in list(self.clients):
            try:
                await ws.send_str(text)
            except ConnectionError:
                self.clients.discard(ws)

    async def set_status(self, state: str, message: str) -> None:
        self.status = {"state": state, "message": message}
        await self.broadcast({"type": "status", **self.status})

    async def on_line(self, line: str) -> None:
        reading = parse_line(line)  # bad/other lines are skipped, never fatal
        if reading is None:
            return
        self.latest = reading
        await self.broadcast({"type": "reading", **reading})


async def run_hub(hub: Hub, name: Optional[str]) -> None:
    """Connect with pybricksdev, stream stdout, retry forever."""
    while True:
        cmd = [sys.executable, "-u", "-m", "pybricksdev", "run", "ble"]  # -u: no buffering, so readings arrive live
        if name:
            cmd += ["-n", name]
        cmd.append(STREAM_FILE)
        await hub.set_status("connecting", "Looking for the hub... turn it on (press the center button).")
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            assert proc.stdout is not None
            async for raw in proc.stdout:
                line = raw.decode(errors="replace").strip()
                if parse_line(line) is not None and hub.status["state"] != "connected":
                    await hub.set_status("connected", "Hub connected")
                await hub.on_line(line)
            await proc.wait()
        except FileNotFoundError:
            await hub.set_status("error", "pybricksdev is not installed. Run: pip install -r requirements.txt")
        await hub.set_status("disconnected", f"Hub disconnected. Retrying in {RETRY_S} s...")
        await asyncio.sleep(RETRY_S)


async def run_sim(hub: Hub) -> None:
    from work.pc.sim_runner import SimRunner
    loop = asyncio.get_running_loop()
    lines: asyncio.Queue[str] = asyncio.Queue()
    runner = SimRunner(lambda line: loop.call_soon_threadsafe(lines.put_nowait, line))
    runner.start()
    await hub.set_status("connected", "SIMULATOR (no robot)")
    try:
        while True:
            await hub.on_line(await lines.get())
    finally:
        runner.stop()


async def ws_handler(request: web.Request) -> web.WebSocketResponse:
    hub: Hub = request.app["hub"]
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    hub.clients.add(ws)
    await ws.send_str(json.dumps({"type": "status", **hub.status}))
    if hub.latest:
        await ws.send_str(json.dumps({"type": "reading", **hub.latest}))
    async for msg in ws:  # the page sends nothing yet; just keep the socket open
        if msg.type == WSMsgType.ERROR:
            break
    hub.clients.discard(ws)
    return ws


def make_app(sim: bool, hub_name: Optional[str]) -> web.Application:
    app = web.Application()
    hub = Hub()
    app["hub"] = hub

    async def start(app: web.Application) -> None:
        app["task"] = asyncio.create_task(run_sim(hub) if sim else run_hub(hub, hub_name))

    async def stop(app: web.Application) -> None:
        app["task"].cancel()

    app.on_startup.append(start)
    app.on_cleanup.append(stop)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/", lambda r: web.FileResponse(os.path.join(DASHBOARD, "index.html")))
    return app


def main() -> None:
    ap = argparse.ArgumentParser(description="FLL sensor dashboard")
    ap.add_argument("--sim", action="store_true", help="fake robot, no hub needed")
    ap.add_argument("--hub", help="hub Bluetooth name (default: first hub found)")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()
    if not args.no_browser:
        webbrowser.open(f"http://localhost:{args.port}")
    web.run_app(make_app(args.sim, args.hub), host="127.0.0.1", port=args.port)  # local only


if __name__ == "__main__":
    main()

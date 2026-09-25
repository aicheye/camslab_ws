"""HTTP + WebSocket server shared by the Python sim and the ROS 2 bridge.

The browser loads the static page from "/" and opens a WebSocket on "/ws".
All messages are JSON objects with a "type" field. See README.md for the list.
"""

import asyncio
import json
import logging
from pathlib import Path

from aiohttp import WSMsgType, web

STATIC_DIR = Path(__file__).parent / "static"

# Column order of each row in a "samples" message. dist = ||p* - p||, (xr, yr) = p*.
SAMPLE_FIELDS = ["t", "x", "y", "qx", "qy", "v", "w", "dist", "xr", "yr"]

log = logging.getLogger("gtg_webui")


class Backend:
    """What UiServer needs from a backend."""

    def on_connect(self):
        """Return the list of messages a newly connected browser receives."""
        raise NotImplementedError

    async def on_message(self, msg):
        """Handle one message sent by the browser."""
        raise NotImplementedError


def hello_msg(backend, mode, can_set_pose, robot_name=""):
    """mode is "batch" (whole run sent at once) or "live" (streamed). robot_name labels
    the robot in "samples" on the map; empty draws no label."""
    return {
        "type": "hello",
        "backend": backend,
        "mode": mode,
        "can_set_pose": can_set_pose,
        "robot_name": robot_name,
        "fields": SAMPLE_FIELDS,
    }


def status_msg(state, message=""):
    """state is one of "idle", "running", "error"."""
    return {"type": "status", "state": state, "message": message}


def samples_msg(samples, reset=False):
    return {"type": "samples", "reset": reset, "samples": samples}


def fleet_msg(robots, reset=False):
    """Robots other than the one in "samples". robots is {name: [row, ...]} with rows in
    SAMPLE_FIELDS order."""
    return {"type": "fleet", "reset": reset, "robots": robots}


@web.middleware
async def no_cache(request, handler):
    """Browsers revalidate static files on every load, so a rebuilt UI shows up on reload."""
    response = await handler(request)
    if request.path.startswith("/static/"):
        response.headers["Cache-Control"] = "no-cache"
    return response


class UiServer:
    def __init__(self, backend, host="127.0.0.1", port=8000):
        self.backend = backend
        self.host = host
        self.port = port
        self._clients = set()
        self._loop = None

    def run(self):
        """Serve until SIGINT or SIGTERM."""
        app = web.Application(middlewares=[no_cache])
        app.router.add_get("/", self._index)
        app.router.add_get("/ws", self._ws)
        app.router.add_static("/static", STATIC_DIR)
        app.on_startup.append(self._on_startup)
        app.on_shutdown.append(self._on_shutdown)
        log.info("web UI on http://%s:%d", self.host, self.port)
        web.run_app(app, host=self.host, port=self.port, print=None)

    async def broadcast(self, msg):
        data = json.dumps(msg)
        for ws in list(self._clients):
            try:
                await ws.send_str(data)
            except (ConnectionError, RuntimeError):
                self._clients.discard(ws)

    def broadcast_threadsafe(self, msg):
        """Broadcast from a thread other than the asyncio thread."""
        if self._loop is not None and not self._loop.is_closed():
            asyncio.run_coroutine_threadsafe(self.broadcast(msg), self._loop)

    async def _on_startup(self, app):
        self._loop = asyncio.get_running_loop()

    async def _on_shutdown(self, app):
        for ws in list(self._clients):
            await ws.close()

    async def _index(self, request):
        return web.FileResponse(
            STATIC_DIR / "index.html", headers={"Cache-Control": "no-cache"}
        )

    async def _ws(self, request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._clients.add(ws)
        try:
            for msg in self.backend.on_connect():
                await ws.send_json(msg)
            async for frame in ws:
                if frame.type != WSMsgType.TEXT:
                    continue
                try:
                    await self.backend.on_message(json.loads(frame.data))
                except Exception as exc:
                    log.exception("failed to handle %s", frame.data)
                    await ws.send_json(status_msg("error", repr(exc)))
        finally:
            self._clients.discard(ws)
        return ws

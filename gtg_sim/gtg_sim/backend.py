"""Web UI backend for the offline sim. "start" runs one simulation and sends the whole run."""

import asyncio
import logging

from gtg_webui.server import Backend, hello_msg, samples_msg, status_msg

from .simulate import simulate
from .types import Params, State

log = logging.getLogger("gtg_sim")


class SimBackend(Backend):
    def __init__(self, params: Params, goal, initial, dt, t_max):
        self.server = None  # set by __main__ after UiServer is built
        self.params = params
        self.goal = list(goal)
        self.initial = dict(initial)  # {"x", "y", "theta"}
        self.dt = dt
        self.t_max = t_max
        self.samples = []
        self.status = status_msg("idle")

    def _config(self):
        params = self.params.to_dict()
        params.update(dt=self.dt, t_max=self.t_max)
        return {"type": "config", "params": params, "goal": self.goal,
                "initial": self.initial}

    def on_connect(self):
        return [hello_msg("Offline sim (Python)", "batch", True), self._config(),
                samples_msg(self.samples, reset=True), self.status]

    async def on_message(self, msg):
        kind = msg.get("type")
        if kind == "set_goal":
            self.goal = [float(msg["x"]), float(msg["y"])]
        elif kind == "set_initial":
            self.initial = {k: float(msg[k]) for k in ("x", "y", "theta")}
        elif kind == "set_params":
            for key, value in msg["params"].items():
                if key in ("dt", "t_max"):
                    setattr(self, key, float(value))
                elif hasattr(self.params, key):
                    setattr(self.params, key, float(value))
        elif kind == "start":
            await self._run()
            return
        elif kind == "reset":
            self.samples = []
            self.status = status_msg("idle")
            await self.server.broadcast(samples_msg([], reset=True))
            await self.server.broadcast(self.status)
            return
        else:
            return
        await self.server.broadcast(self._config())

    async def _run(self):
        if self.dt <= 0 or self.t_max <= 0:
            raise ValueError("dt and t_max must be positive")
        await self.server.broadcast(status_msg("running"))
        initial = State.from_pose(**self.initial)
        loop = asyncio.get_running_loop()
        try:
            self.samples = await loop.run_in_executor(
                None, simulate, initial, self.goal, self.params, self.dt, self.t_max)
        except NotImplementedError as exc:
            self.status = status_msg("error", str(exc))
        except Exception as exc:
            log.exception("simulation failed")
            self.status = status_msg("error", repr(exc))
        else:
            last = self.samples[-1]
            reached = last[7] < self.params.epsilon
            self.status = status_msg(
                "reached" if reached else "idle",
                f"dist {last[7]:.3f} m at t = {last[0]:.2f} s, {len(self.samples)} samples")
            await self.server.broadcast(samples_msg(self.samples, reset=True))
        await self.server.broadcast(self.status)

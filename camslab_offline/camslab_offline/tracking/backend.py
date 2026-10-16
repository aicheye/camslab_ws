"""Web UI backend for the offline sim."""

import asyncio
import dataclasses
import logging

from camslab.adaptive import COLUMNS, Estimator
from camslab_webui.random_trajectory import path_inside, random_params
from camslab_webui.server import Backend, adaptive_config, hello_msg, samples_msg, status_msg

from .simulate import simulate
from camslab.trajectory import check_limits, make_shape, sample_path
from camslab.types import SHAPES, Params, State

log = logging.getLogger("camslab_offline.tracking")

RANDOM_TRIES = 50


class SimBackend(Backend):
    def __init__(self, params: Params, initial, dt, t_max, adaptive=False):
        self.server = None
        self.params = params
        self.initial = dict(initial)
        self.dt = dt
        self.t_max = t_max
        self.adaptive = adaptive
        self.samples = []
        self.status = status_msg("idle")

    def _path(self):
        """p*(t) over [0, t_max] for the map, or [] until position() is written."""
        try:
            return sample_path(make_shape(self.params), self.t_max)
        except (NotImplementedError, ValueError):
            return []

    def _config(self):
        params = self.params.to_dict()
        params.update(dt=self.dt, t_max=self.t_max)
        return {"type": "config", "params": params, "path": self._path(),
                "initial": self.initial, "adaptive": self._adaptive()}

    def _adaptive(self):
        """The estimator's columns when it runs."""
        return adaptive_config(self.adaptive, COLUMNS if self.adaptive else ())

    def on_connect(self):
        return [hello_msg("Offline sim (Python)", "batch", True), self._config(),
                samples_msg(self.samples, reset=True), self.status]

    async def on_message(self, msg):
        kind = msg.get("type")
        if kind == "set_initial":
            self.initial = {k: float(msg[k]) for k in ("x", "y", "theta")}
        elif kind == "set_params":
            for key, value in msg["params"].items():
                if key == "shape":
                    if value not in SHAPES:
                        raise ValueError(f"shape must be one of {SHAPES}, got {value!r}")
                    self.params.shape = value
                elif key == "control_points":
                    points = tuple(float(v) for v in value)
                    make_shape(dataclasses.replace(self.params, control_points=points))
                    self.params.control_points = points
                elif key == "adaptive":
                    self.adaptive = bool(value)
                elif key in ("dt", "t_max"):
                    setattr(self, key, float(value))
                elif hasattr(self.params, key):
                    setattr(self.params, key, float(value))
        elif kind == "randomize_trajectory":
            self._randomize()
            await self.server.broadcast(samples_msg([], reset=True))
            await self.server.broadcast(self.status)
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

    def _randomize(self):
        """Replace the trajectory parameters with a random set for the current shape that
        passes check_limits() over [0, t_max] and whose path stays inside the area."""
        for _ in range(RANDOM_TRIES):
            picked = random_params(self.params.shape, self.params.to_dict())
            if "control_points" in picked:
                picked["control_points"] = tuple(picked["control_points"])
            candidate = dataclasses.replace(self.params, **picked)
            shape = make_shape(candidate)
            try:
                check_limits(shape, candidate, self.t_max)
            except ValueError:
                continue
            if path_inside(sample_path(shape, self.t_max)):
                self.params = candidate
                self.samples = []
                self.status = status_msg("idle", "random trajectory set")
                return
        raise ValueError(f"no random {self.params.shape} within the limits and the area "
                         f"in {RANDOM_TRIES} tries")

    async def _run(self):
        if self.dt <= 0 or self.t_max <= 0:
            raise ValueError("dt and t_max must be positive")
        await self.server.broadcast(status_msg("running"))
        await self.server.broadcast(self._config())
        initial = State.from_pose(**self.initial)
        loop = asyncio.get_running_loop()
        warning = ""
        try:
            check_limits(make_shape(self.params), self.params, self.t_max)
        except ValueError as exc:
            warning = f"; the car cannot track the whole trajectory: {exc}"
        except NotImplementedError:
            pass
        try:
            estimator = Estimator(self.params) if self.adaptive else None
            self.samples = await loop.run_in_executor(
                None, simulate, initial, self.params, self.dt, self.t_max, estimator)
        except (NotImplementedError, ValueError) as exc:
            self.status = status_msg("error", str(exc))
        except Exception as exc:
            log.exception("simulation failed")
            self.status = status_msg("error", repr(exc))
        else:
            last = self.samples[-1]
            self.status = status_msg(
                "idle", f"||p* - p|| = {last[7]:.3f} m at t = {last[0]:.2f} s, "
                        f"{len(self.samples)} samples{warning}")
            await self.server.broadcast(samples_msg(self.samples, reset=True))
        await self.server.broadcast(self.status)

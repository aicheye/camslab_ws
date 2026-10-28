"""Web UI backend for the standalone sim: the car drives in real time and the samples
stream to the browser as they are computed, like the Gazebo bridge."""

import asyncio
import dataclasses
import logging
import time

import numpy as np

from camslab_webui.random_trajectory import path_inside, random_params
from camslab_webui.server import (Backend, adaptive_config, hello_msg, history_msgs, samples_msg,
                                  status_msg)

from .simulate import ESTIMATORS, make_observer, simulate_steps
from camslab.trajectory import check_limits, make_shape, reference, sample_path
from camslab.types import SHAPES, Params, State

log = logging.getLogger("camslab_standalone.tracking")

RANDOM_TRIES = 50


class SimBackend(Backend):
    def __init__(self, params: Params, initial, dt, t_max, adaptive="", seed=None):
        """initial is {x, y, theta} or None for the start of the path. seed seeds the
        random camera and markers of the fiducial estimator, a new draw every run."""
        self.server = None
        self.params = params
        self.rng = np.random.default_rng(seed)
        self.initial = dict(initial) if initial else self._path_start()
        self.start_on_path = initial is None
        self.randomize_on_start = False
        self.dt = dt
        self.t_max = t_max
        self.adaptive = adaptive or ""
        self.samples = []
        self.status = status_msg("idle")
        self.running = False
        self.stop_requested = False

    def _path_start(self):
        """{x, y, theta} of p*(0) and q*(0), or the area's corner until the path exists."""
        try:
            ref = reference(make_shape(self.params), 0.0)
        except (NotImplementedError, ValueError):
            return {"x": 0.5, "y": 0.5, "theta": 0.0}
        return {"x": round(float(ref.p[0]), 4), "y": round(float(ref.p[1]), 4),
                "theta": round(float(np.arctan2(ref.q[1], ref.q[0])), 4)}

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
        observer = make_observer(self.adaptive, self.params)
        if observer is None:
            return adaptive_config(self.adaptive)
        return adaptive_config(self.adaptive, observer.fields, plots=observer.plots)

    def on_connect(self):
        return [hello_msg("Standalone sim (Python)", "live", True), self._config(),
                *history_msgs(list(self.samples)), self.status]

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
                    if value and value not in ESTIMATORS:
                        raise ValueError(f"adaptive must be one of {ESTIMATORS} or empty, "
                                         f"got {value!r}")
                    self.adaptive = value or ""
                elif key in ("dt", "t_max"):
                    setattr(self, key, float(value))
                elif hasattr(self.params, key):
                    setattr(self.params, key, float(value))
        elif kind == "randomize_trajectory":
            self._randomize()
            await self.server.broadcast(samples_msg([], reset=True))
            await self.server.broadcast(self.status)
        elif kind == "start":
            self.start_on_path = bool(msg.get("on_path", False))
            self.randomize_on_start = bool(msg.get("randomize", False))
            if not self.running:
                asyncio.get_running_loop().create_task(self._run())
            return
        elif kind == "stop":
            self.stop_requested = True
            return
        elif kind == "reset":
            self.stop_requested = True
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
        self.running = True
        self.stop_requested = False
        self.samples = []
        warning = ""
        try:
            if self.randomize_on_start:
                self._randomize()
            if self.start_on_path:
                self.initial = self._path_start()
            await self.server.broadcast(status_msg("running"))
            await self.server.broadcast(self._config())
            await self.server.broadcast(samples_msg([], reset=True))
            try:
                check_limits(make_shape(self.params), self.params, self.t_max)
            except ValueError as exc:
                warning = f"; the car cannot track the whole trajectory: {exc}"
            except NotImplementedError:
                pass
            initial = State.from_pose(**self.initial)
            observer = make_observer(self.adaptive, self.params, self.rng)
            await asyncio.get_running_loop().run_in_executor(None, self._drive, initial, observer)
        except (NotImplementedError, ValueError) as exc:
            self.status = status_msg("error", str(exc))
        except Exception as exc:
            log.exception("simulation failed")
            self.status = status_msg("error", repr(exc))
        else:
            if self.samples:
                last = self.samples[-1]
                stopped = " (stopped)" if self.stop_requested else ""
                self.status = status_msg(
                    "idle", f"||p* - p|| = {last[7]:.3f} m at t = {last[0]:.2f} s{stopped}, "
                            f"{len(self.samples)} samples{warning}")
            else:
                self.status = status_msg("idle")
        finally:
            self.running = False
        await self.server.broadcast(self.status)

    def _drive(self, initial, observer):
        """In a worker thread: step the sim at 1 / dt, paced to wall-clock time, and send
        every row to the browsers as it is computed. A step that takes longer than dt
        is sent as soon as it is done, so the sim runs slower than real time but stays
        smooth."""
        start = time.perf_counter()
        for row in simulate_steps(initial, self.params, self.dt, self.t_max, observer):
            if self.stop_requested:
                break
            self.samples.append(row)
            self.server.broadcast_threadsafe(samples_msg([row]))
            lag = start + row[0] - time.perf_counter()
            if lag > 0:
                time.sleep(lag)

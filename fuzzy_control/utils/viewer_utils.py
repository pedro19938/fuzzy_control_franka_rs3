"""Helpers for the MuJoCo viewer.

visual functions (viewer.user_scn): no physics and no collision.
Order of the geoms in the scene: [target marker] [desired path] [EE trail]
"""

import time
from collections import deque

import mujoco
import numpy as np


def _new_geom(scn, kind, size, pos, color):
    if scn.ngeom >= scn.maxgeom:
        return None
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, kind, np.asarray(size, dtype=float), np.asarray(pos, dtype=float),
                        np.eye(3).flatten(), np.asarray(color, dtype=np.float32))
    scn.ngeom += 1
    return g


def _new_line(scn, p0, p1, color, width=2.0):
    g = _new_geom(scn, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), color)
    if g is not None:
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, width,
                             np.asarray(p0, dtype=float), np.asarray(p1, dtype=float))


def create_marker(viewer, pos, radius, color):
    """Sphere at the target position. Must be the FIRST geom created (index 0)."""
    viewer.user_scn.ngeom = 0
    _new_geom(viewer.user_scn, mujoco.mjtGeom.mjGEOM_SPHERE, [radius, 0, 0], pos, color)


def move_marker(viewer, pos):
    viewer.user_scn.geoms[0].pos[:] = pos


def draw_path(viewer, points, color):
    """Line connecting the points (desired path). Stays fixed in the scene."""
    for p0, p1 in zip(points[:-1], points[1:]):
        _new_line(viewer.user_scn, p0, p1, color)


class Trail:
    """Line that follows the EE, showing the last `n_points` points.
    Must be created AFTER the marker and the path (it takes the end of the scene)."""

    def __init__(self, viewer, n_points, color):
        self.scn = viewer.user_scn
        self.start = self.scn.ngeom
        self.color = color
        self.points = deque(maxlen=n_points)

    def add(self, p):
        self.points.append(np.array(p, dtype=float))
        self.scn.ngeom = self.start  # redraws the trail from scratch
        pts = list(self.points)
        for p0, p1 in zip(pts[:-1], pts[1:]):
            _new_line(self.scn, p0, p1, self.color, width=3.0)


def wait_real_time(start, dt):
    """Sleeps as needed so that one simulation step actually lasts dt."""
    wait = dt - (time.time() - start)
    if wait > 0:
        time.sleep(wait)

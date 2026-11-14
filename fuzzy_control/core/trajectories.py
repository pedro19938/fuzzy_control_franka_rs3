"""
Trajectories for the EE.

Every trajectory is called as  pos, vel = traj(t)
  pos: desired position (x, y, z) at instant t  [m]
  vel: desired velocity (derivative of pos)     [m/s]  -> used in the feedforward
"""

from fractions import Fraction
from math import gcd, lcm

import numpy as np


class Waypoints:
    """Visits a list of points, switching every `time_per_waypoint` seconds.
    The desired velocity is zero (the target stays still)."""

    def __init__(self, waypoints, time_per_waypoint):
        self.waypoints = [np.asarray(a, dtype=float) for a in waypoints]
        self.time_per_waypoint = time_per_waypoint

    def __call__(self, t):
        i = int(t // self.time_per_waypoint) % len(self.waypoints)
        return self.waypoints[i], np.zeros(3)

    def path(self, n=None):
        """Points to draw the path in the viewer (closed polygon)."""
        return np.array(self.waypoints + [self.waypoints[0]])


class Lissajous:
    """3D Lissajous curve: each axis oscillates as a sinusoid.

        pos_i(t) = center_i + A_i * sin(2*pi*f_i*t + phase_i)
        vel_i(t) =  A_i * 2*pi*f_i * cos(2*pi*f_i*t + phase_i)

    The shape depends on the RATIO between the axis frequencies:
      1:1 with phase pi/2 -> circle/ellipse;
      1:2 -> "figure eight";
      3:2 -> classic loop.
    """

    def __init__(self, center, amplitude, frequency, phase):

        # lissajous parameters
        self.c = np.asarray(center, dtype=float)
        self.A = np.asarray(amplitude, dtype=float)
        self.w = 2 * np.pi * np.asarray(frequency, dtype=float)
        self.phase = np.asarray(phase, dtype=float)

        # Total period = when all axes return to the start together
        # (LCM of each axis period; e.g.: 4 s and 6 s -> 12 s)
        periods = [Fraction(1 / f).limit_denominator(1000) for f in frequency if f > 0]
        if periods:
            lcm_period = Fraction(lcm(*[p.numerator for p in periods]),
                                  gcd(*[p.denominator for p in periods]))
            self.period = float(lcm_period)
        else:
            self.period = 1.0

    def __call__(self, t):
        arg = self.w * t + self.phase
        return self.c + self.A * np.sin(arg), self.A * self.w * np.cos(arg)

    def path(self, n=400):
        ts = np.linspace(0.0, self.period, n)
        return np.array([self(t)[0] for t in ts])


def make_trajectory(cfg):
    """Builds the trajectory chosen in sim_config.TRAJECTORY."""
    if cfg.TRAJECTORY == "waypoints":
        return Waypoints(cfg.WAYPOINTS, cfg.TIME_PER_WAYPOINT)
    if cfg.TRAJECTORY == "lissajous":
        return Lissajous(**cfg.LISSAJOUS)
    raise ValueError(f"Unknown trajectory: {cfg.TRAJECTORY!r}")

"""Membership functions: take x and return a degree between 0 and 1."""

import numpy as np


def trapezoid(x, a, b, c, d):
    """Rises from a to b, stays at 1 between b and c, falls from c to d.
    If a == b (or c == d), that side becomes a "shoulder" (stays at 1)."""
    x = np.asarray(x, dtype=float)
    with np.errstate(invalid="ignore"):  # allows infinite a/b/c/d
        rise = np.ones_like(x) if b == a else (x - a) / (b - a)
        fall = np.ones_like(x) if d == c else (d - x) / (d - c)
    return np.clip(np.minimum(rise, fall), 0.0, 1.0)


def triangle(x, a, b, c):
    """Triangle = trapezoid with a zero-width top (b == c)."""
    return trapezoid(x, a, b, b, c)


TYPES = {"trapezoid": trapezoid, "triangle": triangle}


def make_function(kind, params):
    """Turns ("triangle", (a, b, c)) from the config into a function f(x).
       creates the corresponding membership function (trapezoid or triangle)
       with the given parameters.
       """
    f = TYPES[kind]
    return lambda x: f(x, *params)

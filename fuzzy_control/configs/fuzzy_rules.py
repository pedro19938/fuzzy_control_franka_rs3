r"""
Fuzzy Controller Configs
=================================

fuzzy controller definition file:
  1. the INPUT sets  (position error, in meters)
  2. the OUTPUT sets (EE velocity, in m/s)
  3. the RULES that link input -> output

Set names (linguistic):
  NB = Negative Big    NS = Negative Small    ZE = Zero
  PS = Positive Small  PB = Positive Big

"""

INF = float("inf")

# ---------------------------------------------------------------------------
# 1. INPUT: error = target - current EE position  [m]
# ---------------------------------------------------------------------------
# E_MAX: maximum error value before we consider it far.
# Decreasing E_MAX makes the control more aggressive
# Increasing makes it smoother.

E_MAX = 0.30

INPUT_SETS = {
    # Very negative error: the target is well "behind" on the axis.
    # Saturates at -E_MAX.
    "NB": ("trapezoid", (-INF, -INF, -E_MAX, -E_MAX / 2)),
    # Moderate negative error.
    "NS": ("triangle",  (-E_MAX, -E_MAX / 2, 0.0)),
    # Practically at the target.
    # The WIDTH of this set defines the "smoothness"
    # near the target: wider -> slower, smoother arrival.
    "ZE": ("triangle",  (-E_MAX / 2, 0.0, E_MAX / 2)),
    # Moderate positive error.
    "PS": ("triangle",  (0.0, E_MAX / 2, E_MAX)),
    # Very positive error: the target is well "ahead".
    # Saturates at +E_MAX.
    "PB": ("trapezoid", (E_MAX / 2, E_MAX, INF, INF)),
}

# ---------------------------------------------------------------------------
# 2. OUTPUT: Cartesian EE velocity along the axis  [m/s]
# ---------------------------------------------------------------------------
# V_MAX: highest velocity the fuzzy can request.
V_MAX = 1.0

# How many points to use to discretize the output for the centroid computation.
# More points = more precise, but slower.
N_OUTPUT_POINTS = 501

OUTPUT_SETS = {
    "NB": ("triangle", (-V_MAX, -V_MAX, -V_MAX / 2)),   # fast backward
    "NS": ("triangle", (-V_MAX, -V_MAX / 2, 0.0)),      # slow backward
    "ZE": ("triangle", (-V_MAX / 2, 0.0, V_MAX / 2)),   # stopped
    "PS": ("triangle", (0.0, V_MAX / 2, V_MAX)),        # slow forward
    "PB": ("triangle", (V_MAX / 2, V_MAX, V_MAX)),      # fast forward
}

# ---------------------------------------------------------------------------
# 3. RULES:  IF error is <A>  THEN velocity is <B>
# ---------------------------------------------------------------------------
RULES = [
    ("NB", "NB"),  # target far behind      -> fast backward
    ("NS", "NS"),  # target slightly behind -> slow backward
    ("ZE", "ZE"),  # at the target          -> stopped
    ("PS", "PS"),  # target slightly ahead  -> slow forward
    ("PB", "PB"),  # target far ahead       -> fast forward
]

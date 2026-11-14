"""
General simulation configs
"""

import numpy as np

# ---------------------------------------------------------------------------
# Robot
# ---------------------------------------------------------------------------
MODEL = "franka_fr3"           # robot name
EE_SITE = "attachment_site"    # EE
INITIAL_KEYFRAME = 0           # initial "home" pose

# ---------------------------------------------------------------------------
# EE trajectory  (positions in meters, world frame)
# in the "home" pose the EE is at ~(0.55, 0.00, 0.62).
# ---------------------------------------------------------------------------
TRAJECTORY = "lissajous"   # "waypoints" or "lissajous"

# Feedforward: adds the trajectory velocity to the fuzzy command.
#   False -> the fuzzy only reacts to the error;
#   True  -> feedforward provides the velocity the trajectory requires,
#   and the fuzzy corrects the remaining deviation.
USE_FEEDFORWARD = True

# the chosen trajectory was "lissajous", but it can be customized to
# any curve in /fuzzy_control/core/trajectories.py.
LISSAJOUS = dict(
    center=[0.55, 0.00, 0.50],
    amplitude=[0.00, 0.30, 0.20],          # [m]  (x, y, z)
    frequency=[0.0, 3 / 20, 2 / 20],       # [Hz] -> full figure every 20 s
    phase=[0.0, np.pi / 2, 0.0],           # [rad]
)

# --- "waypoints": the EE visits one point at a time, switching every TIME_PER_WAYPOINT s
WAYPOINTS = [
    np.array([0.55,  0.20, 0.50]),
    np.array([0.45, -0.20, 0.65]),
    np.array([0.60,  0.00, 0.40]),
    np.array([0.55,  0.00, 0.62]),
]

TIME_PER_WAYPOINT = 4.0  # [s]

# ---------------------------------------------------------------------------
# Inverse kinematics (EE velocity -> joint velocities)
# Low-level control
# ---------------------------------------------------------------------------
LAMBDA_DLS = 0.05

# The arm has 7 joints, but we only control 3 things (x, y, z). The
# "spare" joints (null space) pull the arm back to the home pose with this
# gain, so it does not end up in odd postures. 0 disables it.
NULLSPACE_GAIN = 1.0

# ---------------------------------------------------------------------------
# Visualization / log
# ---------------------------------------------------------------------------
MARKER_RADIUS = 0.02                  # target sphere [m]
MARKER_COLOR = (1.0, 0.0, 0.0, 0.6)   # RGBA
PATH_COLOR = (0.2, 0.6, 1.0, 1.0)     # desired path (blue)
TRAIL_COLOR = (1.0, 0.8, 0.0, 1.0)    # actual EE path (yellow)
TRAIL_DURATION = 6.0                  # how many seconds of trail to show
PRINT_INTERVAL = 0.5                  # prints the error every X s of simulation

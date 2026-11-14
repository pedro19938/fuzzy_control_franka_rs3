"""
Runs the fuzzy EE control of the Franka FR3 with the MuJoCo viewer.

    fuzzy/bin/python runner.py

Configs: fuzzy_control/configs/
  - trajectory (waypoints / lissajous) and feedforward: sim_config.py
  - fuzzy sets and rules: fuzzy_rules.py
"""
# imports
import signal
import threading
import time

import mujoco
import mujoco.viewer
import mujoco_menagerie as mm
import numpy as np

from fuzzy_control.configs import sim_config as cfg
from fuzzy_control.core.ee_controller import EEFuzzyController
from fuzzy_control.core.trajectories import make_trajectory
from fuzzy_control.utils.viewer_utils import (Trail, create_marker, draw_path,
                                              move_marker, wait_real_time)

STEPS_PER_TRAIL_FRAME = 20  # updates the trail every 20 steps (25 Hz)

# helper function to stop the control loop with ctrl+c
def install_ctrl_c():

    stop = threading.Event()

    def on_press(sig, frame):
        print("\nCtrl+C: shutting down... (press again to force)")
        stop.set()
        signal.signal(signal.SIGINT, signal.SIG_DFL)  # next Ctrl+C kills immediately

    signal.signal(signal.SIGINT, on_press)
    return stop


def main() -> None:
    # mujoco model
    model = mm.load(cfg.MODEL)
    data = mujoco.MjData(model)

    # reset to the initial keyframe pose
    mujoco.mj_resetDataKeyframe(model, data, cfg.INITIAL_KEYFRAME)
    mujoco.mj_forward(model, data)

    # fuzzy controller
    controller = EEFuzzyController(model, data)
    trajectory = make_trajectory(cfg)

    # number of steps for the trail (duration / timestep)
    n_trail = int(cfg.TRAIL_DURATION / (model.opt.timestep * STEPS_PER_TRAIL_FRAME))
    stop = install_ctrl_c()

    # Main loop
    with mujoco.viewer.launch_passive(model, data) as viewer:

        # initializes the visual markers in the viewer (trajectory, trail, target)
        with viewer.lock():
            create_marker(viewer, trajectory(0.0)[0], cfg.MARKER_RADIUS, cfg.MARKER_COLOR)
            draw_path(viewer, trajectory.path(), cfg.PATH_COLOR)
            trail = Trail(viewer, n_trail, cfg.TRAIL_COLOR)

        # Control loop
        step = 0
        last_print = 0.0
        while viewer.is_running() and not stop.is_set():
            start = time.time()

            target, target_vel = trajectory(data.time)
            error = controller.step(target, target_vel)
            mujoco.mj_step(model, data)
            step += 1

            # updates the viewer
            with viewer.lock():
                move_marker(viewer, target)
                if step % STEPS_PER_TRAIL_FRAME == 0:
                    trail.add(controller.ee_position())
            viewer.sync()

            # prints the error every cfg.PRINT_INTERVAL seconds
            if data.time - last_print >= cfg.PRINT_INTERVAL:
                last_print = data.time
                print(f"t={data.time:5.1f}s  error (mm) = {np.round(error * 1000, 1)}"
                      f"  |error| = {np.linalg.norm(error) * 1000:5.1f} mm")

            # runs at real-time rate (model.opt.timestep)
            wait_real_time(start, model.opt.timestep)

# main
if __name__ == "__main__":
    main()

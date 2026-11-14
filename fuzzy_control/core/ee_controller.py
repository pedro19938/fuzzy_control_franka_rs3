"""
EE position control with fuzzy + Jacobian.

At each step:
  1. error = target - EE position              (x, y, z)
  2. fuzzy on each axis -> EE velocity         (m/s)
  3. Jacobian (DLS)     -> joint velocities    (rad/s)
  4. integrates and sends to the servos (on the FR3, ctrl = desired joint angle)
"""

import mujoco
import numpy as np

from fuzzy_control.configs import sim_config as cfg
from fuzzy_control.core.fuzzy_siso import FuzzySISO


class EEFuzzyController:
    def __init__(self, model, data, fuzzy=None, site=cfg.EE_SITE,
                 lambda_dls=cfg.LAMBDA_DLS, nullspace_gain=cfg.NULLSPACE_GAIN,
                 use_feedforward=cfg.USE_FEEDFORWARD):
        # mujoco model
        self.model = model
        self.data = data

        # fuzzy controller
        self.fuzzy = fuzzy or FuzzySISO()

        # robot EE
        self.site_id = model.site(site).id

        # IK
        self.lambda_dls = lambda_dls
        self.nullspace_gain = nullspace_gain
        self.use_feedforward = use_feedforward

        self.J = np.zeros((3, model.nv))
        self.reset()

    def reset(self):
        """Stores the current pose as 'home' and as the joint reference."""
        self.q_home = self.data.qpos.copy()
        self.q_ref = self.data.qpos.copy()

    def ee_position(self):
        return self.data.site_xpos[self.site_id].copy()

    def step(self, target, target_vel=None):
        """Computes and applies to data.ctrl the command for one step. Returns the error.

        target_vel: trajectory velocity at the current instant (feedforward).
        """
        m, d = self.model, self.data

        # 1. error -> target_EE - pos_EE
        error = target - d.site_xpos[self.site_id] #

        # 2. fuzzy on each axis (+ trajectory feedforward, if enabled)
        v = np.array([self.fuzzy.compute(e) for e in error])
        if self.use_feedforward and target_vel is not None:
            v += target_vel

        # 3. site position Jacobian and its damped inverse
        mujoco.mj_jacSite(m, d, self.J, None, self.site_id)
        J = self.J
        J_pinv = J.T @ np.linalg.inv(J @ J.T + self.lambda_dls**2 * np.eye(3))
        dq = J_pinv @ v

        # Null space: uses the "spare" joints to return to the home pose
        N = np.eye(m.nv) - J_pinv @ J
        dq += N @ (self.nullspace_gain * (self.q_home - d.qpos))

        # 4. integrates the reference and sends it to the servos
        self.q_ref += dq * m.opt.timestep
        np.clip(self.q_ref, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1], out=self.q_ref)
        d.ctrl[:] = self.q_ref
        return error

"""
Controle de posição do EE com fuzzy + Jacobiano.

A cada passo:
  1. erro = alvo - posição do EE            (x, y, z)
  2. fuzzy em cada eixo -> velocidade do EE (m/s)
  3. Jacobiano (DLS)    -> velocidade das juntas (rad/s)
  4. integra e manda para os servos (no FR3, ctrl = ângulo desejado da junta)
"""

import mujoco
import numpy as np

from fuzzy_control.configs import sim_config as cfg
from fuzzy_control.core.fuzzy_siso import FuzzySISO


class EEFuzzyController:
    def __init__(self, model, data, fuzzy=None, site=cfg.SITE_EE,
                 lambda_dls=cfg.LAMBDA_DLS, ganho_espaco_nulo=cfg.GANHO_ESPACO_NULO,
                 usar_feedforward=cfg.USAR_FEEDFORWARD):
        # modelo mujoco
        self.model = model
        self.data = data
        
        # controlador fuzzy
        self.fuzzy = fuzzy or FuzzySISO()
        
        # EE do robo
        self.site_id = model.site(site).id
        
        # IK
        self.lambda_dls = lambda_dls
        self.ganho_espaco_nulo = ganho_espaco_nulo
        self.usar_feedforward = usar_feedforward

        self.J = np.zeros((3, model.nv))
        self.reset()

    def reset(self):
        """Guarda a pose atual como 'home' e como referência das juntas."""
        self.q_home = self.data.qpos.copy()
        self.q_ref = self.data.qpos.copy()

    def posicao_ee(self):
        return self.data.site_xpos[self.site_id].copy()

    def passo(self, alvo, vel_alvo=None):
        """Calcula e aplica em data.ctrl o comando de um passo. Retorna o erro.

        vel_alvo: velocidade da trajetória no instante atual (feedforward).
        """
        m, d = self.model, self.data

        # 1. erro
        erro = alvo - d.site_xpos[self.site_id]

        # 2. fuzzy em cada eixo (+ feedforward da trajetória, se ligado)
        v = np.array([self.fuzzy.calcular(e) for e in erro])
        if self.usar_feedforward and vel_alvo is not None:
            v += vel_alvo

        # 3. Jacobiano de posição do site e sua inversa amortecida
        mujoco.mj_jacSite(m, d, self.J, None, self.site_id)
        J = self.J
        J_pinv = J.T @ np.linalg.inv(J @ J.T + self.lambda_dls**2 * np.eye(3))
        dq = J_pinv @ v

        # Espaço nulo: usa as juntas "sobrando" para voltar à pose home
        N = np.eye(m.nv) - J_pinv @ J
        dq += N @ (self.ganho_espaco_nulo * (self.q_home - d.qpos))

        # 4. integra a referência e manda para os servos
        self.q_ref += dq * m.opt.timestep
        np.clip(self.q_ref, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1], out=self.q_ref)
        d.ctrl[:] = self.q_ref
        return erro

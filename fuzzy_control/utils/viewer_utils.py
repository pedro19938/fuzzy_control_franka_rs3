"""Ajudantes para o viewer do MuJoCo.

funções visuais (viewer.user_scn): sem física e sem colide.
Ordem dos geoms na cena: [marcador do alvo] [caminho desejado] [rastro do EE]
"""

import time
from collections import deque

import mujoco
import numpy as np


def _novo_geom(scn, tipo, tamanho, pos, cor):
    if scn.ngeom >= scn.maxgeom:
        return None
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, tipo, np.asarray(tamanho, dtype=float), np.asarray(pos, dtype=float),
                        np.eye(3).flatten(), np.asarray(cor, dtype=np.float32))
    scn.ngeom += 1
    return g


def _nova_linha(scn, p0, p1, cor, largura=2.0):
    g = _novo_geom(scn, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), cor)
    if g is not None:
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, largura,
                             np.asarray(p0, dtype=float), np.asarray(p1, dtype=float))


def criar_marcador(viewer, pos, raio, cor):
    """Esfera na posição do alvo. Deve ser o PRIMEIRO geom criado (índice 0)."""
    viewer.user_scn.ngeom = 0
    _novo_geom(viewer.user_scn, mujoco.mjtGeom.mjGEOM_SPHERE, [raio, 0, 0], pos, cor)


def mover_marcador(viewer, pos):
    viewer.user_scn.geoms[0].pos[:] = pos


def desenhar_caminho(viewer, pontos, cor):
    """Linha ligando os pontos (caminho desejado). Fica fixa na cena."""
    for p0, p1 in zip(pontos[:-1], pontos[1:]):
        _nova_linha(viewer.user_scn, p0, p1, cor)


class Rastro:
    """Linha que segue o EE, mostrando os últimos `n_pontos` pontos.
    Deve ser criado DEPOIS do marcador e do caminho (ocupa o fim da cena)."""

    def __init__(self, viewer, n_pontos, cor):
        self.scn = viewer.user_scn
        self.inicio = self.scn.ngeom
        self.cor = cor
        self.pontos = deque(maxlen=n_pontos)

    def adicionar(self, p):
        self.pontos.append(np.array(p, dtype=float))
        self.scn.ngeom = self.inicio  # redesenha o rastro do zero
        pts = list(self.pontos)
        for p0, p1 in zip(pts[:-1], pts[1:]):
            _nova_linha(self.scn, p0, p1, self.cor, largura=3.0)


def esperar_tempo_real(inicio, dt):
    """Dorme o necessário para que um passo de simulação dure dt de verdade."""
    espera = dt - (time.time() - inicio)
    if espera > 0:
        time.sleep(espera)

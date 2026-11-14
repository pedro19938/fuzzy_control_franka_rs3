"""
Trajetórias para o EE.

Toda trajetória é chamada como  pos, vel = traj(t)
  pos: posição desejada (x, y, z) no instante t  [m]
  vel: velocidade desejada (derivada de pos)      [m/s]  -> usada no feedforward
"""

from fractions import Fraction
from math import gcd, lcm

import numpy as np


class PontosFixos:
    """Visita uma lista de pontos, trocando a cada `tempo_por_alvo` segundos.
    A velocidade desejada é zero (o alvo fica parado)."""

    def __init__(self, alvos, tempo_por_alvo):
        self.alvos = [np.asarray(a, dtype=float) for a in alvos]
        self.tempo_por_alvo = tempo_por_alvo

    def __call__(self, t):
        i = int(t // self.tempo_por_alvo) % len(self.alvos)
        return self.alvos[i], np.zeros(3)

    def caminho(self, n=None):
        """Pontos para desenhar o caminho no viewer (polígono fechado)."""
        return np.array(self.alvos + [self.alvos[0]])


class Lissajous:
    """Curva de Lissajous 3D: cada eixo oscila como uma senoide.

        pos_i(t) = centro_i + A_i * sin(2*pi*f_i*t + fase_i)
        vel_i(t) =  A_i * 2*pi*f_i * cos(2*pi*f_i*t + fase_i)

    O desenho depende da RAZÃO entre as frequências dos eixos:
      1:1 com fase pi/2 -> círculo/elipse;   
      1:2 -> "oito";   
      3:2 -> laço clássico.
    """

    def __init__(self, centro, amplitude, frequencia, fase):
        
        # parametros lissajous
        self.c = np.asarray(centro, dtype=float)
        self.A = np.asarray(amplitude, dtype=float)
        self.w = 2 * np.pi * np.asarray(frequencia, dtype=float)
        self.fase = np.asarray(fase, dtype=float)

        # Período total = quando todos os eixos voltam ao início juntos
        # (MMC dos períodos de cada eixo; ex.: 4 s e 6 s -> 12 s)
        periodos = [Fraction(1 / f).limit_denominator(1000) for f in frequencia if f > 0]
        if periodos:
            mmc = Fraction(lcm(*[p.numerator for p in periodos]),
                           gcd(*[p.denominator for p in periodos]))
            self.periodo = float(mmc)
        else:
            self.periodo = 1.0

    def __call__(self, t):
        arg = self.w * t + self.fase
        return self.c + self.A * np.sin(arg), self.A * self.w * np.cos(arg)

    def caminho(self, n=400):
        ts = np.linspace(0.0, self.periodo, n)
        return np.array([self(t)[0] for t in ts])


def criar_trajetoria(cfg):
    """Monta a trajetória escolhida em sim_config.TRAJETORIA."""
    if cfg.TRAJETORIA == "pontos":
        return PontosFixos(cfg.ALVOS, cfg.TEMPO_POR_ALVO)
    if cfg.TRAJETORIA == "lissajous":
        return Lissajous(**cfg.LISSAJOUS)
    raise ValueError(f"Trajetória desconhecida: {cfg.TRAJETORIA!r}")

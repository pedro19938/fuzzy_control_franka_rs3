"""Funções de pertinência: recebem x e devolvem um grau entre 0 e 1."""

import numpy as np


def trapezio(x, a, b, c, d):
    """Sobe de a até b, fica em 1 entre b e c, desce de c até d.
    Se a == b (ou c == d), aquele lado vira um "ombro" (fica em 1)."""
    x = np.asarray(x, dtype=float)
    with np.errstate(invalid="ignore"):  # permite a/b/c/d infinitos
        sobe = np.ones_like(x) if b == a else (x - a) / (b - a)
        desce = np.ones_like(x) if d == c else (d - x) / (d - c)
    return np.clip(np.minimum(sobe, desce), 0.0, 1.0)


def triangulo(x, a, b, c):
    """Triângulo = trapézio com o topo de largura zero (b == c)."""
    return trapezio(x, a, b, b, c)


TIPOS = {"trapezio": trapezio, "triangulo": triangulo}


def criar_funcao(tipo, params):
    """Transforma ("triangulo", (a, b, c)) da config numa função f(x).
       cria a func de pertinência correspondente (trapezio ou triangulo) 
       com os parâmetros dados.
       """
    f = TIPOS[tipo]
    return lambda x: f(x, *params)

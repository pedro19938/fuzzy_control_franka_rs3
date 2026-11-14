"""
Controlador fuzzy Mamdani de 1 entrada e 1 saída (SISO).

As 4 etapas:
  1. Fuzzificação   -> grau do erro em cada conjunto (NG, NP, ZE, PP, PG)
  2. Regras          -> força de cada regra = grau do seu antecedente
  3. Agregação       -> corta cada conjunto de saída na força da regra (min)
                        e une todos (max)
  4. Defuzzificação  -> centroide da forma agregada = número final
"""

import numpy as np

from fuzzy_control.configs import fuzzy_rules as cfg
from fuzzy_control.core.membership import criar_funcao


class FuzzySISO:
    def __init__(self, conjuntos_entrada=None, conjuntos_saida=None, regras=None,
                 v_max=None, n_pontos=None):
        # configs em configs/fuzzy_rules.py
        
        # conjuntos de entrada, saida e regras 
        conjuntos_entrada = conjuntos_entrada or cfg.CONJUNTOS_ENTRADA
        conjuntos_saida = conjuntos_saida or cfg.CONJUNTOS_SAIDA
        self.regras = regras or cfg.REGRAS
        
        # V_max permitida e número de pontos para avaliar a saída
        v_max = v_max or cfg.V_MAX
        n_pontos = n_pontos or cfg.N_PONTOS_SAIDA
        
        # cria funções de pertinência para cada conjunto de entrada e saída
        self.entrada = {nome: criar_funcao(*mf) for nome, mf in conjuntos_entrada.items()}

        # A saída é avaliada uma vez só, num universo discreto
        self.v = np.linspace(-v_max, v_max, n_pontos)
        self.saida = {nome: criar_funcao(*mf)(self.v) for nome, mf in conjuntos_saida.items()}

    def fuzzificar(self, erro):
        """Etapa 1: {conjunto: grau} para um valor de erro."""
        return {nome: float(mf(erro)) for nome, mf in self.entrada.items()}

    def calcular(self, erro, detalhes=False):
        """Etapas 1 a 4: erro (m) -> velocidade (m/s)."""
        graus = self.fuzzificar(erro)

        agregado = np.zeros_like(self.v)
        for se, entao in self.regras:
            forca = graus[se]                               # etapa 2
            cortado = np.minimum(forca, self.saida[entao])  # implicação (min)
            agregado = np.maximum(agregado, cortado)        # etapa 3 (max)

        # defuzzyficação: centroide da forma agregada
        area = agregado.sum()
        v = 0.0 if area == 0 else float((self.v * agregado).sum() / area)  # etapa 4

        if detalhes:
            return v, graus, agregado
        return v

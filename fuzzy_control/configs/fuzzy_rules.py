r"""
Configs Controlador Fuzzy
=================================
  
arquivo de definicao do controlador fuzzy:
  1. os conjuntos da ENTRADA (erro de posição, em metros)
  2. os conjuntos da SAÍDA   (velocidade do EE, em m/s)
  3. as REGRAS que ligam entrada -> saída

Nomes dos conjuntos (linguísticos):
  NG = Negativo Grande    NP = Negativo Pequeno    ZE = Zero
  PP = Positivo Pequeno   PG = Positivo Grande

"""

INF = float("inf")

# ---------------------------------------------------------------------------
# 1. ENTRADA: erro = alvo - posição atual do EE  [m]
# ---------------------------------------------------------------------------
# E_MAX: valor maximo de erro antes de considerarmos longe.
# Diminuir E_MAX deixa o controle mais agressivo
# Aumentar deixa mais suave.

E_MAX = 0.30

CONJUNTOS_ENTRADA = {
    # Erro muito negativo: o alvo está bem "atrás" no eixo. 
    # Satura em -E_MAX.
    "NG": ("trapezio",  (-INF, -INF, -E_MAX, -E_MAX / 2)),
    # Erro negativo moderado.
    "NP": ("triangulo", (-E_MAX, -E_MAX / 2, 0.0)),
    # Praticamente no alvo. 
    # A LARGURA deste conjunto define a "suavidade"
    # perto do alvo: mais largo -> chegada mais lenta e suave.
    "ZE": ("triangulo", (-E_MAX / 2, 0.0, E_MAX / 2)),
    # Erro positivo moderado.
    "PP": ("triangulo", (0.0, E_MAX / 2, E_MAX)),
    # Erro muito positivo: o alvo está bem "à frente". 
    # Satura em +E_MAX.
    "PG": ("trapezio",  (E_MAX / 2, E_MAX, INF, INF)),
}

# ---------------------------------------------------------------------------
# 2. SAÍDA: velocidade cartesiana do EE no eixo  [m/s]
# ---------------------------------------------------------------------------
# V_MAX: maior velocidade que o fuzzy pode pedir.
V_MAX = 1.0

# Quantos pontos usar para discretizar a saída no cálculo do centroide.
# Mais pontos = mais preciso, mas mais lento.
N_PONTOS_SAIDA = 501

CONJUNTOS_SAIDA = {
    "NG": ("triangulo", (-V_MAX, -V_MAX, -V_MAX / 2)),   # ré rápida
    "NP": ("triangulo", (-V_MAX, -V_MAX / 2, 0.0)),      # ré devagar
    "ZE": ("triangulo", (-V_MAX / 2, 0.0, V_MAX / 2)),   # parado
    "PP": ("triangulo", (0.0, V_MAX / 2, V_MAX)),        # frente devagar
    "PG": ("triangulo", (V_MAX / 2, V_MAX, V_MAX)),      # frente rápida
}

# ---------------------------------------------------------------------------
# 3. REGRAS:  SE erro é <A>  ENTÃO velocidade é <B>
# ---------------------------------------------------------------------------
REGRAS = [
    ("NG", "NG"),  # alvo muito atrás    -> ré rápida
    ("NP", "NP"),  # alvo um pouco atrás -> ré devagar
    ("ZE", "ZE"),  # no alvo             -> parado
    ("PP", "PP"),  # alvo um pouco à frente -> frente devagar
    ("PG", "PG"),  # alvo muito à frente    -> frente rápida
]

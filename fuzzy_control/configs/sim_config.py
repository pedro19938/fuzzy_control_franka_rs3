"""
Configs gerais da simulação
"""

import numpy as np

# ---------------------------------------------------------------------------
# Robo
# ---------------------------------------------------------------------------
MODELO = "franka_fr3"          # nome robo
SITE_EE = "attachment_site"    # EE 
KEYFRAME_INICIAL = 0           # pose inicial "home"

# ---------------------------------------------------------------------------
# Trajetória do EE  (posições em metros, referencial do mundo)
# na pose "home" o EE fica em ~(0.55, 0.00, 0.62).
# ---------------------------------------------------------------------------
TRAJETORIA = "lissajous"   # "pontos" ou "lissajous"

# Feedforward: soma a velocidade da trajetória ao comando do fuzzy. 
#   False -> o fuzzy só reage ao erro; 
#   True  -> forward fornece a velocidade que a trajetoria exige, 
#   e o fuzzy corrige o desvio que sobra. 
USAR_FEEDFORWARD = True

# trajetoria escolhida foi a "lissajous", mas pode ser customizada para 
# qualque curva em /fuzzy_control/core/trajectories.py.
LISSAJOUS = dict(
    centro=[0.55, 0.00, 0.50],
    amplitude=[0.00, 0.30, 0.20],          # [m]  (x, y, z)
    frequencia=[0.0, 3 / 20, 2 / 20],      # [Hz] -> figura completa a cada 20 s
    fase=[0.0, np.pi / 2, 0.0],            # [rad]
)

# --- "pontos": o EE visita um ponto por vez, trocando a cada TEMPO_POR_ALVO s
ALVOS = [
    np.array([0.55,  0.20, 0.50]),
    np.array([0.45, -0.20, 0.65]),
    np.array([0.60,  0.00, 0.40]),
    np.array([0.55,  0.00, 0.62]),
]

TEMPO_POR_ALVO = 4.0  # [s]

# ---------------------------------------------------------------------------
# Cinemática inversa (velocidade do EE -> velocidade das juntas)
# Controle de baixo nivel
# ---------------------------------------------------------------------------
LAMBDA_DLS = 0.05

# O braço tem 7 juntas, mas só controlamos 3 coisas (x, y, z). As juntas
# "sobrando" (espaço nulo) puxam o braço de volta para a pose home com este
# ganho, para ele não ir parar em posturas estranhas. 0 desliga.
GANHO_ESPACO_NULO = 1.0

# ---------------------------------------------------------------------------
# Visualização / log
# ---------------------------------------------------------------------------
RAIO_MARCADOR = 0.02                  # esfera do alvo [m]
COR_MARCADOR = (1.0, 0.0, 0.0, 0.6)   # RGBA
COR_CAMINHO = (0.2, 0.6, 1.0, 1.0)    # caminho desejado (azul)
COR_RASTRO = (1.0, 0.8, 0.0, 1.0)     # caminho real do EE (amarelo)
DURACAO_RASTRO = 6.0                  # quantos segundos de rastro mostrar
INTERVALO_PRINT = 0.5                 # imprime o erro a cada X s de simulação

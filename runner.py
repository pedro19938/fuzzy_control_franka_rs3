"""
Roda o controle fuzzy do EE do Franka FR3 com o viewer do MuJoCo.

    fuzzy/bin/python runner.py

Configurações: fuzzy_control/configs/
  - trajetória (pontos / lissajous) e feedforward: sim_config.py
  - conjuntos e regras fuzzy: fuzzy_rules.py
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
from fuzzy_control.core.trajectories import criar_trajetoria
from fuzzy_control.utils.viewer_utils import (Rastro, criar_marcador, desenhar_caminho,
                                              esperar_tempo_real, mover_marcador)

PASSOS_POR_FRAME_RASTRO = 20  # atualiza o rastro a cada 20 passos (25 Hz)

# funcao auxiliar para encerrar o loop de controle com ctrl+c
def instalar_ctrl_c():
    
    parar = threading.Event()

    def ao_apertar(sig, frame):
        print("\nCtrl+C: encerrando... (aperte de novo para forçar)")
        parar.set()
        signal.signal(signal.SIGINT, signal.SIG_DFL)  # próximo Ctrl+C mata direto

    signal.signal(signal.SIGINT, ao_apertar)
    return parar


def main() -> None:
    # modelo mujoco
    model = mm.load(cfg.MODELO)
    data = mujoco.MjData(model)
    
    # resetar para a pose inicial do keyframe
    mujoco.mj_resetDataKeyframe(model, data, cfg.KEYFRAME_INICIAL)
    mujoco.mj_forward(model, data)

    # controlador fuzzy
    controlador = EEFuzzyController(model, data)
    trajetoria = criar_trajetoria(cfg)

    # numero de passos para o rastro (duração / timestep)
    n_rastro = int(cfg.DURACAO_RASTRO / (model.opt.timestep * PASSOS_POR_FRAME_RASTRO))
    parar = instalar_ctrl_c()

    # Loop principal
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # inicializa os marcadores visuais no viwer (trajetoria, rastro, alvo)
        with viewer.lock():
            criar_marcador(viewer, trajetoria(0.0)[0], cfg.RAIO_MARCADOR, cfg.COR_MARCADOR)
            desenhar_caminho(viewer, trajetoria.caminho(), cfg.COR_CAMINHO)
            rastro = Rastro(viewer, n_rastro, cfg.COR_RASTRO)

        # Loop de controle
        passo = 0
        ultimo_print = 0.0
        while viewer.is_running() and not parar.is_set():
            inicio = time.time()

            alvo, vel_alvo = trajetoria(data.time)
            erro = controlador.passo(alvo, vel_alvo)
            mujoco.mj_step(model, data)
            passo += 1

            # atualiza o viewer
            with viewer.lock():
                mover_marcador(viewer, alvo)
                if passo % PASSOS_POR_FRAME_RASTRO == 0:
                    rastro.adicionar(controlador.posicao_ee())
            viewer.sync()

            # printa o erro a cada cfg.INTERVALO_PRINT segundos
            if data.time - ultimo_print >= cfg.INTERVALO_PRINT:
                ultimo_print = data.time
                print(f"t={data.time:5.1f}s  erro (mm) = {np.round(erro * 1000, 1)}"
                      f"  |erro| = {np.linalg.norm(erro) * 1000:5.1f} mm")

            # roda na frequencia real (model.opt.timestep)
            esperar_tempo_real(inicio, model.opt.timestep)

# main
if __name__ == "__main__":
    main()

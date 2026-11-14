# Fuzzy End-Effector Control — Franka Research 3 (MuJoCo)

Mamdani fuzzy controller that drives the end-effector of a Franka Research 3
arm along a target trajectory in MuJoCo. The fuzzy engine is written from
scratch in NumPy (no fuzzy library), so every stage — fuzzification,
inference, aggregation and defuzzification — is visible in the code.

## How it works

```
trajectory(t) ──► position error (x, y, z) ──► FUZZY (per axis) ──► EE velocity
      │                                                                │
      └──────────── desired velocity (feedforward, optional) ─────────►+
                                                                       │
                                            damped least-squares Jacobian inverse
                                                                       │
                                     joint velocities ──► integrated ──► position servos
```

1. **Error** — target position minus the current position of `attachment_site` (the flange).
2. **Fuzzy controller** — the same single-input/single-output controller runs on each
   axis and maps the position error (m) to an end-effector velocity (m/s).
3. **Feedforward (optional)** — the trajectory's own velocity is added, so the fuzzy
   controller only has to correct the remaining deviation.
4. **Inverse kinematics** — a damped pseudo-inverse of the position Jacobian turns the
   Cartesian velocity into joint velocities. The 3 redundant DOF (7 joints, 3 controlled
   coordinates) are used in the null space to keep the arm close to its home posture.
5. **Actuation** — the FR3 model uses position servos, so joint velocities are integrated
   into joint-angle references and written to `data.ctrl`.

### The fuzzy controller

| Stage | Implementation |
|---|---|
| Fuzzification | 5 sets on the error: triangles in the middle, trapezoids that saturate at ±`E_MAX` |
| Inference | one rule per set, `min` implication |
| Aggregation | `max` |
| Defuzzification | centroid over a 501-point discretized output universe |

Linguistic labels: **NG** negative big · **NP** negative small · **ZE** zero ·
**PP** positive small · **PG** positive big.

| IF error is | NG | NP | ZE | PP | PG |
|---|---|---|---|---|---|
| **THEN velocity is** | NG | NP | ZE | PP | PG |

Sets, rules and their meaning are documented in
[`fuzzy_control/configs/fuzzy_rules.py`](fuzzy_control/configs/fuzzy_rules.py).

## Results

Lissajous curve (3:2, 60 × 40 cm in the y–z plane, 20 s period), error measured over
the second period, after the initial transient:

| Controller | Mean error | Max error |
|---|---|---|
| Fuzzy feedback only | 57.7 mm | 91.4 mm |
| Fuzzy + feedforward | **4.4 mm** | **5.6 mm** |

Feedback alone can only produce velocity when there is error, so it always lags a
moving target. Adding the trajectory velocity as feedforward removes most of that lag.

## Project structure

```
runner.py                     entry point: viewer + control loop
fuzzy_control/
├── configs/
│   ├── fuzzy_rules.py        fuzzy sets and rules (the controller's "knowledge")
│   └── sim_config.py         robot, trajectory, feedforward, IK gains, visuals
├── core/
│   ├── membership.py         triangular / trapezoidal membership functions
│   ├── fuzzy_siso.py         Mamdani engine (fuzzify → infer → aggregate → centroid)
│   ├── ee_controller.py      fuzzy + Jacobian → joint commands
│   └── trajectories.py       fixed waypoints and 3D Lissajous curves
└── utils/
    └── viewer_utils.py       target marker, desired path, EE trail, real-time pacing
```

## Installation

Requires Python 3.12. The robot model is downloaded automatically by
[`mujoco-menagerie`](https://github.com/google-deepmind/mujoco_menagerie) on first run.

```bash
git clone git@github.com:pedro19938/fuzzy_control_frank_rs3.git
cd fuzzy_control_frank_rs3
python3 -m venv fuzzy
fuzzy/bin/pip install -r requirements.txt
```

## Usage

```bash
fuzzy/bin/python runner.py
```

The MuJoCo viewer shows:

- **red sphere** — current target
- **blue line** — desired path
- **yellow line** — actual end-effector trail (last 6 s)

The terminal prints the tracking error every 0.5 s. Press `Ctrl+C` once to stop cleanly
(twice to force quit), or close the window.

> The viewer needs a display. Over SSH, run it from a terminal with access to the
> machine's screen (e.g. directly on the computer running the simulation).

## Configuration

Everything is set in `fuzzy_control/configs/` — no changes to `core/` are needed.

**`sim_config.py`**

| Parameter | Description |
|---|---|
| `TRAJETORIA` | `"lissajous"` or `"pontos"` (fixed waypoints) |
| `USAR_FEEDFORWARD` | add the trajectory velocity to the fuzzy output |
| `LISSAJOUS` | center, amplitude, frequency and phase per axis |
| `ALVOS`, `TEMPO_POR_ALVO` | waypoints and how long to stay at each one |
| `LAMBDA_DLS` | damping of the Jacobian pseudo-inverse |
| `GANHO_ESPACO_NULO` | pull towards the home posture (0 disables it) |

**`fuzzy_rules.py`**

| Parameter | Description |
|---|---|
| `E_MAX` | error at which the input sets saturate (smaller = more aggressive) |
| `V_MAX` | output velocity range |
| `CONJUNTOS_ENTRADA`, `CONJUNTOS_SAIDA` | membership functions, as `("triangulo", (a, b, c))` or `("trapezio", (a, b, c, d))` |
| `REGRAS` | list of `(input set, output set)` pairs |
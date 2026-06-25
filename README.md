# AEBS Simulation & Verification Framework

## Project Overview
This repository implements a simulation and verification framework for **Autonomous Emergency Braking Systems (AEBS)**. It models the system as a **Stochastic Hybrid System (SHS)** with continuous physical dynamics, discrete control logic, and an exogenous lead vehicle, and over-approximates it as a sound **Interval Markov Decision Process (IMDP)** for formal verification.

The project runs on two tracks that share the same plant and controller:

* **Simulation track:** an interactive continuous-time simulator for inspecting scenarios and comparing controllers.
* **Verification track:** a discretisation pipeline that builds a provably sound IMDP and exports it in the PRISM modelling language for robust probabilistic model checking (in Storm).

## Verification Status & Scope
The work is structured as a three-track research programme. Only Track 1 is implemented today, and the README is explicit about that so claims are not overstated.

* **Track 1 (current): sound IMDP under perfect perception.** Implemented and certified for the static and steady lead models. Perception is symbolic (perfect).
* **Track 2 (planned): perception-uncertainty injection into the abstraction.** The simulator already has a working sensor model (100 m range cutoff, configurable false-negative rate, optional Gaussian position/velocity noise), but the abstraction-side perception is still perfect; injecting perception uncertainty into the IMDP, and replacing the hand-set sensor model with a calibrated DNN, is future work. Note that despite the "DNN" naming in the code, no neural network is present yet.
* **Track 3 (planned): runtime monitor synthesis.**
* **Unpredictable (nondeterministic) lead:** defined in code but not yet wired through the discretiser or PRISM composition; the wrapper rejects it until the second action axis is built.

## Key Features
* **Modular construction:** decoupled Plant (physics), Controller (logic), Perception (sensor model), and an injectable Lead behaviour, composed in the exported model via a turn-based scheduler (`perceive` then `control` then `time_step`).
* **Sound IMDP abstraction:** deterministic dimensions (gap, v_ego) use exact image-box enumeration; the stochastic dimension (v_lead) uses a truncated-Gaussian range bound. Per-transition soundness is checked exhaustively.
* **Injectable lead-behaviour models:** static, steady, and (planned) unpredictable leads, mapped to Euro NCAP CCR scenarios. One IMDP per lead behaviour.
* **Dual control logic** (reactive; switches on closing speed, using distance triggers when `v_rel < 5 m/s` and Time-to-Collision triggers otherwise):
  * **Safe Mode** (used for verification, brakes early): TTC warn / brake / emergency = 6.0 / 5.0 / 4.0 s; distance = 15 / 10 / 5 m; decel = -4 / -8 m/s².
  * **Industry Mode** (brakes late): TTC = 2.6 / 1.6 / 1.0 s; distance = 10 / 6 / 2 m; decel = -4 / -9.8 m/s².
* **PRISM-language export:** artifacts are written in the PRISM modelling language and verified with **Storm** against PCTL properties such as `Pmax=? [ F "crash" ]`.

## System Dynamics
The abstraction operates in the **relative frame** with state `(gap [m], v_ego [m/s], v_lead [m/s])`, where dimensions 0 and 1 are deterministic and dimension 2 (v_lead) carries Gaussian noise. The one-step update (`dt = 0.1 s`, `alpha = 0.5`) is:

* **Ego velocity:** first-order lag from a zero baseline each step, `a_ego = alpha * u`, then `v_ego' = max(0, v_ego + a_ego * dt)`. The half-gain models actuation lag and is conservative (it under-applies the commanded deceleration in the first step).
* **Lead velocity:** exogenous and not controller-driven. The static and steady models hold speed, `v_lead' = max(0, v_lead)`. Lead acceleration uncertainty enters as Gaussian noise on v_lead; combining the process noise (0.05) and the lead noise (2.0) and scaling by `dt` gives `sigma_on_velocity` approximately 0.2 m/s.
* **Gap:** `gap' = gap + (v_lead' - v_ego') * dt`.

The lead behaviour is supplied by an injectable `LeadModel`, which also owns the v_lead grid band, the noise, the acceleration choice set, and the scenario list.

## Benchmarks & Grounding
* **Control logic:** the hierarchical strategy (warning, brake, emergency) and the Time-to-Collision and distance thresholds follow the industry-standard [MathWorks AEB benchmark](https://www.mathworks.com/help/driving/ug/autonomous-emergency-braking-with-sensor-fusion.html).
* **Scenarios:** lead behaviours and approach speeds follow the [Euro NCAP Safety Assist protocols](https://www.euroncap.com/en/for-engineers/protocols/safety-assist/). The lead models map onto the Euro NCAP Car-to-Car Rear cases: CCRs (stationary), CCRm (moving), CCRb (braking).

## Verification Methodology
The continuous SHS is over-approximated by a **sound IMDP**, and soundness is established **per transition** rather than via a global additive error bound.

* **Image-box kernel:** for each source cell and action, the deterministic image is bounded by evaluating the monotone dynamics at the cell corners. Every target cell the image box touches receives an interval, so a cell whose image straddles a grid boundary (including the crash wall at `gap = 0`) routes mass to both sides. This replaced an earlier cell-centre routing that could record genuinely reachable cells as unreachable.
* **Per-transition containment:** the true transition probability from every continuous state in a source cell lies inside the stored `[lo, hi]` interval. This is the soundness condition, and it is certified exhaustively by `tests/test_containment_full.py`, which samples every cell and action against the stored kernel and reports any excess.
* **Formal preservation:** soundness follows the interval-MC / Markov-set-chain framework (D'Innocenzo, Abate & Katoen, 2012): per-transition containment discharges the soundness hypothesis, and the model checker's robust value iteration returns probability brackets that provably contain the truth.
* **Diagnostic only:** the SA13 additive bound `E = N * K` is still computed and printed, but it is structurally vacuous for this system (noise enters only one dimension), so it is reported as descriptive and is **not** the certificate.

Methodological references: Soudjani & Abate (2013) abstraction framework; D'Innocenzo, Abate & Katoen (2012) interval-MC containment; Lavaei, Soudjani, Abate & Zamani (2022) survey. Full bibliographic details are maintained in the accompanying paper.

## Installation

```bash
# Clone the repository
git clone https://github.com/hebaalkayed/AEBS-Stochastic-Hybrid-Discretisation.git
cd AEBS-Stochastic-Hybrid-Discretisation

# Create a virtual environment
python -m venv .venv
source .venv/bin/activate      # On Windows: .venv\Scripts\activate

# Install Python dependencies
pip install numpy matplotlib
```

* `numpy` is required by the abstraction pipeline; `matplotlib` by the simulator's plotting.
* **Storm** (the probabilistic model checker) must be installed separately to verify the exported `.prism` artifacts.
* `torch` is not required by the current code; it would be needed only if the planned calibrated DNN perception (Track 2) is added.

## Usage

### 1. Continuous simulation (inspect scenarios, compare controllers)
```bash
python experiments/run_simulation.py
```
The script is interactive. Follow the prompts:
1. **Lead behaviour:** `1. Static` (wall / stopped car), `2. Steady` (constant speed), `3. Unpredictable` (erratic braking).
2. **Physics:** initial gap (m), ego velocity (m/s), and (if non-static) lead velocity (m/s).
3. **Controller:** `1. Industry` (late braking) or `2. Safe` (early braking).
4. A plot of gap and velocities appears; close it to run another scenario.

The lead behaviour is the "ground truth" the environment generates each step:

| Behaviour | Lead dynamics | Notes |
| :--- | :--- | :--- |
| **Static** | Lead held at 0 m/s | Stationary obstacle (wall or stopped car). |
| **Steady** | Constant speed (set at init) | Lead holds its initial velocity. |
| **Unpredictable** | Gaussian-walk acceleration (std 2.0) with a per-step jerk event (probability 0.05) firing a hard brake (-8 m/s²) or hard accel (+5 m/s²); speed clamped to [0, 50] m/s | Stochastic sampling oracle for stress-testing the controller. It is **not** the same object as the abstraction's `UnpredictableLead`, which is a nondeterministic choice set; the simulator samples, the model quantifies. |

### 2. Generate the IMDP / PRISM artifact (verification track)
```bash
python experiments/run_modular_abstraction.py
```
* Choose the grid `grid_preset` (for example `medium_tight` for the production grid, `coarse` for a fast check) and the lead model (`lead_model='static'`) inside the script.
* **Output:** a PRISM-language model at the configured `output_path` (default `artifacts/<prefix>_modular_system.prism`).
* The run prints a **scenario cheat sheet** mapping each named scenario to its `start_s` state ID.

### 3. Certify soundness
```bash
python -m tests.test_containment_full
```
* Set `PRESET` and the `LEAD_MODEL` environment variable to match the grid and lead model you generated.
* The check is parallel, streaming, persistent, and resumable (it appends progress to a CSV and skips completed chunks on restart). A clean run reports zero violations.

### 4. Model check in Storm
Load the exported `.prism` file in Storm and check, for example:
```
Pmax=? [ F "crash" ]
```
passing the scenario's initial state as a constant, `start_s=<ID>`, using an ID from the cheat sheet printed during generation.

## Scientific Context
This repository is the "concrete system" in the abstraction-refinement loop.
* **Input:** the continuous state space of the AEBS plant.
* **Process:** sound grid-based discretisation into an Interval MDP.
* **Output:** an IMDP whose transition intervals bracket the true probabilities.
* **Goal:** establish that if the abstract IMDP is safe under robust model checking, the concrete SHS is guaranteed safe.
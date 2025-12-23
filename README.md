# AEBS Simulation & Verification Framework

## Project Overview
This repository implements a high-fidelity simulation and verification framework for **Autonomous Emergency Braking Systems (AEBS)**. It models the system as a **Stochastic Hybrid System (SHS)**, incorporating continuous physical dynamics, discrete control logic, and stochastic perception uncertainty.

The primary goal of this project is to serve as a ground-truth benchmark for **Formal Verification**. It supports the implementation of abstraction algorithms (based on recent surveys) to discretize the continuous state space into **Interval Markov Decision Processes (IMDPs)**, enabling **Assume-Guarantee reasoning** with respect to strict safety specifications.

## Key Features
* **Modular Architecture:** Decoupled Plant (Physics), Controller (Logic), Perception (Sensor Model), and Environment.
* **Industry Benchmarks:** Physics and scenarios derived from **Euro NCAP** and **NHTSA** protocols.
* **Stochastic Perception:** Configurable Neural Network uncertainty (False Negative Rates) to model "AI blindness."
* **Dual Control Logic:**
    * **Industry Mode:** Standard "Impact Mitigation" logic (brakes late, high efficiency).
    * **Safe Mode:** Formally verifiable "Collision Avoidance" logic (brakes early, high safety).

## Simulation Scenarios
The environment module supports 5 critical testing scenarios standard in the automotive industry:

| Scenario | Dynamics | Source | Description |
| :--- | :--- | :--- | :--- |
| **Urban Static** | $v_{ego}=10m/s$ | Euro NCAP City | Standard approach to a parked car (~36 km/h). |
| **Urban Cut-Out** | $v_{ego}=10m/s$ | Research | Obstacle appears suddenly at 15m gap (TTC=1.5s). |
| **Highway Static** | $v_{ego}=25m/s$ | Euro NCAP Inter-Urban | High-speed approach to static wall (~90 km/h). |
| **Highway Cut-Out** | $v_{ego}=25m/s$ | Engineering Stress Test | Obstacle appears at 40m gap. The physical limit of braking. |
| **Highway Traffic** | Dynamic | Real-world | Lead vehicle performs unpredictable acceleration/braking. |

## Physics & Kinematics Sources
To ensure the validity of verification results, the vehicle dynamics are calibrated using real-world commercial vehicle data:

1. **Vehicle Dynamics (N1 Class):**
    * Max Deceleration: $-9.8 m/s^2$ (1g) on dry asphalt.
    * Response Model: System is modeled as a discrete-time Linear Time-Invariant (LTI) system with simulation step $dt=0.2s$.
2. **Controller Logic:**
    * *Industry Mode* follows **MathWorks AEBS** and **Bosch** specifications (TTC < 1.0s trigger).
    * *Safe Mode* utilizes conservative braking profiles suitable for **Safety-Critical Verification**.

## Installation & Usage

### 1. Setup
```bash
# Clone the repository
git clone <your-repo-url>
cd <your-repo-name>

# Create a virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install numpy matplotlib torch
```

### 2. Running the Simulation
The main experiment script is interactive. It allows you to mix and match scenarios, controllers, and noise levels.

```bash
python experiments/run_full_comparison.py
```

Follow the on-screen prompts:
1.  **Choose Scenario:** (e.g., `highway_cutout`)
2.  **Choose Controller:** (`industry` for baseline, `safe` for verified logic)
3.  **Set Noise:** (`0.0` for perfect sensors, `0.5` for 50% sensor failure rate)

## Project Structure
```text
├── src/
│   ├── system/
│   │   ├── vehicle_plant.py    # Physics Engine (Kinematics)
│   │   ├── controller.py       # AEBS Logic (Industry vs Safe)
│   │   ├── perception.py       # Sensor Abstraction (Confusion Matrix)
│   │   └── environment.py      # Scenario & Lead Vehicle Management
│   └── visualiser/
│       └── plotter.py          # Visualization & Graphing Module
├── experiments/
│   └── run_full_comparison.py  # Interactive CLI for running tests
└── README.md
```

## Scientific Context
This repository represents the "Concrete System" in the Abstraction-Refinement loop. 
* **Input:** Continuous State Space $\mathbb{R}^n$.
* **Process:** Discretization via grid-based abstraction.
* **Output:** Interval MDP representing lower/upper bounds of transition probabilities.
* **Goal:** Prove that if the abstract IMDP is safe, the concrete SHS is guaranteed to be safe.
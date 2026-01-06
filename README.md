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
* **PRISM Integration:** Exports .prism artifacts ready for PCTL model checking (e.g., Pmax=? [ F "crash" ]).

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
To ensure the validity of verification results, the system dynamics and testing protocols are grounded in established industry and academic benchmarks:

* **Vehicle Dynamics:** The continuous plant model is defined by the **ARCH-COMP20 AINNCS Benchmark** for Adaptive Cruise Control (ACC). It models the vehicle as a point mass with first-order acceleration lag dynamics:
    $$\dot{a}(t) = -2a(t) + 2u(t) - \mu v^2(t)$$
    See: *Johnson et al., ["ARCH-COMP20 Category Report: AINNCS"](https://easychair.org/publications/paper/Jvwg), 2020.*
* **Control Logic:** The hierarchical control strategy (Warning $\to$ Partial $\to$ Emergency) and Time-to-Collision (TTC) thresholds are derived from the industry-standard [MathWorks AEB Benchmark](https://www.mathworks.com/help/driving/ug/autonomous-emergency-braking-with-sensor-fusion.html).
* **Testing Scenarios:** Simulation speeds (10–80 km/h) and obstacle configurations (e.g., *Urban Cut-Out*, *Highway Static*) are defined according to the official [Euro NCAP Safety Assist Protocols](https://www.euroncap.com/en/for-engineers/protocols/safety-assist/).

## Installation & Usage

### 1. Setup
```bash
# Clone the repository
git clone https://github.com/hebaalkayed/AEBS-Stochastic-Hybrid-Discretisation.git
cd AEBS-Stochastic-Hybrid-Discretisation

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

### Follow the on-screen prompts:

1.  **Select Lead Behavior:**
    * `1. Static`: A wall or stopped car (tests collision avoidance).
    * `2. Steady`: A lead vehicle moving at constant speed (tests ACC/tailgating).
    * `3. Unpredictable`: An unpredictable driver (tests reaction time to erratic braking).

2.  **Configure Physics:**
    * Input the **Initial Gap** (meters) and **Ego Velocity** (m/s).
    * *(If non-static)* Input the **Lead Velocity** (m/s).

3.  **Select Controller:**
    * `1. Industry`: Uses standard Euro NCAP thresholds (brakes late, prioritizes comfort).
    * `2. Safe`: Uses the Formally Verified logic (brakes early, prioritizes safety guarantees).

4.  **Analyze & Repeat:**
    * A graph will appear showing **Gap (Blue)** and **Velocity (Orange)**.
    * Close the graph window to immediately run a new scenario (e.g., to test the same physics against a different controller).

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

## Workflow & Verification
This script runs the physics engine, discretizes the state space, calculates transition probabilities (integrating noise), and exports the Hybrid MDP.

```bash
python tests/verify_relative_model.py
```

* **Input:** src/system/vehicle_plant.py (Physics) + src/system/controller.py (Logic).
* **Output:** artifacts/relative_model.prism
* **Verify in PRISM:**
1.  **open PRISM**
2.  **Load artifacts/relative_model.prism**
3.  **Go to the Properties tab and run: Pmax=? [ F "crash" ]**

## Scientific Context
This repository represents the "Concrete System" in the Abstraction-Refinement loop. 
* **Input:** Continuous State Space $\mathbb{R}^n$.
* **Process:** Discretization via grid-based abstraction.
* **Output:** Interval MDP representing lower/upper bounds of transition probabilities.
* **Goal:** Prove that if the abstract IMDP is safe, the concrete SHS is guaranteed to be safe.
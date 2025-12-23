import sys
import os
import numpy as np

# Path Setup
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from system.vehicle_plant import BenchmarkAV
from system.controller import AEBSController
from system.perception import PerceptionSystem
from system.environment import TrafficEnvironment
from visualiser.plotter import SimulationPlotter  # <--- NEW MODULE

def run_test(scenario, mode, nn_noise, belief_noise, label):
    dt = 0.2
    plant = BenchmarkAV(dt=dt)
    env = TrafficEnvironment(dt=dt)
    
    # 1. Setup
    x = env.reset(scenario)
    controller = AEBSController(dt=dt, mode=mode, belief_noise_std=belief_noise)
    controller.est_velocity = env.ego_speed 
    nn = PerceptionSystem(false_negative_rate=nn_noise)

    print(f"\n--- {label} ---")
    print(f"Init: Gap={x[0]}m | Ego={x[1]}m/s | Lead={x[2]}m/s")
    
    history = {'time': [], 'dist': [], 'ego_v': [], 'lead_v': []}
    crashed = False
    stop_idx = None
    crash_idx = None

    # 2. Simulation Loop
    for step in range(200): # 40 seconds
        t = step * dt
        
        # Loop Logic
        obs = nn.observe(x[0])
        u_ego = controller.get_action(obs, x[0])
        u_lead = env.get_lead_action()
        x = plant.get_next_state(x, u_ego, u_lead)
        
        # Logging
        history['time'].append(t)
        history['dist'].append(x[0])
        history['ego_v'].append(x[1])
        history['lead_v'].append(x[2])
        
        # Event Detection
        if x[0] <= 0:
            print(f"!!! CRASH at T={t:.1f}s (Impact Vel: {x[1]:.2f}) !!!")
            crashed = True
            crash_idx = len(history['time']) - 1
            break
        
        if x[1] < 0.05 and stop_idx is None:
            stop_idx = len(history['time']) - 1
            print(f"Safe Stop at T={t:.1f}s (Gap: {x[0]:.2f}m)")

    # 3. Visualization (Delegated)
    SimulationPlotter.plot_scenario(history, label, stop_idx, crash_idx)
    
    return crashed

# --- USER INPUT HELPERS ---
def get_choice(prompt, options):
    print(f"\n{prompt}")
    for i, opt in enumerate(options):
        print(f"  {i+1}. {opt}")
    idx = int(input("Select number: ")) - 1
    return options[idx]

if __name__ == "__main__":
    print("=== AEBS SCENARIO SIMULATION ===")
    scenarios = ['urban_static', 'urban_cutout', 'highway_static', 'highway_cutout', 'highway_traffic']
    
    while True:
        scen = get_choice("Choose Scenario:", scenarios)
        mode = get_choice("Controller Mode:", ['industry', 'safe'])
        
        noise_in = input("Perception Noise (0.0 - 1.0) [Default 0.1]: ")
        nn_noise = 0.1 if noise_in == "" else float(noise_in)
        
        label = f"{scen.upper()} | {mode.upper()} | NN={nn_noise}"
        
        # Notice how clean this call is now!
        run_test(scen, mode, nn_noise, 0.0, label)
        
        if input("\nRun again? (y/n): ") != 'y': break
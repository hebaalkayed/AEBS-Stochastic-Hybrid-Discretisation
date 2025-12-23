import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Path Setup
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '..', 'src'))

from system.vehicle_plant import BenchmarkAV
from system.controller import AEBSController
from system.perception import PerceptionSystem

def run_test_case(mode_name, color):
    dt = 0.2
    plant = BenchmarkAV(dt=dt)
    
    # Select the Controller Mode
    controller = AEBSController(dt=dt, mode=mode_name)
    controller.est_velocity = 25.0
    
    # Perfect Perception (to isolate controller logic)
    nn = PerceptionSystem(false_negative_rate=0.0)

    # INITIAL STATE: 25 m/s (~90 km/h) with 100m Gap
    x = np.array([100.0, 25.0, 0.0])
    
    history = {'time': [], 'dist': [], 'vel': []}
    crashed = False

    for step in range(100): # 20 seconds
        t = step * dt
        obs = nn.observe(x[0])
        u = controller.get_action(obs, x[0])
        x = plant.get_next_state(x, u, noise_scale=0.0)
        
        history['time'].append(t)
        history['dist'].append(x[0])
        history['vel'].append(x[1])
        
        if x[0] <= 0:
            crashed = True
            break
            
    final_status = "CRASHED" if crashed else "SAFE"
    print(f"Mode: {mode_name.upper():<10} | Result: {final_status} | Final Gap: {x[0]:.2f}m | Impact Vel: {x[1]:.2f}m/s")
    
    plt.plot(history['time'], history['dist'], color=color, linewidth=2, label=f"{mode_name.upper()} (Standard)")
    return history

if __name__ == "__main__":
    plt.figure(figsize=(10, 6))
    print("--- INDUSTRY vs SAFE MODE COMPARISON (90 km/h) ---")
    
    # Run Industry Standard (MathWorks)
    run_test_case('industry', 'red')
    
    # Run Verification Safe (Our Calculation)
    run_test_case('safe', 'green')
    
    plt.axhline(0, color='k', linewidth=3, label='Crash Line')
    plt.title('AEBS Logic Comparison: Impact Mitigation vs. Avoidance')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.grid(True)
    plt.show()
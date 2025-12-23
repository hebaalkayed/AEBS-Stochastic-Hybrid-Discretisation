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

def run_static_obstacle_test(noise_level, label, color):
    """
    Runs a single simulation against a STATIC WALL with a specific NN Noise level.
    noise_level: Probability of False Negative (missing the car)
    """
    # 1. Setup Modules
    dt = 0.2
    plant = BenchmarkAV(dt=dt)
    
    # PARAMETER: Control the Perception Noise here
    nn = PerceptionSystem(false_negative_rate=noise_level, false_positive_rate=0.0)
    
    # Use standard controller (perfect belief to isolate perception error)
    controller = AEBSController(dt=dt, belief_noise_std=0.0)
    
    # 2. Initial State [Gap=100m, Ego=25m/s, Lead=0m/s (STATIC)]
    x = np.array([100.0, 25.0, 0.0])
    controller.est_velocity = 25.0
    
    history = {'time': [], 'dist': []}
    
    print(f"Running Case: {label:<20} | Noise (FN)={noise_level*100:>3.0f}%")
    
    # 3. Simulation Loop
    crashed = False
    for step in range(100): # 20 seconds
        t = step * dt
        
        # Perception
        obs = nn.observe(x[0])
        
        # Control
        u = controller.get_action(obs, x[0])
        
        # Physics (noise_scale=0.0 means STATIC/STEADY physics, no random driver)
        x = plant.get_next_state(x, u, noise_scale=0.0)
        
        history['time'].append(t)
        history['dist'].append(x[0])
        
        if x[0] <= 0:
            print(f"  -> !!! CRASH at {t:.1f}s !!!")
            crashed = True
            break
    
    if not crashed:
        print(f"  -> SAFE (Final Gap: {x[0]:.2f}m)")
        
    # Plot this run
    plt.plot(history['time'], history['dist'], color=color, linewidth=2, label=f"{label} ({noise_level*100:.0f}%)")

if __name__ == "__main__":
    plt.figure(figsize=(10, 6))
    
    print("--- STATIC OBSTACLE PERCEPTION TEST ---")
    
    # CASE 1: Perfect Vision (Baseline)
    run_static_obstacle_test(0.00, "Perfect Vision", "green")
    
    # CASE 2: Low Noise (Real World Standard)
    run_static_obstacle_test(0.10, "Low Noise", "blue")
    
    # CASE 3: High Noise (Bad Weather/Sensor)
    run_static_obstacle_test(0.40, "High Noise", "orange")
    
    # CASE 4: Very High Noise (Failure Mode)
    run_static_obstacle_test(0.80, "Severe Noise", "red")

    # Final Plot Formatting
    plt.axhline(0, color='k', linewidth=3, linestyle='-', label='Crash Line')
    plt.title('Impact of Perception Noise on AEBS Safety (Static Obstacle)')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance to Wall (m)')
    plt.legend()
    plt.grid(True)
    plt.show()
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

# --- SCENARIO DEFINITIONS ---

# 1. PERCEPTION CASES (The Eyes)
PERCEPTION_CASES = {
    'good':      {'fn': 0.00, 'fp': 0.00, 'desc': 'Oracle Vision'},
    'medium':    {'fn': 0.10, 'fp': 0.05, 'desc': '10% Error Rate'},
    'blind':     {'fn': 1.00, 'fp': 0.00, 'desc': 'Totally Blind'},
}

# 2. TRAFFIC CASES (The Physics)
TRAFFIC_CASES = {
    'static':   {'v_lead': 0.0,  'noise': 0.0, 'desc': 'Static Wall'},
    'steady':   {'v_lead': 15.0, 'noise': 0.0, 'desc': 'Constant Speed'},
    'crazy':    {'v_lead': 15.0, 'noise': 1.0, 'desc': 'Stochastic Driver'},
}

# 3. BELIEF CASES (The Brain/Estimation) - NEW!
BELIEF_CASES = {
    'perfect':  {'std': 0.0, 'desc': 'Perfect Internal Model'},
    'noisy':    {'std': 1.0, 'desc': 'Noisy Estimation (High Variance)'},
}

def run_specific_test(perc_name, traffic_name, belief_name='perfect'):
    # Load Configs
    p_conf = PERCEPTION_CASES[perc_name]
    t_conf = TRAFFIC_CASES[traffic_name]
    b_conf = BELIEF_CASES[belief_name]
    
    print(f"\n>>> TEST: Eyes='{perc_name}' | Traffic='{traffic_name}' | Belief='{belief_name}'")
    
    # Setup Modules (Pass the new noise parameter!)
    dt = 0.2
    plant = BenchmarkAV(dt=dt)
    nn = PerceptionSystem(false_negative_rate=p_conf['fn'], false_positive_rate=p_conf['fp'])
    controller = AEBSController(dt=dt, belief_noise_std=b_conf['std'])
    
    # Initial State [Gap=100m, Ego=25m/s, Lead=...]
    # I increased Gap to 100m to give the car time to react!
    x = np.array([100.0, 25.0, t_conf['v_lead']])
    
    # Sync belief at start
    controller.est_velocity = 25.0
    
    history = {'time': [], 'dist': []}
    
    crashed = False
    for step in range(100): # 20 seconds
        t = step * dt
        
        # Perception
        obs = nn.observe(x[0])
        
        # Control (Uses Belief internally)
        u = controller.get_action(obs, x[0])
        
        # Physics
        x = plant.get_next_state(x, u, noise_scale=t_conf['noise'])
        
        history['time'].append(t)
        history['dist'].append(x[0])
        
        if x[0] <= 0:
            print(f"!!! CRASH at T={t:.1f}s !!!")
            crashed = True
            break
            
    status = "CRASHED" if crashed else "SAFE"
    print(f"    Result: {status} (Final Gap: {x[0]:.2f}m)")
    return history, status

if __name__ == "__main__":
    # --- RUN THE COMPARISON ---
    
    # 1. The Baseline: Everything Perfect (Should stay SAFE now)
    h1, r1 = run_specific_test('good', 'static', 'perfect')
    
    # 2. The Challenge: Blind Perception (Should CRASH)
    h2, r2 = run_specific_test('blind', 'static', 'perfect')
    
    # 3. The New Feature: Noisy Beliefs (Might CRASH or act weird)
    h3, r3 = run_specific_test('good', 'static', 'noisy')

    # Plot
    plt.figure(figsize=(10, 6))
    plt.plot(h1['time'], h1['dist'], 'g-', linewidth=2, label='Perfect System')
    plt.plot(h2['time'], h2['dist'], 'r--', linewidth=2, label='Blind System')
    plt.plot(h3['time'], h3['dist'], 'b:', linewidth=2, label='Noisy Beliefs')
    
    plt.axhline(0, color='k', label='Crash')
    plt.title('Impact of Perception & Belief Errors')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.grid(True)
    plt.show()
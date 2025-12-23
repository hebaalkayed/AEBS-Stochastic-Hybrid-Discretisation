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

# --- COLOR CODES ---
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def get_state_color(state):
    if state == 'drive': return Colors.GREEN
    if state == 'coast': return Colors.BLUE
    if state == 'brake': return Colors.YELLOW
    if state == 'emergency_brake': return Colors.RED
    return Colors.RESET

def run_simulation():
    # 1. Initialize Modules
    dt = 0.2
    plant = BenchmarkAV(dt=dt)
    
    # PARAMETRIZED NN: We can now control robustness!
    # Let's make it crappy (10% error rate) to see what happens
    nn_system = PerceptionSystem(false_negative_rate=0.10) 
    
    # CONTROLLER with Beliefs
    controller = AEBSController(dt=dt)
    
    # 2. Initial State [Gap=40m, Ego_Vel=20m/s, Lead_Vel=15m/s]
    # IMPORTANT: Sync controller belief with reality at T=0
    x = np.array([40.0, 20.0, 15.0]) 
    controller.est_velocity = 20.0 
    
    history = {'time': [], 'dist': [], 'true_v': [], 'est_v': []}
    
    print(f"{Colors.HEADER}--- STARTING SIMULATION (Robustness Test) ---{Colors.RESET}")
    print(f"{'Time':<6} | {'Dist':<8} | {'True V':<8} | {'Est V':<8} | {'NN':<3} | {'Action'}")
    print("-" * 65)
    
    # 3. Loop
    for step in range(60): 
        t = step * dt
        
        # A. PERCEPTION
        true_dist = x[0]
        obs_class = nn_system.observe(true_dist)
        
        # B. CONTROL (Using Beliefs, NOT Truth)
        # Note: We pass 'true_dist' as 'observed_distance' for now (perfect lidar, imperfect camera)
        # If you want noisy distance too, we can add noise here.
        u_acc = controller.get_action(
            perception_output=obs_class, 
            observed_distance=true_dist 
        )
        
        # C. PHYSICS
        x = plant.get_next_state(x, u_acc)
        
        # D. LOGGING
        c = get_state_color(controller.state)
        nn_str = "OBJ" if obs_class == 1 else "---"
        print(f"{t:5.1f}s | {x[0]:7.2f}m | {x[1]:7.2f}  | {controller.est_velocity:7.2f}  | {nn_str} | {c}{controller.state.upper():<10}{Colors.RESET}")
        
        history['time'].append(t)
        history['dist'].append(x[0])
        history['true_v'].append(x[1])
        history['est_v'].append(controller.est_velocity)

        if x[0] <= 0:
            print(f"{Colors.RED}{Colors.BOLD}!!! CRASH !!!{Colors.RESET}")
            break
            
    # 4. Plotting
    plt.figure(figsize=(10, 8))
    
    plt.subplot(2, 1, 1)
    plt.plot(history['time'], history['dist'], 'b-o', label='Distance')
    plt.axhline(0, color='r', linestyle='--', label='Crash')
    plt.axhline(3, color='orange', linestyle='--', label='Emergency')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.grid(True)
    plt.title('AEBS Simulation: Truth vs Belief')
    
    plt.subplot(2, 1, 2)
    plt.plot(history['time'], history['true_v'], 'g-', label='True Velocity')
    plt.plot(history['time'], history['est_v'], 'k--', label='Controller Belief')
    plt.ylabel('Velocity (m/s)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_simulation()
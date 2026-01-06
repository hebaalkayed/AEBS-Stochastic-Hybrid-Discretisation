import sys
import os
import matplotlib.pyplot as plt

# Path Setup
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from system.vehicle_plant import VehiclePlant
from system.controller import AEBSController
from system.perception import PerceptionSystem
from system.environment import TrafficEnvironment
from visualiser.plotter import SimulationPlotter

def get_user_config():
    print("\n" + "="*40)
    print("      AEBS SCENARIO CONFIGURATION")
    print("="*40)
    
    # 1. Behavior
    print("Select Lead Vehicle Behavior:")
    print("  1. Static (Wall/Stopped Car)")
    print("  2. Steady (Constant Velocity)")
    print("  3. Unpredictable (Stochastic)")
    choice = input("Choice [1]: ") or "1"
    
    behavior_map = {'1': 'static', '2': 'steady', '3': 'unpredictable'}
    behavior = behavior_map.get(choice, 'static')
    
    # 2. Initial Physics
    print(f"\n--- Scenario: {behavior.upper()} ---")
    init_gap = float(input(f"Initial Gap (m) [40.0]: ") or 40.0)
    ego_v = float(input(f"Ego Velocity (m/s) [20.0]: ") or 20.0)
    
    lead_v = 0.0
    if behavior != 'static':
        lead_v = float(input(f"Lead Velocity (m/s) [10.0]: ") or 10.0)
        
    # 3. Controller Mode
    print("\nSelect Controller Benchmark:")
    print("  1. Industry (Late Braking)")
    print("  2. Safe (Early Braking)")
    c_choice = input("Choice [2]: ") or "2"
    c_mode = 'industry' if c_choice == '1' else 'safe'
    
    return behavior, init_gap, ego_v, lead_v, c_mode

def run_simulation_loop():
    
    while True:
        # A. Get Config from User
        behavior, init_gap, init_ego_v, init_lead_v, c_mode = get_user_config()
        
        # B. Initialize Components
        dt = 0.1
        env = TrafficEnvironment(dt=dt)
        perception = PerceptionSystem(false_negative_rate=0.0) 
        controller = AEBSController(mode=c_mode, lead_behavior=behavior)
        
        # USE WORLD FRAME FOR PLOTTING
        plant = VehiclePlant(dt=dt, coordinate_system='world_frame')
        
        # C. INJECT CONFIGURATION INTO ENVIRONMENT
        env.configure(
            scenario_type=behavior,
            initial_gap=init_gap,
            initial_ego_v=init_ego_v,
            initial_lead_v=init_lead_v
        )
        
        # D. Run Loop
        history = {'time': [], 'dist': [], 'ego_v': [], 'lead_v': []}
        crashed = False
        
        # Sync Plant with Env Initial State
        plant.actual_velocity = init_ego_v
        
        print(f"\n--- STARTING SIMULATION ({c_mode.upper()} MODE) ---")
        
        for step in range(300): # 30 seconds
            t = step * dt
            
            # 1. ENVIRONMENT UPDATE
            ego_disp = plant.actual_velocity * dt
            ground_truth = env.update_physics(ego_disp, plant.actual_velocity)
            
            # 2. PERCEPTION UPDATE
            is_seen, obs_gap, obs_v_rel = perception.read_sensors(ground_truth)
            
            # 3. CONTROLLER UPDATE
            acc_cmd, action_name = controller.get_action(is_seen, obs_gap, obs_v_rel)
            
            # 4. PLANT UPDATE
            plant.step(acc_cmd)
            
            # 5. Logging
            history['time'].append(t)
            history['dist'].append(ground_truth['gap'])
            history['ego_v'].append(ground_truth['v_ego'])
            history['lead_v'].append(ground_truth['v_lead'])
            
            if ground_truth['gap'] <= 0:
                print(f"!!! CRASH DETECTED at T={t:.1f}s !!!")
                crashed = True
                break
                
            if plant.actual_velocity < 0.1 and ground_truth['v_lead'] < 0.1:
                print(f"Scenario Ended: Both Vehicles Stopped at T={t:.1f}s")
                break

        # E. Plot
        print("\nPlot generated. Close the window to continue...")
        SimulationPlotter.plot_scenario(history, f"Scenario: {behavior.upper()} | Controller: {c_mode.upper()}")
        plt.show()
        
        # F. Ask to Repeat
        retry = input("\nWould you like to simulate another scenario? (y/n) [y]: ")
        if retry.lower() == 'n':
            print("Exiting Simulation. Drive Safe!")
            break

if __name__ == "__main__":
    run_simulation_loop()
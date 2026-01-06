import sys
import os
import numpy as np

# Add src to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.grid import Grid
from src.abstraction.pipeline import execute_abstraction

def test_relative_physics_logic():
    print("\n[Test] Checking Relative Physics (Gap Dynamics)...")
    
    # 1. Setup Plant in RELATIVE FRAME (Gap)
    plant = VehiclePlant(coordinate_system='relative_frame', dt=0.5) 
    
    # State: Gap=50m, Closing Speed=20m/s
    x_0 = np.array([50.0, 20.0, 0.0])
    u_coast = 0.0
    
    # 2. Step
    x_next = plant.get_deterministic_next_state(x_0, u_coast)
    
    print(f"  Start: Gap={x_0[0]}m, Vel={x_0[1]}m/s")
    print(f"  End:   Gap={x_next[0]}m, Vel={x_next[1]}m/s")
    
    # 3. Verify
    delta_gap = x_next[0] - x_0[0]
    
    if delta_gap < 0:
        print("   SUCCESS: Gap decreased (System is strictly relative).")
    else:
        print("   FAILED: Gap increased or stayed same (System is absolute).")
        raise AssertionError("Plant logic is inverted!")

def verify_universal_model():
    print("=======================================================")
    print("       TEST: UNIVERSAL RELATIVE MODEL (Gap Dynamics)")
    print("=======================================================")

    # 1. Validate Physics First
    test_relative_physics_logic()

    # 2. Setup Abstraction
    # Grid covers the Relative State Space
    # Gap: 0-100m, Closing Speed: 0-40m/s
    grid = Grid(
        x_bounds=(0, 100), 
        v_bounds=(0, 40), 
        custom_resolution=(1.0, 1.0, 0.5), # <--- FIXED NAME HERE
        preset=None 
    )
    grid.description = "Universal Relative Grid"
    
    # Plant in Relative Frame
    plant = VehiclePlant(coordinate_system='relative_frame', dt=0.1)
    
    # Controller (Safe Mode)
    controller = AEBSController(mode='safe')

    # 3. Run Pipeline
    print(f"\n[Exec] Running Abstraction...")
    finite_mdp = execute_abstraction(plant, grid, controller)

    # 4. Export
    output_path = "artifacts/relative_model.prism"
    if not os.path.exists("artifacts"): os.makedirs("artifacts")
    
    finite_mdp.to_prism(output_path)
    
    print(f"\n[Artifact] Generated: {output_path}")
    print("This model is valid for ANY scenario (Static, Lead, Cut-in) where:")
    print("  x = Distance to Collision")
    print("  v = Closing Speed (V_ego - V_lead)")

    # 5. Spot Check a Transition
    # Pick a state near the middle: Gap 50, Vel 20
    test_state = np.array([50.0, 20.0, 0.0])
    src_idx = grid.state_to_index(test_state)
    src_flat = grid.get_flat_index(src_idx)
    
    print(f"\n[Verification] Checking Relative Dynamics...")
    if (src_flat, 'coast') in finite_mdp.transitions:
        trans_list = finite_mdp.transitions[(src_flat, 'coast')]
        
        # Weighted average of target IDs
        avg_target = sum(t.target_id * t.p_mean for t in trans_list)
        
        print(f"    - Source State ID: {src_flat} (Gap ~ 50m)")
        print(f"    - Target State ID: {avg_target:.1f} (Weighted Avg)")
        
        if avg_target < src_flat:
            print("     SUCCESS: Target ID is lower than Source ID.")
            print("       (This implies the state moved 'Left' in the grid -> Gap Decreased)")
        else:
            print("     FAIL: Target ID is higher. The car is driving backwards away from the obstacle.")

if __name__ == "__main__":
    verify_universal_model()
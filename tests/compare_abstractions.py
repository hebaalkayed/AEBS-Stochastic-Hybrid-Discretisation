import sys
import os
import time

# ==============================================================================
#  PATH SETUP
#  Add the project root to sys.path so we can import from 'src'
#  Assumes folder structure:
#    project_root/
#      src/
#      tests/
#        compare_abstractions.py
# ==============================================================================
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.abstraction.grid import Grid, GRID_PRESETS
from src.system.vehicle_plant import VehiclePlant
from src.abstraction.engine import AbstractionEngine

def compare_grids():
    """
    Runs a physics validity check across all defined grid presets.
    Goal: Find the coarsest grid that still captures the braking physics accurately.
    """
    print(f"\n{'='*100}")
    print(f"ABSTRACTION GRID COMPARISON TOOL")
    print(f"{'='*100}")
    print(f"{'PRESET':<10} | {'RESOLUTION (X, V, A)':<25} | {'STATES':<10} | {'PHYSICS DELTA V (Expected < 0)'}")
    print("-" * 100)

    plant = VehiclePlant()
    
    # Test Case: Brake from 20m/s
    # We want to see if the grid is fine enough to capture the deceleration
    # Physics: Brake -4.0 m/s^2 for 0.1s should result in -0.4 m/s change (roughly)
    test_state_cont = [50.0, 20.0, 0.0]
    action_brake = -4.0

    for name in GRID_PRESETS.keys():
        # 1. Initialize Grid with Preset
        try:
            grid = Grid(preset=name)
        except Exception as e:
            print(f"{name:<10} | {'ERROR: ' + str(e):<50}")
            continue
        
        # 2. Run a Quick Physics Check
        engine = AbstractionEngine(plant, grid)
        idx = grid.state_to_index(test_state_cont)
        
        result = ""
        
        if idx is None:
            result = " Out of Bounds"
        else:
            # Compute transitions for one cell
            transitions = engine.compute_transitions(idx, action_brake)
            
            if not transitions:
                result = "  No Transitions (Stuck?)"
            else:
                # Calculate expected velocity change
                # Get weighted average of next states to see the "Average Physics"
                avg_next_v = 0.0
                total_prob = 0.0
                
                for flat, (p_min, p_max) in transitions.items():
                    center = grid.index_to_cell_center(grid.get_tuple_index(flat))
                    # Use mean probability for the estimate
                    prob = (p_min + p_max) / 2.0
                    
                    avg_next_v += center[1] * prob
                    total_prob += prob
                
                if total_prob > 0:
                    avg_next_v /= total_prob
                    delta = avg_next_v - 20.0
                    
                    # Formatting logic for clarity
                    if delta < -0.01:
                         result = f" {delta:.4f} m/s (Captured)"
                    elif delta > 0.01:
                         result = f" {delta:.4f} m/s (Accelerated? Bug?)"
                    else:
                         result = f"  {delta:.4f} m/s (Too Coarse)"
                else:
                    result = " Probability Error"

        # 3. Print Row
        res_str = str(grid.resolution)
        print(f"{name:<10} | {res_str:<25} | {grid.total_states:<10} | {result}")

    print("-" * 100)
    print("RECOMMENDATION: Pick the 'coarsest' preset that still gets a Green Check (✅).")
    print("This balances Verification Speed vs. Physical Accuracy.")

if __name__ == "__main__":
    compare_grids()
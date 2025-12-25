import numpy as np
import sys
import os

# Add src to path so we can import modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.abstraction.grid import Grid
from src.abstraction.engine import AbstractionEngine

def test_single_transition():
    # 1. Setup the System
    # Grid: 0-100m, 0-30m/s, -10 to 5 m/s^2
    grid = Grid(x_bounds=(0, 100), v_bounds=(0, 30), a_bounds=(-10, 5), resolution=(1.0, 1.0, 0.5))
    plant = VehiclePlant()
    
    engine = AbstractionEngine(plant, grid)

    # 2. Pick a Test State
    # Let's say we are at x=50m, v=20m/s, a=0
    test_state_continuous = np.array([50.0, 20.0, 0.0])
    test_idx = grid.state_to_index(test_state_continuous)
    
    print(f"--- TEST SETUP ---")
    print(f"Continuous State: {test_state_continuous}")
    print(f"Grid Index: {test_idx}")
    print(f"Cell Center: {grid.index_to_cell_center(test_idx)}")
    
    # 3. Run Abstraction for 'BRAKE' (-4.0 m/s^2)
    print(f"\n--- COMPUTING TRANSITIONS (Action: Brake -4.0) ---")
    brake_action = -4.0
    transitions = engine.compute_transitions(test_idx, brake_action)

    # 4. Analyze Results
    print(f"Found {len(transitions)} reachable target cells.")
    
    total_prob_min = 0.0
    total_prob_max = 0.0
    
    print("\nTop 5 Most Likely Transitions:")
    sorted_trans = sorted(transitions.items(), key=lambda item: item[1][0], reverse=True)
    
    for flat_idx, (p_min, p_max) in sorted_trans[:5]:
        target_idx_tuple = grid.get_tuple_index(flat_idx)
        target_center = grid.index_to_cell_center(target_idx_tuple)
        
        print(f"  -> Cell {target_idx_tuple} (Center: {target_center})")
        print(f"     Interval: [{p_min:.4f}, {p_max:.4f}]")
        print(f"     Physics Check: Delta V = {target_center[1] - 20.0:.2f} m/s")
        
        total_prob_min += p_min
        total_prob_max += p_max

    print(f"\nTotal Probability Mass (Sum of lower bounds): {total_prob_min:.4f}")
    
    # 5. Sanity Checks
    if len(transitions) == 0:
        print("❌ FAILED: No transitions found. Noise sigma might be too small relative to grid size.")
    elif total_prob_min > 1.0:
        print("❌ FAILED: Probability > 1.0. Check Lipschitz error addition.")
    else:
        print("✅ SUCCESS: Abstraction generated valid probability intervals.")
        
    # Check physics (Velocity should decrease)
    # Get the weighted average velocity of targets
    avg_next_v = 0.0
    weight_sum = 0.0
    for flat_idx, (p_min, _) in transitions.items():
        idx_t = grid.get_tuple_index(flat_idx)
        center = grid.index_to_cell_center(idx_t)
        avg_next_v += center[1] * p_min
        weight_sum += p_min
        
    if weight_sum > 0:
        avg_next_v /= weight_sum
        print(f"Average Next Velocity: {avg_next_v:.2f} m/s (Should be < 20.0)")

if __name__ == "__main__":
    test_single_transition()
import time
import itertools
import sys
import os

# Ensure we can find 'src' if running this file directly
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from src.system.vehicle_plant import VehiclePlant
from src.abstraction.grid import Grid
from src.abstraction.engine import AbstractionEngine
from src.types.mdp import MDP

def execute_abstraction(plant, grid, controller=None):
    """
    The Main Abstraction Pipeline.
    
    Orchestrates the conversion of the Continuous System (dt-SCS) 
    into a Finite Interval MDP, and optionally appends Controller Logic.
    
    Args:
        plant: The physics and noise model.
        grid: The discretization of the state space.
        controller: (Optional) Instance of AEBSController to generate logic formulas.
        
    Returns:
        mdp: The populated Finite MDP structure.
    """
    print(f"\n[Pipeline] Starting Abstraction Process...")
    print(f"    - System: Alpha={plant.alpha}, Noise Std={plant.noise_std}")
    print(f"    - Grid:   {grid.description}")
    print(f"    - States: {grid.total_states} (Resolution: {grid.resolution})")
    
    # 1. Initialize Components
    engine = AbstractionEngine(plant, grid)
    finite_mdp = MDP(num_states=grid.total_states)
    
    # 2. Define Controller Inputs (The 'U' set)
    # These must match the actions your Controller returns in 'get_action_name_for_state'
    actions = {
        'coast': 0.0,
        'brake_warn': -4.0,
        'brake_full': -9.8
    }
    
    # 3. Execution Loop
    start_time = time.time()
    processed_count = 0
    total_cells = grid.total_states
    
    # Calculate a smart logging interval (Log every 5% or at least every 1000 states)
    log_interval = max(1000, total_cells // 20)

    # Create iterator for all grid cells (x, v, a)
    iterator = itertools.product(*[range(d) for d in grid.shape])
    
    for idx_tuple in iterator:
        src_flat_id = grid.get_flat_index(idx_tuple)
        
        # Iterate over all possible actions
        # We calculate transitions for ALL actions, creating an "Open Loop" MDP.
        # The Controller module (added later) will restrict which actions are actually taken.
        for action_name, accel_val in actions.items():
            
            # A. Compute Transitions (Physics + Integral + Lipschitz)
            transitions = engine.compute_transitions(idx_tuple, accel_val)
            
            # B. Populate the Finite MDP
            for target_flat_id, (p_min, p_max) in transitions.items():
                finite_mdp.add_transition(
                    src=src_flat_id,
                    action=action_name,
                    target=target_flat_id,
                    p_min=p_min,
                    p_max=p_max
                )
        
        # Logging Progress
        processed_count += 1
        if processed_count % log_interval == 0:
            percent = (processed_count / total_cells) * 100
            print(f"    - Processed {processed_count}/{total_cells} states ({percent:.1f}%)")

    duration = time.time() - start_time
    print(f"[Pipeline] Abstraction Complete in {duration:.2f} seconds.")
    print(f"    - {finite_mdp.get_stats()}")
    
    # 4. Generate Controller Logic (if provided)
    # We attach the generated PRISM text to the MDP object metadata.
    # The MDP.to_prism method will handle writing this to the file.
    if controller:
        print(f"[Pipeline] Generating Controller Logic for mode '{controller.mode}'...")
        controller_logic_str = controller.generate_prism_logic(grid)
        finite_mdp.controller_logic = controller_logic_str
    
    return finite_mdp
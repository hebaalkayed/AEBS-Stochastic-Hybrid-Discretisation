import time
import itertools
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from src.system.vehicle_plant import VehiclePlant
from src.abstraction.grid import Grid
from src.abstraction.engine import AbstractionEngine
from src.types.mdp import MDP

def execute_abstraction(plant, grid, controller=None):
    """
    The Main Abstraction Pipeline.
    """
    print(f"\n[Pipeline] Starting Abstraction Process...")
    print(f"    - Coord System: {plant.coordinate_system.upper()}") # <-- Updated Name
    print(f"    - Grid:   {grid.description}")
    
    # 1. Initialize Components
    engine = AbstractionEngine(plant, grid)
    finite_mdp = MDP(num_states=grid.total_states)
    
    # 2. Define Controller Inputs
    actions = {
        'coast': 0.0,
        'brake_warn': -4.0,
        'brake_full': -9.8
    }
    
    # 3. Execution Loop
    start_time = time.time()
    processed_count = 0
    total_cells = grid.total_states
    
    log_interval = max(1000, total_cells // 20)
    iterator = itertools.product(*[range(d) for d in grid.shape])
    
    for idx_tuple in iterator:
        src_flat_id = grid.get_flat_index(idx_tuple)
        
        for action_name, accel_val in actions.items():
            
            # A. Compute Transitions
            transitions = engine.compute_transitions(idx_tuple, accel_val)
            
            # B. Populate MDP
            for target_flat_id, (p_min, p_max) in transitions.items():
                finite_mdp.add_transition(
                    src=src_flat_id,
                    action=action_name,
                    target=target_flat_id,
                    p_min=p_min,
                    p_max=p_max
                )
        
        processed_count += 1
        if processed_count % log_interval == 0:
            percent = (processed_count / total_cells) * 100
            print(f"    - Processed {processed_count}/{total_cells} states ({percent:.1f}%)")

    duration = time.time() - start_time
    print(f"[Pipeline] Abstraction Complete in {duration:.2f} seconds.")
    print(f"    - {finite_mdp.get_stats()}")
    
    # 4. Generate Controller Logic
    if controller:
        print(f"[Pipeline] Generating Controller Logic for mode '{controller.mode}'...")
        # UPDATED: Pass the coordinate system name
        controller_logic_str = controller.generate_prism_logic(
            grid, 
            plant_coords=plant.coordinate_system
        )
        finite_mdp.controller_logic = controller_logic_str
    
    return finite_mdp
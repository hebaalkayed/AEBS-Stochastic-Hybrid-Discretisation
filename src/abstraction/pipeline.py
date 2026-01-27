import time
import itertools
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from src.system.vehicle_plant import VehiclePlant
from src.abstraction.grid import Grid
from src.abstraction.engine import AbstractionEngine
from src.abstraction.types.mdp import MDP
from src.abstraction.semantics.labeling import LabelingGrammar

def execute_abstraction(plant, grid, controller=None):
    print(f"\n[Pipeline] Starting Abstraction Process...")
    print(f"    - Coord System: {plant.coordinate_system.upper()}")
    
    # 1. Initialize Components
    engine = AbstractionEngine(plant, grid)
    finite_mdp = MDP(num_states=grid.total_states)
    
    # 2. Initialize the Labeling Grammar (The Semantics)
    # This acts as the "Judge" that tags states based on the Controller's rules.
    grammar = LabelingGrammar(controller)
    print(f"    - Semantics: {grammar}")
    
    # 3. Dynamic Actions (System z inputs)
    # We pull the specific braking forces from the controller to match the simulation
    action_brake_val = getattr(controller, 'acc_brake', -4.0)
    action_emergency_val = getattr(controller, 'acc_emergency', -9.8)
    
    actions = {
        'coast': 0.0,
        'brake_warn': action_brake_val,     # e.g., -4.0 (Standard)
        'brake_full': action_emergency_val  # e.g., -9.8 (Emergency)
    }
    
    print(f"    - Actions Mapped: {actions}")
    
    # 4. Main Loop
    start_time = time.time()
    processed_count = 0
    total_cells = grid.total_states
    log_interval = max(1000, total_cells // 20)

    iterator = itertools.product(*[range(d) for d in grid.shape])
    
    for idx_tuple in iterator:
        src_flat_id = grid.get_flat_index(idx_tuple)
        
        # --- A. SEMANTIC LABELING ---
        # 1. Get Physical Reality
        center_state = grid.index_to_cell_center(idx_tuple)
        
        # 2. Apply Grammar (L(s))
        # e.g., returns ['safe', 'braking', 'emergency']
        labels = grammar.get_labels(center_state)
        
        # 3. Store in Model (Both Struct and Label Groups)
        finite_mdp.add_state_info(src_flat_id, tuple(center_state), labels)

        # --- B. TRANSITION CALCULATION ---
        for action_name, accel_val in actions.items():
            transitions = engine.compute_transitions(idx_tuple, accel_val)
            for target_id, (p_min, p_max) in transitions.items():
                finite_mdp.add_transition(src_flat_id, action_name, target_id, p_min, p_max)
        
        processed_count += 1
        
        if processed_count % log_interval == 0:
            percent = (processed_count / total_cells) * 100
            print(f"    - Processed {processed_count}/{total_cells} states ({percent:.1f}%)")

    # 5. Finalize
    duration = time.time() - start_time
    print(f"[Pipeline] Abstraction Complete in {duration:.2f} seconds.")
    print(f"    - {finite_mdp.get_stats()}")
    
    # 6. Generate Logic Block
    if controller:
        finite_mdp.controller_logic = controller.generate_prism_logic(
            grid, plant_coords=plant.coordinate_system
        )
        
    return finite_mdp
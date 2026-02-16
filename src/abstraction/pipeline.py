import os
import itertools
from src.abstraction.types.grid import Grid
from src.abstraction.algorithms.discretizer import DiscretizationAlgorithm
from src.abstraction.modules.wrappers import PlantWrapper
from src.abstraction.semantics.labeling import LabelingGrammar
from src.abstraction.types.abstracted_controller import ControllerModel
from src.abstraction.types.perception_model import PerceptionModel
from src.abstraction.exporters.prism_generator import PrismModelGenerator

def run_modular_abstraction(plant, controller, grid_preset='medium', enable_perception_noise=False):
    """
    High-Level Orchestrator with Dynamic Initialization Support.
    """
    print(f"\n[Pipeline] Starting Modular Abstraction (Preset: {grid_preset})...")
    print(f"[Pipeline] Perception Mode: {'NOISY (Explicit)' if enable_perception_noise else 'PERFECT (Symbolic)'}")
    
    # 1. Setup
    grid = Grid(preset=grid_preset)
    algo = DiscretizationAlgorithm(grid)
    
    # --- MODULE 1: PLANT ---
    print("[Pipeline] Module 1/3: Vehicle Plant")
    plant_imdp = algo.run(PlantWrapper(plant, controller), name="Plant")
    sink_id = plant_imdp.finalize_sink_state()
    
    # Labeling
    plant_imdp.add_label("crash", 0)
    safe_states = [i for i in range(1, sink_id)] 
    plant_imdp.add_bulk_label("safe_physics", safe_states)
    
    # --- MODULE 2: CONTROLLER ---
    print("[Pipeline] Module 2/3: AEBS Controller (Synthesis)")
    
    ctrl_model = ControllerModel(
        name="Controller", 
        input_var="y", 
        output_var="u",
        variable_def="u : [0..2] init 0;" 
    )
    
    action_map = {'coast': 0, 'brake_warn': 1, 'brake_full': 2}
    grammar = LabelingGrammar(controller)
    
    for idx in itertools.product(*[range(d) for d in grid.shape]):
        flat_id = grid.get_flat_index(idx)
        center = grid.index_to_cell_center(idx)
        
        # A. Logic
        gap, v = center[0], center[1]
        act_name = controller.get_action_name_for_state(gap, v, 0, plant_coords='relative_frame')
        act_id = action_map.get(act_name, 0)
        ctrl_model.add_rule(flat_id, act_id)
        
        # B. Labels
        tags = grammar.get_labels(center)
        for t in tags:
            plant_imdp.add_label(t, flat_id)

    # Controller Robustness (Sink)
    ctrl_model.add_rule(sink_id, 0)

    # --- DYNAMIC SCENARIO SETUP ---
    scenarios = {
        "Nominal_Cruising":  (40.0, 15.0, 0.0), 
        "Warning_Threshold": (25.0, 15.0, 0.0), 
        "Critical_Braking":  (15.0, 15.0, 0.0), 
        "Imminent_Collision": (10.0, 20.0, 0.0), 
        "Post_Collision":    (0.0, 0.0, 0.0)    
    }

    print("\n" + "="*60)
    print("   SCENARIO ID CHEAT SHEET (Use these in PRISM -const)")
    print("="*60)
    print(f" {'SCENARIO':<20} | {'PHYSICS (Gap, V, A)':<25} | {'ID'}")
    print("-" * 60)
    
    for name, values in scenarios.items():
        idx = grid.state_to_index(values)
        if idx:
            flat_id = grid.get_flat_index(idx)
            print(f" {name:<20} | {str(values):<25} | {flat_id}")
        else:
            print(f" {name:<20} | {str(values):<25} | OUT OF BOUNDS")
    print("="*60 + "\n")

    # KEY CHANGE: Set initial state to a variable, not a number
    plant_imdp.initial_state = "start_s"

    # --- MODULE 3: PERCEPTION ---
    print("[Pipeline] Module 3/3: Perception Layer")
    perception = PerceptionModel(
        name="Perception", 
        input_var="s", 
        output_var="y", 
        max_state=sink_id
    )
    perception.enable_noise = enable_perception_noise

    # --- EXPORT ---
    output_path = "artifacts/modular_system.prism"
    
    # Define 'start_s' as a global constant for PRISM
    constants = {'start_s': 'int'}
    globals_dict = {} 
    
    generator = PrismModelGenerator(output_path)
    
    generator.generate(
        modules=[plant_imdp, perception, ctrl_model], 
        globals_dict=globals_dict,
        constants=constants 
    )
    
    return output_path
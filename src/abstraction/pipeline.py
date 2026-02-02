import os
import itertools
from src.abstraction.types.grid import Grid
from src.abstraction.algorithms.discretizer import DiscretizationAlgorithm
from src.abstraction.modules.wrappers import PlantWrapper
from src.abstraction.semantics.labeling import LabelingGrammar
from src.abstraction.types.abstracted_controller import ControllerModel
from src.abstraction.types.perception_model import PerceptionModel # <--- NEW
from src.abstraction.exporters.prism_generator import PrismModelGenerator

def run_modular_abstraction(plant, controller, grid_preset='medium', enable_perception_noise=False):
    """
    High-Level Orchestrator.
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
    
    plant_imdp.add_label("crash", 0)
    plant_imdp.add_label("initial", 0)
    safe_states = [i for i in range(1, sink_id)] 
    plant_imdp.add_bulk_label("safe_physics", safe_states)
    
    # --- MODULE 2: CONTROLLER ---
    print("[Pipeline] Module 2/3: AEBS Controller (Synthesis)")
    ctrl_model = ControllerModel(name="Controller", input_var="y", output_var="u")
    
    action_map = {'coast': 0, 'brake_warn': 1, 'brake_full': 2}
    grammar = LabelingGrammar(controller)
    label_map = {"braking": [], "warning": [], "emergency": [], "safe": []}

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
    
    globals_dict = {
        "t": "[1..3] init 1",
        "u": "[0..2] init 0",
        "y": f"[0..{sink_id}] init 0"
    }
    
    generator = PrismModelGenerator(output_path)
    
    # This now uses the Streaming methods in the generator
    generator.generate(
        modules=[plant_imdp, perception, ctrl_model], 
        globals_dict=globals_dict
    )
    
    return output_path
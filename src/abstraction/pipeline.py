import os
import itertools
from src.abstraction.types.grid import Grid
from src.abstraction.algorithms.discretizer import DiscretizationAlgorithm
from src.abstraction.modules.wrappers import PlantWrapper
from src.abstraction.semantics.labeling import LabelingGrammar
from src.abstraction.types.abstracted_controller import ControllerModel
from src.abstraction.types.perception_model import PerceptionModel
from src.abstraction.exporters.prism_generator import PrismModelGenerator

def run_modular_abstraction(plant, controller, grid_preset='medium', enable_perception_noise=False,
                            grid_bounds=None, output_path=None, lead_noise_std=2.0):
    """
    High-Level Orchestrator with Dynamic Initialization Support.
    
    Args:
        plant:          VehiclePlant or POCVehiclePlant instance.
        controller:     AEBSController or POCController instance.
        grid_preset:    Name of grid resolution preset (e.g. 'medium', 'micro').
        enable_perception_noise: If True, use explicit perception enumeration.
        grid_bounds:    Optional dict with keys 'x_bounds', 'v_bounds', 'vl_bounds'
                        to override default Grid bounds.
        output_path:    Optional output file path (default: 'artifacts/modular_system.prism').
        lead_noise_std: Lead vehicle acceleration noise std (m/s²). Default 2.0 for
                        production system. Use ~0.5 for the POC micro-world.
    """
    print(f"\n[Pipeline] Starting Modular Abstraction (Preset: {grid_preset})...")
    print(f"[Pipeline] Perception Mode: {'NOISY (Explicit)' if enable_perception_noise else 'PERFECT (Symbolic)'}")
    
    # 1. Setup
    if grid_bounds:
        grid = Grid(preset=grid_preset, **grid_bounds)
    else:
        grid = Grid(preset=grid_preset)
    algo = DiscretizationAlgorithm(grid)
    
    # --- MODULE 1: PLANT ---
    print("[Pipeline] Module 1/3: Vehicle Plant")
    plant_imdp = algo.run(
        PlantWrapper(plant, controller, lead_noise_std=lead_noise_std),
        name="Plant"
    )
    sink_id = plant_imdp.finalize_sink_state()
    
    # Labeling (Phase 1: crash state s=0 is always crash)
    plant_imdp.add_label("crash", 0)
    
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
        # BUG FIX: Use closing speed v_rel = v_ego - v_lead, NOT raw v_ego.
        # With v_lead=0 (static obstacle) they're identical, so the bug was hidden.
        # With v_lead>0, using v_ego overestimates danger (shorter TTC than reality).
        gap, v_ego, v_lead = center[0], center[1], center[2]
        v_rel = v_ego - v_lead
        
        act_name = controller.get_action_name_for_state(gap, v_rel, 0, plant_coords='relative_frame')
        act_id = action_map.get(act_name, 0)
        ctrl_model.add_rule(flat_id, act_id)
        
        # B. Labels
        tags = grammar.get_labels(center)
        for t in tags:
            plant_imdp.add_label(t, flat_id)

    # Labeling (Phase 2: safe_physics = all non-crash, non-sink states)
    crash_states = plant_imdp.labels.get("crash", set())
    safe_states = [i for i in range(1, sink_id) if i not in crash_states]
    plant_imdp.add_bulk_label("safe_physics", safe_states)

    # Controller Robustness (Sink)
    ctrl_model.add_rule(sink_id, 0)

    # --- DYNAMIC SCENARIO TABLE ---
    # Build scenarios from the actual grid — works for both production and POC
    scenarios = {}
    
    # Gather controller decisions and labels per state
    state_info = {}
    for idx in itertools.product(*[range(d) for d in grid.shape]):
        flat_id = grid.get_flat_index(idx)
        center = grid.index_to_cell_center(idx)
        gap, v_ego, v_lead = center[0], center[1], center[2]
        v_rel = v_ego - v_lead
        act_name = controller.get_action_name_for_state(gap, v_rel, 0, plant_coords='relative_frame')
        act_id = action_map.get(act_name, 0)
        labels = grammar.get_labels(center)
        state_info[flat_id] = {
            'center': center, 'action': act_name, 'act_id': act_id,
            'labels': labels, 'gap': gap, 'v_ego': v_ego, 'v_lead': v_lead
        }
    
    # Pick representative states for each controller decision zone
    # Prefer v_lead=0 (static obstacle) for clarity, highest v_ego for drama
    for zone_name, target_act, sort_key in [
        ("Safe_Cruising",      'coast',      lambda s: (s['gap'], s['v_ego'])),   # far + fast
        ("Warning_Brake",      'brake_warn', lambda s: (s['gap'], s['v_ego'])),   # far + fast braking
        ("Emergency_Brake",    'brake_full', lambda s: (s['gap'], s['v_ego'])),   # far + fast emergency
        ("Imminent_Collision", 'brake_full', lambda s: (-s['gap'], s['v_ego'])),  # close + fast
        ("Post_Collision",     None,         lambda s: (s['gap'],)),              # crash state
    ]:
        if target_act is None:
            # Find a crash state
            candidates = [sid for sid, info in state_info.items() 
                          if info['gap'] <= 0 and info['v_ego'] == 0]
            if candidates:
                scenarios[zone_name] = candidates[0]
        else:
            candidates = [(sid, info) for sid, info in state_info.items()
                          if info['action'] == target_act and info['v_lead'] == 0
                          and info['v_ego'] > 0 and info['gap'] > 0]
            if not candidates:
                # Fallback: allow any v_lead
                candidates = [(sid, info) for sid, info in state_info.items()
                              if info['action'] == target_act and info['v_ego'] > 0]
            if candidates:
                candidates.sort(key=lambda x: sort_key(x[1]), reverse=True)
                scenarios[zone_name] = candidates[0][0]

    print("\n" + "="*60)
    print("   SCENARIO ID CHEAT SHEET (Use these in PRISM -const)")
    print("="*60)
    print(f" {'SCENARIO':<22} | {'STATE (Gap, V, Vl)':<22} | {'Action':<12} | {'ID':>3}")
    print("-" * 66)
    
    for name, flat_id in scenarios.items():
        info = state_info.get(flat_id, {})
        c = info.get('center', (0,0,0))
        act = info.get('action', '?')
        print(f" {name:<22} | ({c[0]:>4.0f}, {c[1]:>3.0f}, {c[2]:>3.0f}){'':<9} | {act:<12} | {flat_id:>3}")
    print("="*66 + "\n")

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
    if output_path is None:
        output_path = "artifacts/modular_system.prism"
    
    constants = {'start_s': 'int'}
    globals_dict = {} 
    
    generator = PrismModelGenerator(output_path)
    
    generator.generate(
        modules=[plant_imdp, perception, ctrl_model], 
        globals_dict=globals_dict,
        constants=constants 
    )
    
    return output_path
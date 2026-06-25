import os
import itertools

from src.abstraction.types.grid import Grid
from src.abstraction.algorithms.discretizer import DiscretizationAlgorithm
from src.abstraction.modules.wrappers import PlantWrapper
from src.abstraction.semantics.labeling import LabelingGrammar
from src.abstraction.types.abstracted_controller import ControllerModel
from src.abstraction.types.perception_model import PerceptionModel
from src.abstraction.exporters.prism_generator import PrismModelGenerator
from src.abstraction.algorithms.discretizer import compute_global_error_bound
from src.abstraction.profiles.lead_model import get_lead_model        # new injected path


def run_modular_abstraction(plant, controller, grid_preset='medium', enable_perception_noise=False,
                            grid_bounds=None, output_path=None, lead_model='static'):
    """
    High-Level Orchestrator. A LeadModel owns the lead's v_lead rule,
    grid band, noise, and scenarios. 'static'/'steady' are the deterministic
    leads; nondeterministic leads are rejected by the wrapper until that path
    is built.
    """
    lead = get_lead_model(lead_model)
    wrapper = PlantWrapper(plant, controller, lead_model=lead)
    scenarios_list = lead.scenarios()
    prefix = lead.prefix
    lead_desc = lead.name
    gb = dict(grid_bounds) if grid_bounds else {}
    gb.setdefault('vl_bounds', lead.vl_bounds())   # model pins the band unless caller overrides

    print(f"\n[Pipeline] Starting Modular Abstraction (Preset: {grid_preset})...")
    print(f"[Pipeline] Lead: {lead_desc}  (prefix='{prefix}')")
    print(f"[Pipeline] Perception Mode: {'NOISY (Explicit)' if enable_perception_noise else 'PERFECT (Symbolic)'}")
    if grid_preset != 'medium_tight':
        print(f"[Pipeline] NOTE: preset '{grid_preset}' != 'medium_tight'. If you ship "
              "this grid, re-run tests/test_containment_full.py on it.")

    # --- Setup grid ---
    grid = Grid(preset=grid_preset, **gb) if gb else Grid(preset=grid_preset)
    algo = DiscretizationAlgorithm(grid)

    # --- MODULE 1: PLANT ---
    print("[Pipeline] Module 1/3: Vehicle Plant")
    plant_imdp = algo.run(wrapper, name="Plant")
    sink_id = plant_imdp.finalize_sink_state()

    # --- DIAGNOSTIC ONLY: SA13 additive bound E (not the certificate) ---
    N_HORIZON = 10
    report = compute_global_error_bound(plant_imdp, N_HORIZON)
    print(f"[Pipeline] (diagnostic) worst-row mean interval half-width K = {report['max_K']:.6f}")
    print(f"[Pipeline] (diagnostic) SA13 additive bound E = N*K = {report['E']:.6f}  (N={N_HORIZON})")
    print(f"[Pipeline] (diagnostic) worst (state, action) = ({report['worst_state']}, {report['worst_action']})")
    print("[Pipeline] (diagnostic) E is descriptive only; soundness is via containment, not E.")

    # Labeling (Phase 1: crash state s=0)
    plant_imdp.add_label("crash", 0)

    # --- MODULE 2: CONTROLLER ---
    print("[Pipeline] Module 2/3: AEBS Controller (Synthesis)")
    ctrl_model = ControllerModel(name="Controller", input_var="y", output_var="u",
                                 variable_def="u : [0..2] init 0;")
    action_map = {'coast': 0, 'brake_warn': 1, 'brake_full': 2}
    grammar = LabelingGrammar(controller)

    for idx in itertools.product(*[range(d) for d in grid.shape]):
        flat_id = grid.get_flat_index(idx)
        center = grid.index_to_cell_center(idx)
        gap, v_ego, v_lead = center[0], center[1], center[2]
        v_rel = v_ego - v_lead
        act_name = controller.get_action_name_for_state(gap, v_rel, 0, plant_coords='relative_frame')
        ctrl_model.add_rule(flat_id, action_map.get(act_name, 0))
        for t in grammar.get_labels(center):
            plant_imdp.add_label(t, flat_id)

    crash_states = plant_imdp.labels.get("crash", set())
    safe_states = [i for i in range(1, sink_id) if i not in crash_states]
    plant_imdp.add_bulk_label("safe_physics", safe_states)
    ctrl_model.add_rule(sink_id, 0)

    # --- Scenario cheat sheet ---
    print("\n" + "=" * 66)
    print("   SCENARIO ID CHEAT SHEET (Use these in PRISM -const start_s=ID)")
    print("=" * 66)
    print(f" {'SCENARIO':<22} | {'STATE (Gap, V, Vl)':<20} | {'Action':<12} | {'ID':>7}")
    print("-" * 66)
    scenarios = {}
    for name, physical_state in scenarios_list:
        idx = grid.state_to_index(physical_state)
        if idx is None:
            print(f" {name:<22} | {str(physical_state):<20} | {'OUT OF BOUNDS':<12} | {'N/A':>7}")
            continue
        flat_id = grid.get_flat_index(idx)
        c = grid.index_to_cell_center(idx)
        act_name = controller.get_action_name_for_state(c[0], c[1] - c[2], 0, plant_coords='relative_frame')
        scenarios[name] = flat_id
        print(f" {name:<22} | ({c[0]:>4.0f},{c[1]:>4.0f},{c[2]:>4.0f}){'':<8} | {act_name:<12} | {flat_id:>7}")
    print("=" * 66 + "\n")

    plant_imdp.initial_state = "start_s"

    # --- MODULE 3: PERCEPTION ---
    print("[Pipeline] Module 3/3: Perception Layer")
    perception = PerceptionModel(name="Perception", input_var="s", output_var="y", max_state=sink_id)
    perception.enable_noise = enable_perception_noise

    # --- EXPORT ---
    if output_path is None:
        output_path = f"artifacts/{prefix}_modular_system.prism"
    PrismModelGenerator(output_path).generate(
        modules=[plant_imdp, perception, ctrl_model],
        globals_dict={}, constants={'start_s': 'int'})

    print(f"[Pipeline] Wrote {output_path}")
    return output_path
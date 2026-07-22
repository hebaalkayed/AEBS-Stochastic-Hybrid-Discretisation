"""Resolution sweep: how fast do the [Pmin, Pmax] brackets separate as the
partition slack shrinks? Everything pinned except the swept resolutions:
three-point sweep over gap resolution plus one ego-channel probe.
Domain reduced so the finest grid stays evening-sized. Imminent_Collision
is deliberately out of bounds (already-decided [1,1] state).
NOTE: sweep grids are exploratory, NOT covered by the production containment
certificate; any grid promoted from here must be re-certified first.
"""
import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.types.grid import GRID_PRESETS
from src.abstraction.pipeline import run_modular_abstraction

SWEEP_PRESETS = {
    'sweep_r100':  {'res': (1.0,  0.5,  0.1), 'desc': "Sweep: gap 1.0 m (baseline slack)"},
    'sweep_r050':  {'res': (0.5,  0.5,  0.1), 'desc': "Sweep: gap 0.5 m"},
    'sweep_r025':  {'res': (0.25, 0.5,  0.1), 'desc': "Sweep: gap 0.25 m"},
    'sweep_r050v': {'res': (0.5,  0.25, 0.1), 'desc': "Sweep: gap 0.5 m + v_ego 0.25 m/s (ego probe)"},
}
GRID_PRESETS.update(SWEEP_PRESETS)

SWEEP_BOUNDS = {
    'x_bounds':  (0, 100),
    'v_bounds':  (0, 16),
    'vl_bounds': (-1.0, 3.0),
}


def main():
    os.makedirs('artifacts', exist_ok=True)
    for preset, spec in SWEEP_PRESETS.items():
        out = f"artifacts/sweep_{preset}.prism"
        print(f"\n===== SWEEP {preset}: res={spec['res']} bounds={SWEEP_BOUNDS} =====", flush=True)
        t0 = time.time()
        run_modular_abstraction(
            VehiclePlant(coordinate_system='relative_frame'),
            AEBSController(mode='industry'),
            grid_preset=preset,
            grid_bounds=dict(SWEEP_BOUNDS),
            enable_perception_noise=False,
            output_path=out,
            lead_model='static',
        )
        print(f"===== {preset} done in {time.time() - t0:.1f}s =====", flush=True)


if __name__ == '__main__':
    main()

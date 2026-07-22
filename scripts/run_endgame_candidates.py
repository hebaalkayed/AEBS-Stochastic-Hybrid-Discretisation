"""Overnight generation of two endgame candidate grids on the scenario
window. Resolution to be confirmed against the sweep results tomorrow;
these two bracket the plausible range. Imminent_Collision is deliberately
out of bounds: it is the already-decided [1,1] state, certified and
reported on the production medium_tight model.
NOTE: candidates are NOT certified until tests/test_containment_full.py
is re-run on the chosen grid; only certified grids ship.
"""
import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.types.grid import GRID_PRESETS
from src.abstraction.pipeline import run_modular_abstraction

ENDGAME_PRESETS = {
    'endgame_a': {'res': (0.25, 0.25, 0.1), 'desc': "Endgame candidate A (1.07M cells)"},
    'endgame_b': {'res': (0.1,  0.1,  0.1), 'desc': "Endgame candidate B (6.6M cells)"},
}
GRID_PRESETS.update(ENDGAME_PRESETS)

BOUNDS = {
    'x_bounds':  (0, 100),
    'v_bounds':  (0, 16),
    'vl_bounds': (-1.0, 3.0),
}


def main():
    os.makedirs('artifacts', exist_ok=True)
    for preset, spec in ENDGAME_PRESETS.items():
        out = f"artifacts/{preset}.prism"
        print(f"\n===== ENDGAME {preset}: res={spec['res']} bounds={BOUNDS} =====", flush=True)
        t0 = time.time()
        run_modular_abstraction(
            VehiclePlant(coordinate_system='relative_frame'),
            AEBSController(mode='industry'),
            grid_preset=preset,
            grid_bounds=dict(BOUNDS),
            enable_perception_noise=False,
            output_path=out,
            lead_model='static',
        )
        print(f"===== {preset} done in {time.time() - t0:.1f}s =====", flush=True)


if __name__ == '__main__':
    main()

"""Regenerate only the sweep_r050v model (ego-channel probe). Its .prism
survived the disk-full incident; the .drn write was the casualty, and the
pipeline always writes both, so the whole model is rebuilt for a consistent
pair. Same preset, bounds, controller, and lead as the original sweep run.
"""
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.types.grid import GRID_PRESETS
from src.abstraction.pipeline import run_modular_abstraction

GRID_PRESETS['sweep_r050v'] = {'res': (0.5, 0.25, 0.1),
                               'desc': "Sweep: gap 0.5 m + v_ego 0.25 m/s (ego probe)"}

run_modular_abstraction(
    VehiclePlant(coordinate_system='relative_frame'),
    AEBSController(mode='industry'),
    grid_preset='sweep_r050v',
    grid_bounds={'x_bounds': (0, 100), 'v_bounds': (0, 16), 'vl_bounds': (-1.0, 3.0)},
    enable_perception_noise=False,
    output_path="artifacts/sweep_sweep_r050v.prism",
    lead_model='static',
)

"""Generate the extra_fine model: Following window at control-visible
resolution. Preset 'extra_fine' and the Following scenarios are repo code."""
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.pipeline import run_modular_abstraction

run_modular_abstraction(
    VehiclePlant(coordinate_system='relative_frame'),
    AEBSController(mode='industry'),
    grid_preset='extra_fine',
    grid_bounds={'x_bounds': (0, 16), 'v_bounds': (4, 16), 'vl_bounds': (5.0, 15.0)},
    enable_perception_noise=False,
    output_path="artifacts/extra_fine.prism",
    lead_model='static',
)

import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.poc_system.poc_vehicle_plant import POCVehiclePlant
from src.poc_system.poc_controller import POCController
from src.abstraction.pipeline import run_modular_abstraction


def main():
    print("--- STARTING MODULAR ABSTRACTION EXPERIMENT ---")

    # ================================================================
    # Toggle this to switch between the two systems:
    #   True  = 72-state micro-world POC (hand-traceable, for validation)
    #   False = Full production system (~100k states)
    # ================================================================
    RUN_POC = True

    ENABLE_NOISE = False

    if RUN_POC:
        print("Mode: MICRO-WORLD POC (72 states)")

        plant = POCVehiclePlant(dt=1.0, alpha=1.0)
        controller = POCController()

        grid_preset = 'micro'
        grid_bounds = {
            'x_bounds':  (0, 16),    # 16m road → 9 gap cells (0,2,...,16)
            'v_bounds':  (0, 6),     # 0-6 m/s  → 4 v_ego cells (0,2,4,6)
            'vl_bounds': (-2, 3),    # 3-sigma coverage → 6 v_lead cells (-2,-1,0,1,2,3)
        }                            # captures 99.6% of mass (was 64.6% with (0,1))
        lead_noise = 0.5     # scaled for micro-world (production uses 2.0)
        out_path = "artifacts/micro_world_system.prism"

    else:
        print("Mode: FULL PRODUCTION SYSTEM")

        plant = VehiclePlant(coordinate_system='relative_frame')
        controller = AEBSController(mode='safe')

        grid_preset = 'fast_medium'
        grid_bounds = None
        lead_noise = 2.0
        out_path = "artifacts/modular_system.prism"

    print(f"Configuration: Preset='{grid_preset}' | Noise={ENABLE_NOISE}")

    start_time = time.time()

    output_file = run_modular_abstraction(
        plant,
        controller,
        grid_preset=grid_preset,
        grid_bounds=grid_bounds,
        enable_perception_noise=ENABLE_NOISE,
        output_path=out_path,
        lead_noise_std=lead_noise,
    )

    elapsed = time.time() - start_time
    print(f"SUCCESS! Generation took {elapsed:.2f} seconds.")
    print(f"Generated PRISM file: {os.path.abspath(output_file)}")


if __name__ == "__main__":
    main()
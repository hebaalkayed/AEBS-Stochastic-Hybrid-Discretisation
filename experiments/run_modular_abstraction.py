import sys
import os
import time 

# Add project root to python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.pipeline import run_modular_abstraction

def main():
    print("--- STARTING MODULAR ABSTRACTION EXPERIMENT ---")
    
    # --- EXPERIMENT CONFIGURATION ---
    GRID_PRESET = 'medium'  # Options: 'coarse', 'medium', 'fine'
    
    # Toggle this to switch modes:
    # False = Fast generation, Perfect Perception (Symbolic)
    # True  = Slower generation, Noisy Perception (Explicit State Map)
    ENABLE_NOISE = False 
    
    print(f"Configuration: Preset='{GRID_PRESET}' | Noise={ENABLE_NOISE} | Cores=ALL")

    plant = VehiclePlant(coordinate_system='relative_frame')
    controller = AEBSController(mode='safe')
    
    start_time = time.time()
    
    output_file = run_modular_abstraction(
        plant, 
        controller, 
        grid_preset=GRID_PRESET,
        enable_perception_noise=ENABLE_NOISE
    )
    
    end_time = time.time()
    duration = end_time - start_time
    
    print(f"SUCCESS! Generation took {duration:.2f} seconds ({duration/60:.2f} minutes).")
    print(f"Generated PRISM file: {os.path.abspath(output_file)}")

if __name__ == "__main__":
    main()
import sys
import os

# Add project root to python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.pipeline import run_modular_abstraction

def main():
    print("--- STARTING MODULAR ABSTRACTION EXPERIMENT ---")
    
    # 1. Setup Concrete System
    # We use 'coarse' for a fast test run (approx 1-2 mins)
    # Switch to 'medium' for the final thesis generation (approx 15 mins)
    grid_preset = 'coarse' 
    
    plant = VehiclePlant(coordinate_system='relative_frame')
    controller = AEBSController(mode='safe')
    
    # 2. Run the Pipeline
    output_file = run_modular_abstraction(plant, controller, grid_preset)
    
    print(f"Generated PRISM file: {os.path.abspath(output_file)}")

if __name__ == "__main__":
    main()
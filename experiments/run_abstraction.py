import sys
import os
import time

# --- PATH SETUP ---
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# --- IMPORTS ---
from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.grid import Grid
from src.abstraction.pipeline import execute_abstraction

def run_experiment(mode):
    print(f"\n{'='*60}")
    print(f" EXPERIMENT: Generating Formal Model for '{mode.upper()}' Controller")
    print(f"{'='*60}")

    # 1. Setup Physics (Relative Frame is mandatory for PRISM)
    plant = VehiclePlant(coordinate_system='relative_frame')

    # 2. Setup Grid
    # Use 'medium' for thesis quality, 'coarse' for fast checks
    grid = Grid(preset='medium') 

    # 3. Setup Logic
    controller = AEBSController(mode=mode)
    
    # 4. Run Pipeline
    mdp = execute_abstraction(plant, grid, controller)

    # 5. Export
    # Save artifacts relative to the project root, not the experiments folder
    output_path = os.path.join(os.path.dirname(__file__), '..', 'artifacts', f'{mode}_model.prism')
    mdp.to_prism(output_path)
    
    print(f"SUCCESS: Model saved to {os.path.abspath(output_path)}")

def main():
    # Run both
    run_experiment('industry')
    run_experiment('safe')

if __name__ == "__main__":
    main()
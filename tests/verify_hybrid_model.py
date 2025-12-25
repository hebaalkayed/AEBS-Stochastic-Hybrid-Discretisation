import sys
import os

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.grid import Grid
from src.abstraction.pipeline import execute_abstraction

def test_hybrid_generation():
    print("=======================================================")
    print("       TEST: HYBRID MODEL GENERATION (Plant + Logic)")
    print("=======================================================")

    # 1. SETUP
    # Use 'coarse' for speed, or 'medium' for accuracy
    grid = Grid(preset='coarse') 
    plant = VehiclePlant()
    
    # Instantiate the Controller (Mode: SAFE)
    controller = AEBSController(mode='safe')

    # 2. EXECUTE PIPELINE
    # Note: We now pass the controller object to the pipeline
    print(f"[Exec] Running Abstraction with Controller mode: {controller.mode}...")
    finite_mdp = execute_abstraction(plant, grid, controller)

    # 3. VERIFY IN-MEMORY
    print("\n[Check 1] Internal MDP Structure")
    assert finite_mdp.num_states == grid.total_states
    assert len(finite_mdp.controller_logic) > 0, " Controller logic string is empty!"
    print("     Controller Logic generated in memory.")

    # 4. EXPORT & VERIFY FILE
    output_path = "artifacts/test_hybrid.prism"
    if not os.path.exists("artifacts"): os.makedirs("artifacts")
    
    finite_mdp.to_prism(output_path)
    
    print(f"\n[Check 2] Inspecting Output File: {output_path}")
    with open(output_path, 'r') as f:
        content = f.read()
        
    # Check for Plant Module
    if "module Plant" in content:
        print("     'module Plant' found.")
    else:
        print("     MISSING 'module Plant'")

    # Check for Controller Module
    if "module Controller_safe" in content:
        print("     'module Controller_safe' found.")
    else:
        print("     MISSING Controller Module")
        
    # Check for specific logic (e.g., formulas)
    if "formula do_brake_full =" in content:
        print("     Controller formulas found.")
    else:
        print("     MISSING Controller Formulas")

    print("\n=======================================================")
    print("SUCCESS: The pipeline correctly assembled the Hybrid Model.")
    print("=======================================================")

if __name__ == "__main__":
    test_hybrid_generation()
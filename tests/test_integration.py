import sys
import os

# Add project root to path so we can see 'src'
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant
from src.abstraction.grid import Grid
# UPDATED IMPORT: Pointing to the new location
from src.abstraction.pipeline import execute_abstraction

def test_full_pipeline():
    """
    Integration Test:
    1. Instantiates the physics plant.
    2. Sets up a 'medium' grid.
    3. Runs the full Abstraction Pipeline.
    4. Verifies the output MDP is valid.
    5. Exports the artifact to 'artifacts/verified_model.prism'.
    """
    print("=======================================================")
    print("       TEST: FULL ABSTRACTION PIPELINE")
    print("=======================================================")

    # 1. SETUP
    # Using 'medium' preset as validated in compare_abstractions.py
    plant = VehiclePlant(alpha=0.5, dt=0.1)
    grid = Grid(preset='medium') 

    # 2. EXECUTE
    # This runs the heavy lifting defined in src/abstraction/pipeline.py
    finite_mdp = execute_abstraction(plant, grid)

    # 3. VERIFY
    print("\n[Verification] Inspecting Output MDP...")
    
    # Check 1: State Count
    assert finite_mdp.num_states == grid.total_states
    print(f"    ✅ State count matches grid ({finite_mdp.num_states}).")

    # Check 2: Transitions Exist
    total_transitions = sum(len(t) for t in finite_mdp.transitions.values())
    assert total_transitions > 0
    print(f"    ✅ Generated {total_transitions} transitions.")

    # 4. EXPORT ARTIFACT
    output_dir = "artifacts"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    output_file = os.path.join(output_dir, "model.prism")
    
    print(f"\n[Artifact] Saving to {output_file}...")
    finite_mdp.to_prism(output_file)
    
    print(f"\n[Success] Pipeline finished successfully.")
    print("=======================================================")

if __name__ == "__main__":
    test_full_pipeline()
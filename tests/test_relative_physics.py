import sys
import os
import numpy as np

# Add src to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.system.vehicle_plant import VehiclePlant

def test_dual_modes():
    print("--- TESTING DUAL-MODE PHYSICS ---")
    dt = 0.1
    v_init = 20.0
    
    # CASE 1: WORLD FRAME (Simulation)
    # x = Position on track. Positive velocity means x INCREASES.
    plant_world = VehiclePlant(x=0.0, v=v_init, dt=dt, coordinate_system='world_frame')
    plant_world.step(0.0) # Coast
    
    print(f"World Frame: Start=0.0 -> End={plant_world.x:.2f}")
    if plant_world.x > 0.0:
        print(" World Frame Logic Correct (Position Increased)")
    else:
        print(" World Frame Logic FAILED")

    # CASE 2: RELATIVE FRAME (Verification)
    # x = Gap. Positive velocity (closing speed) means x DECREASES.
    plant_rel = VehiclePlant(x=100.0, v=v_init, dt=dt, coordinate_system='relative_frame')
    plant_rel.step(0.0) # Coast
    
    print(f"Relative Frame: Start=100.0 -> End={plant_rel.x:.2f}")
    if plant_rel.x < 100.0:
        print(" Relative Frame Logic Correct (Gap Decreased)")
    else:
        print(" Relative Frame Logic FAILED (Gap did not shrink!)")

if __name__ == "__main__":
    test_dual_modes()
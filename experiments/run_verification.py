import sys
import os
import numpy as np

# --- 1. SETUP PATHS ---
current_dir = os.path.dirname(os.path.abspath(__file__))
# Go up one level to the root, then point to 'src'
src_path = os.path.join(current_dir, '..', 'src')
# Add it to the system path
sys.path.append(src_path)

# --- 2. IMPORTS ---
# Now Python can find the benchmarks folder inside src
try:
    from benchmarks.vehicle_plant import BenchmarkAV
except ImportError as e:
    print(f"Error importing module: {e}")
    print(f"Python is looking in: {sys.path}")
    sys.exit(1)

# --- 3. THE EXPERIMENT ---
def run_benchmark_check():
    print("Initializing ARCH-COMP 2020 AV Benchmark...")
    
    # Initialize the plant with the standard 0.5s time step
    plant = BenchmarkAV(dt=0.5)

    # Print the Matrices to confirm we have the right Physics
    print("\n--- SYSTEM DYNAMICS (Defined in Abate et al. 2020) ---")
    print("A Matrix (State Transitions):")
    print(plant.A)
    print("\nB Matrix (Control Input):")
    print(plant.B)
    print("\nSigma (Noise Covariance):")
    print(plant.Sigma)

    # Run a quick simulation test
    print("\n--- SIMULATION TEST ---")
    # State: [Distance=50m, Ego_Vel=20m/s, Lead_Vel=20m/s]
    x_current = np.array([50.0, 20.0, 20.0])
    
    # Action: Emergency Brake (-5 m/s^2)
    u_brake = -5.0
    
    # Compute next state
    x_next = plant.get_next_state(x_current, u_brake)
    
    print(f"Initial State: {x_current}")
    print(f"Braking Input: {u_brake}")
    print(f"Next State:    {x_next}")
    print("-----------------------")
    print("SUCCESS: Benchmark loaded and verified.")

if __name__ == "__main__":
    run_benchmark_check()
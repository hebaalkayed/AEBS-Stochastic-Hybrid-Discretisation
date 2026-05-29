"""
Sigma sweep diagnostic.

Maps K(sigma/w) for the SA13 range bound across a range of lead-vehicle
noise values, on a fixed grid. Each run reports K for one edge cell and
one interior cell — if the theory holds, they agree to ~5 decimal places.

Used to characterise the SA13 range bound's asymptotic behaviour:
    K * (sigma/w) -> 2/sqrt(2*pi) ≈ 0.798 for sigma/w >> 1.

Run from the repo root:
    python experiments/diagnostics/sigma_sweep.py
"""
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.types.grid import Grid
from src.abstraction.modules.wrappers import PlantWrapper

from experiments.diagnostics.dump_row import dump_row


def sigma_sweep(plant, controller, grid_preset, grid_bounds=None):
    """
    Maps K vs sigma/w for the worst row (state 5135) and an interior row.

    Skips the full ~2M-state discretisation — just builds the grid,
    instantiates the wrapper at each noise value, and calls dump_row.
    Each iteration is fast (seconds, not minutes).
    """
    print("\n" + "=" * 72)
    print("  SIGMA SWEEP — mapping K(sigma/w) for the SA13 range bound")
    print("=" * 72)

    if grid_bounds:
        grid = Grid(preset=grid_preset, **grid_bounds)
    else:
        grid = Grid(preset=grid_preset)

    w_vlead = grid.resolution[2]
    print(f"\n[Sweep] v_lead cell width w = {w_vlead}")
    print(f"[Sweep] Each row reports K (in-grid) and K (with sink).")
    print(f"[Sweep] If the theory holds, edge and interior K should agree to ~5dp.\n")

    # lead_noise values chosen so sigma/w spans ~0.25 to ~8.
    # sigma_on_velocity ≈ lead_noise * 0.1  (for lead_noise >> plant.noise_std)
    sweep_values = [0.25, 0.5, 1.0, 2.0, 4.0, 6.0, 8.0]

    for lead_noise in sweep_values:
        print("\n" + "-" * 72)
        print(f"  lead_noise_std = {lead_noise}  m/s^2")
        print("-" * 72)
        pw = PlantWrapper(plant, controller, lead_noise_std=lead_noise)
        sigma_v = pw.sigma_on_velocity
        ratio = sigma_v / w_vlead
        print(f"[Sweep] sigma_on_velocity = {sigma_v:.5f} m/s   sigma/w = {ratio:.3f}")

        dump_row(5135, 0, grid, pw, label=f"edge   (sigma/w={ratio:.2f})")
        interior_idx = grid.state_to_index((10.0, 10.0, 10.0))
        if interior_idx is not None:
            interior = grid.get_flat_index(interior_idx)
            dump_row(interior, 0, grid, pw,
                     label=f"interior (sigma/w={ratio:.2f})")

    print("\n" + "=" * 72)
    print("  SWEEP COMPLETE — look at the K(with sink) column across runs.")
    print("  Expected: K * (sigma/w) converges to 2/sqrt(2*pi) ≈ 0.798.")
    print("=" * 72 + "\n")


def main():
    plant = VehiclePlant(coordinate_system='relative_frame')
    controller = AEBSController(mode='safe')
    sigma_sweep(plant, controller, grid_preset='medium_tight')


if __name__ == "__main__":
    main()
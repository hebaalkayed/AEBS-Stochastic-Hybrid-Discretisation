"""Row-for-row kernel equivalence check between two discretizer files.

Purpose: certify that a refactor of the discretizer changed no probability.
Loads OLD and NEW discretizer implementations from explicit file paths,
builds the grid and wrapper from the CURRENTLY INSTALLED package, feeds the
SAME bins and the SAME image box to both kernels, and requires the returned
transition dictionaries to be exactly equal (bitwise float equality) for
every (source cell, action) row.

Usage (from the repo root, with the venv active):
  python tests/verify_refactor_equivalence.py OLD_PATH NEW_PATH
  GRID_PRESET=coarse python tests/verify_refactor_equivalence.py ...
  MAX_CELLS=5000     python tests/verify_refactor_equivalence.py ...

Notes:
- Keep a copy of the pre-refactor discretizer BEFORE overwriting it, e.g.
  copy src\\abstraction\\algorithms\\discretizer.py C:\\Temp\\discretizer_old.py
  (do this by file copy, not by PowerShell '>' redirection, which writes a
  UTF-16 BOM that breaks importing).
- The NEW file may import src.abstraction.types.cell_topology; the package
  must therefore already contain it when this script runs.
- Default preset is coarse (about a minute per core, exhaustive). MAX_CELLS
  subsamples but ALWAYS includes the top-gap boundary slab, where the escape
  rows live.
"""
import sys, os, time, itertools
import importlib.util

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    old_path, new_path = sys.argv[1], sys.argv[2]
    preset = os.environ.get("GRID_PRESET", "coarse")
    lead_name = os.environ.get("LEAD_MODEL", "static")
    max_cells = int(os.environ.get("MAX_CELLS", "0"))   # 0 = exhaustive

    old = load_module("disc_old", old_path)
    new = load_module("disc_new", new_path)

    from src.system.vehicle_plant import VehiclePlant
    from src.system.controller import AEBSController
    from src.abstraction.modules.wrappers import PlantWrapper
    from src.abstraction.profiles.lead_model import get_lead_model
    from src.abstraction.types.grid import Grid

    lead = get_lead_model(lead_name)
    grid = Grid(preset=preset, vl_bounds=lead.vl_bounds())
    wrapper = PlantWrapper(VehiclePlant(coordinate_system="relative_frame"),
                           AEBSController(mode="safe"), lead_model=lead)
    actions = wrapper.get_action_space()
    bins = [np.asarray(b, float) for b in grid.bins]
    shape = tuple(grid.shape)
    total = int(grid.total_states)

    all_cells = list(itertools.product(*[range(s) for s in shape]))
    if max_cells and max_cells < len(all_cells):
        # always include the top-gap slab (escape territory), then random rest
        top_slab = [c for c in all_cells if c[0] == shape[0] - 1]
        rng = np.random.default_rng(20260709)
        rest = [all_cells[i] for i in
                rng.choice(len(all_cells), size=max(0, max_cells - len(top_slab)),
                           replace=False)]
        cells = top_slab + rest
        print(f"Subsampled: {len(top_slab)} top-gap cells + {len(rest)} random "
              f"of {len(all_cells):,}")
    else:
        cells = all_cells
        print(f"Exhaustive: {len(cells):,} cells x {len(actions)} actions")

    t0 = time.time()
    mismatches = 0
    for n_done, idx in enumerate(cells, 1):
        c_lo = [bins[d][idx[d]] for d in range(3)]
        c_hi = [bins[d][idx[d] + 1] for d in range(3)]
        for a_id, a_val in actions.items():
            lo, hi, spd = new.make_image_box(wrapper, c_lo, c_hi, a_val)
            if old._compute_kernel_accurate(lo, hi, spd, bins, shape, total) != \
               new._compute_kernel_accurate(lo, hi, spd, bins, shape, total):
                mismatches += 1
                if mismatches <= 5:
                    print(f"MISMATCH at cell {idx}, action {a_id}")
        if n_done % 10000 == 0:
            rate = n_done / (time.time() - t0)
            print(f"  {n_done:,}/{len(cells):,} cells | {rate:,.0f} cells/s "
                  f"| mismatches {mismatches}", flush=True)

    dt = time.time() - t0
    print(f"\nDONE: {len(cells):,} cells x {len(actions)} actions in {dt:.0f}s")
    print("PASS: every transition row identical between the two kernels."
          if mismatches == 0 else
          f"FAIL: {mismatches} mismatching rows. Do NOT adopt the new kernel.")
    sys.exit(0 if mismatches == 0 else 1)


if __name__ == "__main__":
    main()
"""
EXHAUSTIVE containment test — checks every (source cell, action) transition of
the injected lead-model abstraction against sampled true probabilities.

Injected-only: the lead
behaviour comes from a LeadModel, default 'static', overridable with
LEAD_MODEL=steady. The grid v_lead band is pinned to the model's vl_bounds.
PRESET defaults to the certified grid (medium_tight) and is overridable with
GRID_PRESET=coarse etc.; keep it on the grid you intend to ship.

Check classes (v2):
  sample        true one-step probability at a sampled state falls outside the
                stored interval for a target the true distribution reaches
                (the original, one-sided check); sampled states span the
                HALF-OPEN cell, lower faces exact, upper faces at the nearest
                representable in-cell value
  lower_vs_zero a stored lower bound exceeds tolerance for a target that at
                least one sampled state reaches with (numerically) zero
                probability — catches wrongly-certain coverage, e.g. the
                boundary-exact certain-coverage class, which the one-sided
                check is structurally blind to
  escape        a sampled state's deterministic image leaves the grid; its
                true row is {sink: 1.0} and the stored row must contain it —
                catches missing escape routing
  empty_row     a (cell, action) row with no transitions at all — a missing
                PRISM command, i.e. a deadlock in the composed model
  infeasible    row interval sums admit no probability distribution
                (sum of uppers < 1 or sum of lowers > 1)

NOTE: this validates the in-memory kernel (make_image_box + _compute_kernel_
accurate), not the generated .prism file. A file-parsing containment test is on
the roadmap to close that gap.

Streaming, persistent, resumable. Progress and violation CSVs are bound to
the RUN IDENTITY: lead model, grid preset, partition fingerprint (bins_hash),
kernel source hash, and test version. A change to any of these produces new
CSV filenames, so a certificate can never silently resume progress recorded
against a different grid, a different kernel, or a weaker test. Legacy
per-model CSVs (containment_progress_static.csv etc.) are ignored.
Run:  python -m tests.test_containment_full
      LEAD_MODEL=steady python -m tests.test_containment_full
      GRID_PRESET=coarse python -m tests.test_containment_full
"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
import math
import itertools
import csv
import time
import hashlib
from pathlib import Path
from multiprocessing import Pool, cpu_count

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.modules.wrappers import PlantWrapper
from src.abstraction.algorithms.discretizer import (
    _compute_kernel_accurate, make_image_box, fast_norm_cdf, TRUNCATE_NOISE,
)
from src.abstraction.types.cell_topology import cell_of, flat_index
from src.abstraction.types.grid import Grid

# ---------------- configuration ----------------
PRESET          = os.environ.get("GRID_PRESET", "medium_tight")
LEAD_MODEL      = os.environ.get("LEAD_MODEL", "static")   # injected-only; default shipped model
CHUNK_SIZE      = 2000
SAMPLES_PER_DIM = (3, 3, 3)
TOLERANCE       = 1e-7
CRASH_WALL      = 0.0
TEST_VERSION    = 3          # bump when the checking logic changes
_tag            = LEAD_MODEL
# CSV filenames are derived per run inside main(): they encode the lead
# model, preset, bins_hash, kernel source hash, and TEST_VERSION.
# ------------------------------------------------


def build_wrapper():
    plant = VehiclePlant(coordinate_system="relative_frame")
    controller = AEBSController(mode="safe")
    from src.abstraction.profiles.lead_model import get_lead_model
    return PlantWrapper(plant, controller, lead_model=get_lead_model(LEAD_MODEL))


def build_grid():
    from src.abstraction.profiles.lead_model import get_lead_model
    return Grid(preset=PRESET, vl_bounds=get_lead_model(LEAD_MODEL).vl_bounds())


def true_transition_probs(wrapper, s, a_val, bins, shape, total):
    """True one-step transition probabilities of the modelling kernel from
    state s under a_val. Escaped deterministic images (out of grid, other
    than through the crash wall) map to the sink, {total: 1.0}, so that
    missing escape routing in the stored row is a detectable violation
    rather than a silent skip."""
    nd, spd = wrapper.get_next_state_distribution(np.asarray(s, dtype=float), a_val)
    if nd[0] <= CRASH_WALL:
        return {0: 1.0}

    # half-open assignment via the shared convention module: the convention
    # is a DEFINITION shared with the kernel; the probability arithmetic
    # below stays independent of it.
    jg, jv = cell_of(nd[0], bins[0]), cell_of(nd[1], bins[1])
    if jg is None or jv is None:
        return {total: 1.0}          # deterministic image left the grid

    sv = spd[2]
    inside = (fast_norm_cdf(bins[2][-1], nd[2], sv) - fast_norm_cdf(bins[2][0], nd[2], sv)) \
        if TRUNCATE_NOISE else 1.0
    nr  = 7.0 * sv
    jlo = max(0, int(np.digitize(nd[2] - nr, bins[2]) - 1))
    jhi = min(shape[2] - 1, int(np.digitize(nd[2] + nr, bins[2]) - 1))

    out = {}
    for jl in range(jlo, jhi + 1):
        p = fast_norm_cdf(bins[2][jl + 1], nd[2], sv) - fast_norm_cdf(bins[2][jl], nd[2], sv)
        if TRUNCATE_NOISE and inside > 0:
            p /= inside
        if p > 1e-9:
            out[flat_index(jg, jv, jl, shape)] = p
    return out


def check_chunk(args):
    chunk_id, cell_indices, bins, shape, total = args
    wrapper = build_wrapper()
    actions = wrapper.get_action_space()
    ng, nv, nl = SAMPLES_PER_DIM

    n_cells = n_viol = 0
    worst = 0.0
    worst_rec = None

    def record(excess, rec):
        nonlocal n_viol, worst, worst_rec
        n_viol += 1
        if excess > worst:
            worst = excess
            worst_rec = rec

    for idx in cell_indices:
        n_cells += 1
        src_flat = flat_index(idx[0], idx[1], idx[2], shape)
        if src_flat == 0:
            continue
        cell_lo = [bins[d][idx[d]] for d in range(3)]
        cell_hi = [bins[d][idx[d] + 1] for d in range(3)]
        # Sample the HALF-OPEN cell: the lower face is included and sampled
        # exactly; the upper face belongs to the neighbouring cell, so the top
        # sample is the largest representable IN-CELL coordinate. Sampling the
        # upper face itself demands the neighbour's dynamics from this cell's
        # row, which containment does not require, and produces spurious
        # violations where a Dirac dimension's indicator flips exactly at the
        # face (observed at gap-zero cells under braking).
        def cell_samples(d, n):
            s = np.linspace(cell_lo[d], cell_hi[d], n)
            s[-1] = np.nextafter(cell_hi[d], cell_lo[d])
            return s
        gs, vs, ls = cell_samples(0, ng), cell_samples(1, nv), cell_samples(2, nl)

        for a_id, a_val in actions.items():
            nlo, nhi, spd = make_image_box(wrapper, cell_lo, cell_hi, a_val)
            stored = _compute_kernel_accurate(nlo, nhi, spd, bins, shape, total)

            # ---- row-level checks (once per cell and action) ----
            if not stored:
                record(1.0, (src_flat, a_id, -1, 0.0, 0.0, 0.0, "empty_row"))
            else:
                sum_lo = sum(v[0] for v in stored.values())
                sum_hi = sum(v[1] for v in stored.values())
                if sum_hi < 1.0 - TOLERANCE:
                    record(1.0 - sum_hi,
                           (src_flat, a_id, -1, 0.0, round(sum_lo, 6),
                            round(sum_hi, 6), "infeasible"))
                if sum_lo > 1.0 + TOLERANCE:
                    record(sum_lo - 1.0,
                           (src_flat, a_id, -1, 0.0, round(sum_lo, 6),
                            round(sum_hi, 6), "infeasible"))

            # ---- per-sample checks ----
            sample_supports = []
            for g in gs:
                for v in vs:
                    for l in ls:
                        T = true_transition_probs(wrapper, (g, v, l), a_val,
                                                  bins, shape, total)
                        sample_supports.append(set(T.keys()))
                        for j, Tval in T.items():
                            lo, hi = stored.get(j, (0.0, 0.0))
                            excess = max(0.0, Tval - hi, lo - Tval)
                            if excess > TOLERANCE:
                                record(excess,
                                       (src_flat, a_id, j, round(Tval, 6),
                                        round(lo, 6), round(hi, 6),
                                        "escape" if j == total else "sample"))

            # ---- two-sided lower-bound check ----
            # A stored lower bound above tolerance must be honoured by EVERY
            # sampled state. A target absent from a sample's support has true
            # probability at most 1e-9 there (the support filter), so any
            # stored lower bound above TOLERANCE (1e-7) for it is a genuine
            # violation. The original one-sided loop never evaluates these.
            for j, (lo, hi) in stored.items():
                if lo > TOLERANCE and any(j not in sup for sup in sample_supports):
                    record(lo, (src_flat, a_id, j, 0.0, round(lo, 6),
                                round(hi, 6), "lower_vs_zero"))
    return chunk_id, n_cells, n_viol, worst, worst_rec


def load_done(progress_csv):
    done = set()
    if os.path.exists(progress_csv):
        with open(progress_csv, newline="") as f:
            for row in csv.reader(f):
                if row and row[0].isdigit():
                    done.add(int(row[0]))
    return done


def main():
    grid = build_grid()
    bins = [np.asarray(b, dtype=float) for b in grid.bins]
    shape = list(grid.shape)
    total = grid.total_states

    # ---- run identity: everything the certificate depends on ----
    import src.abstraction.algorithms.discretizer as _disc
    kernel_hash = hashlib.sha256(Path(_disc.__file__).read_bytes()).hexdigest()
    run_id = (f"{_tag}_{PRESET}_g{grid.bins_hash[:8]}"
              f"_k{kernel_hash[:8]}_v{TEST_VERSION}")
    progress_csv = f"containment_progress_{run_id}.csv"
    violations_csv = f"containment_violations_{run_id}.csv"
    print(f"[Run] id = {run_id}")
    print(f"[Run] progress file: {progress_csv}")
    for legacy in (f"containment_progress_{_tag}.csv",
                   f"containment_violations_{_tag}.csv"):
        if os.path.exists(legacy):
            print(f"[Run] NOTE: legacy file '{legacy}' belongs to an older run "
                  f"identity and is ignored.")

    all_idx = list(itertools.product(*[range(s) for s in shape]))
    chunks = [(i, all_idx[k:k + CHUNK_SIZE], bins, shape, total)
              for i, k in enumerate(range(0, len(all_idx), CHUNK_SIZE))]

    done = load_done(progress_csv)
    todo = [c for c in chunks if c[0] not in done]

    print(f"Lead='{_tag}' | grid {PRESET} | {len(all_idx):,} cells x 3 actions")
    print(f"Chunks: {len(chunks)} | done {len(done)} | todo {len(todo)}")
    if not todo:
        print("Nothing left to do. Delete the progress CSV to re-run.")
        return

    if not os.path.exists(progress_csv):
        with open(progress_csv, "w") as f:
            f.write(f"# run_id,{run_id}\n")
            f.write("chunk_id,n_cells,n_viol,worst_excess,worst_record\n")
    if not os.path.exists(violations_csv):
        with open(violations_csv, "w") as f:
            f.write(f"# run_id,{run_id}\n")
            f.write("src_flat,action,target,trueT,lo,hi,worst_excess,kind\n")

    total_cells = total_viol = 0
    global_worst = 0.0
    t0 = time.time()
    env_cores = os.environ.get("CONTAINMENT_CORES")
    ncore = int(env_cores) if env_cores else max(1, cpu_count() - 1)
    print(f"Using {ncore} cores.\n")

    try:
        with Pool(ncore) as pool:
            for (cid, ncell, nviol, worst, rec) in pool.imap_unordered(check_chunk, todo):
                total_cells += ncell
                total_viol += nviol
                global_worst = max(global_worst, worst)
                with open(progress_csv, "a") as f:
                    f.write(f'{cid},{ncell},{nviol},{worst:.3e},"{rec}"\n')
                if nviol > 0 and rec is not None:
                    with open(violations_csv, "a") as f:
                        f.write(f"{rec[0]},{rec[1]},{rec[2]},{rec[3]},{rec[4]},"
                                f"{rec[5]},{worst:.3e},{rec[6]}\n")
                elapsed = time.time() - t0
                rate = total_cells / elapsed if elapsed > 0 else 0.0
                print(f"chunk {cid:6d} | cells {total_cells:>9,} | violations {total_viol:>4} | "
                      f"worst {global_worst:.2e} | {rate:6.0f} cells/s", flush=True)
    except KeyboardInterrupt:
        print("\nInterrupted. Progress saved — rerun to resume.")
        return

    print(f"\nDONE [{_tag}]. cells checked this run = {total_cells:,} | "
          f"violations = {total_viol} | worst excess = {global_worst:.2e}")
    print(f"PASS — '{_tag}' model is per-transition sound across its grid."
          if total_viol == 0 else
          f"FAIL — {total_viol} violations; see {violations_csv}")


if __name__ == "__main__":
    main()
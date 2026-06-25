"""
EXHAUSTIVE containment test — checks every (source cell, action) transition of
the injected lead-model abstraction against sampled true probabilities.

Injected-only: the lead
behaviour comes from a LeadModel, default 'static', overridable with
LEAD_MODEL=steady. The grid v_lead band is pinned to the model's vl_bounds.
PRESET selects resolution; keep it on the grid you intend to ship.

NOTE: this validates the in-memory kernel (make_image_box + _compute_kernel_
accurate), not the generated .prism file. A file-parsing containment test is on
the roadmap to close that gap.

Streaming, persistent, resumable (per-model CSVs).
Run:  python -m tests.test_containment_full
      LEAD_MODEL=steady python -m tests.test_containment_full
"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
import math
import itertools
import csv
import time
from multiprocessing import Pool, cpu_count

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.modules.wrappers import PlantWrapper
from src.abstraction.algorithms.discretizer import (
    _compute_kernel_accurate, make_image_box, fast_norm_cdf, TRUNCATE_NOISE,
)
from src.abstraction.types.grid import Grid

# ---------------- configuration ----------------
PRESET          = "coarse"
LEAD_MODEL      = os.environ.get("LEAD_MODEL", "static")   # injected-only; default shipped model
CHUNK_SIZE      = 2000
SAMPLES_PER_DIM = (3, 3, 3)
TOLERANCE       = 1e-7
CRASH_WALL      = 0.0
_tag            = LEAD_MODEL
PROGRESS_CSV    = f"containment_progress_{_tag}.csv"
VIOLATIONS_CSV  = f"containment_violations_{_tag}.csv"
# ------------------------------------------------


def build_wrapper():
    plant = VehiclePlant(coordinate_system="relative_frame")
    controller = AEBSController(mode="safe")
    from src.abstraction.profiles.lead_model import get_lead_model
    return PlantWrapper(plant, controller, lead_model=get_lead_model(LEAD_MODEL))


def build_grid():
    from src.abstraction.profiles.lead_model import get_lead_model
    return Grid(preset=PRESET, vl_bounds=get_lead_model(LEAD_MODEL).vl_bounds())


def flat_of(idx, shape):
    f, m = 0, 1
    for i in reversed(range(3)):
        f += idx[i] * m; m *= shape[i]
    return f


def true_transition_probs(wrapper, s, a_val, bins, shape):
    nd, spd = wrapper.get_next_state_distribution(np.asarray(s, dtype=float), a_val)
    if nd[0] <= CRASH_WALL:
        return {0: 1.0}

    def cell_of(v, d):
        k = int(np.digitize(v, bins[d]) - 1)
        return k if 0 <= k < shape[d] else None
    jg, jv = cell_of(nd[0], 0), cell_of(nd[1], 1)
    if jg is None or jv is None:
        return {}

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
            out[flat_of((jg, jv, jl), shape)] = p
    return out


def check_chunk(args):
    chunk_id, cell_indices, bins, shape, total = args
    wrapper = build_wrapper()
    actions = wrapper.get_action_space()
    ng, nv, nl = SAMPLES_PER_DIM

    n_cells = n_viol = 0
    worst = 0.0
    worst_rec = None

    for idx in cell_indices:
        n_cells += 1
        src_flat = flat_of(idx, shape)
        if src_flat == 0:
            continue
        center = np.array([(bins[d][idx[d]] + bins[d][idx[d] + 1]) / 2 for d in range(3)])
        half = [(bins[d][idx[d] + 1] - bins[d][idx[d]]) / 2 for d in range(3)]
        gs = np.linspace(bins[0][idx[0]], bins[0][idx[0] + 1], ng)
        vs = np.linspace(bins[1][idx[1]], bins[1][idx[1] + 1], nv)
        ls = np.linspace(bins[2][idx[2]], bins[2][idx[2] + 1], nl)

        for a_id, a_val in actions.items():
            nlo, nhi, spd = make_image_box(wrapper, center, half, a_val)
            stored = _compute_kernel_accurate(nlo, nhi, spd, bins, shape, total)
            for g in gs:
                for v in vs:
                    for l in ls:
                        T = true_transition_probs(wrapper, (g, v, l), a_val, bins, shape)
                        for j, Tval in T.items():
                            lo, hi = stored.get(j, (0.0, 0.0))
                            excess = max(0.0, Tval - hi, lo - Tval)
                            if excess > TOLERANCE:
                                n_viol += 1
                                if excess > worst:
                                    worst = excess
                                    worst_rec = (src_flat, a_id, j, round(Tval, 6),
                                                 round(lo, 6), round(hi, 6))
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

    all_idx = list(itertools.product(*[range(s) for s in shape]))
    chunks = [(i, all_idx[k:k + CHUNK_SIZE], bins, shape, total)
              for i, k in enumerate(range(0, len(all_idx), CHUNK_SIZE))]

    done = load_done(PROGRESS_CSV)
    todo = [c for c in chunks if c[0] not in done]

    print(f"Lead='{_tag}' | grid {PRESET} | {len(all_idx):,} cells x 3 actions")
    print(f"Chunks: {len(chunks)} | done {len(done)} | todo {len(todo)}")
    if not todo:
        print("Nothing left to do. Delete the progress CSV to re-run.")
        return

    if not os.path.exists(PROGRESS_CSV):
        with open(PROGRESS_CSV, "w") as f:
            f.write("chunk_id,n_cells,n_viol,worst_excess,worst_record\n")
    if not os.path.exists(VIOLATIONS_CSV):
        with open(VIOLATIONS_CSV, "w") as f:
            f.write("src_flat,action,target,trueT,lo,hi,worst_excess\n")

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
                with open(PROGRESS_CSV, "a") as f:
                    f.write(f'{cid},{ncell},{nviol},{worst:.3e},"{rec}"\n')
                if nviol > 0 and rec is not None:
                    with open(VIOLATIONS_CSV, "a") as f:
                        f.write(f"{rec[0]},{rec[1]},{rec[2]},{rec[3]},{rec[4]},{rec[5]},{worst:.3e}\n")
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
          f"FAIL — {total_viol} violations; see {VIOLATIONS_CSV}")


if __name__ == "__main__":
    main()
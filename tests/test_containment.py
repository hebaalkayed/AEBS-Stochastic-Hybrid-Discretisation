"""
Per-transition containment test for the IMDP abstraction.

SOUNDNESS CONDITION (the only hypothesis the over-approximation theorem needs):
    for every source cell A_i, action a, target cell A_j,
        T(A_j | s, a)  in  [P_lo, P_hi]   for EVERY continuous s in A_i.

This test samples many continuous states inside each source cell, computes the
true transition probability to every reachable target cell, and checks it lies
inside the stored IMDP interval. It is both (a) evidence of soundness for the
panel and (b) a regression guard that auto-catches the deterministic "straddle".

Run:  python -m tests.test_containment
Adjust the imports below to match your package layout if needed.
"""
import numpy as np
import math
import itertools

# --- adjust these imports to your project ---
from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.modules.wrappers import PlantWrapper
from src.abstraction.algorithms.discretizer import (
    _compute_kernel_accurate, make_image_box, fast_norm_cdf,
)
from src.abstraction.types.grid import Grid


def true_transition_probs(wrapper, s, a_val, bins, grid_shape, flat_of,
                          crash_wall=0.0):
    """
    Exact T(A_j | s, a) from a single continuous state s.
    Gap, v_ego deterministic (Dirac); v_lead ~ N(mean, sigma) -> CDF box prob.
    Returns {flat_target_id: prob}.
    """
    nd, sigma_per_dim = wrapper.get_next_state_distribution(np.asarray(s), a_val)
    if nd[0] <= crash_wall:
        return {0: 1.0}

    # deterministic cells
    def cell_of(val, d):
        k = int(np.digitize(val, bins[d]) - 1)
        return k if 0 <= k < grid_shape[d] else None
    jg, jv = cell_of(nd[0], 0), cell_of(nd[1], 1)
    if jg is None or jv is None:
        return {}

    sv = sigma_per_dim[2]
    out = {}
    for jl in range(grid_shape[2]):
        p = fast_norm_cdf(bins[2][jl + 1], nd[2], sv) \
          - fast_norm_cdf(bins[2][jl], nd[2], sv)
        if p > 1e-7:
            out[flat_of((jg, jv, jl))] = p
    return out


def check_cell(wrapper, grid, src_idx, a_id, a_val, n_per_dim=(15, 7, 3)):
    bins, shape = grid.bins, grid.shape
    total_states = grid.total_states

    def flat_of(idx):
        f, m = 0, 1
        for i in reversed(range(3)):
            f += idx[i] * m; m *= shape[i]
        return f

    center = np.array([(bins[d][src_idx[d]] + bins[d][src_idx[d] + 1]) / 2
                       for d in range(3)])
    half = [(bins[d][src_idx[d] + 1] - bins[d][src_idx[d]]) / 2 for d in range(3)]

    next_lo, next_hi, sigma_per_dim = make_image_box(wrapper, center, half, a_val)
    stored = _compute_kernel_accurate(next_lo, next_hi, sigma_per_dim,
                                      bins, shape, total_states)

    ng, nv, nl = n_per_dim
    gs = np.linspace(bins[0][src_idx[0]], bins[0][src_idx[0] + 1], ng)
    vs = np.linspace(bins[1][src_idx[1]], bins[1][src_idx[1] + 1], nv)
    ls = np.linspace(bins[2][src_idx[2]], bins[2][src_idx[2] + 1], nl)

    violations, max_excess, reached = 0, 0.0, set()
    for g in gs:
        for v in vs:
            for l in ls:
                T = true_transition_probs(wrapper, (g, v, l), a_val,
                                          bins, shape, flat_of)
                for j, Tval in T.items():
                    reached.add(j)
                    lo, hi = stored.get(j, (0.0, 0.0))
                    excess = max(0.0, Tval - hi, lo - Tval)
                    if excess > 1e-9:
                        violations += 1
                        max_excess = max(max_excess, excess)

    missed = reached - set(stored.keys())
    slo = sum(p[0] for p in stored.values())
    shi = sum(p[1] for p in stored.values())
    feasible = (slo <= 1.0 + 1e-9) and (shi >= 1.0 - 1e-9)
    return dict(stored=len(stored), reached=len(reached), missed=len(missed),
                violations=violations, max_excess=max_excess, feasible=feasible)


def run(preset="medium_tight", n_sample_cells=40, seed=0):
    rng = np.random.default_rng(seed)
    plant = VehiclePlant(coordinate_system="relative_frame")
    controller = AEBSController(mode="safe")
    wrapper = PlantWrapper(plant, controller, lead_noise_std=2.0)
    grid = Grid(preset=preset)
    actions = wrapper.get_action_space()

    # sample interior source cells (avoid the absorbing crash cell id 0)
    idx_pool = [tuple(rng.integers(0, grid.shape[d]) for d in range(3))
                for _ in range(n_sample_cells)]

    total_viol, worst = 0, 0.0
    print(f"{'cell (g,v,l)':>16} {'act':>4} {'stored':>7} {'reached':>8} "
          f"{'missed':>7} {'viol':>6} {'max_excess':>11} {'feasible':>9}")
    print("-" * 80)
    for idx in idx_pool:
        for a_id, a_val in actions.items():
            r = check_cell(wrapper, grid, idx, a_id, a_val)
            total_viol += r["violations"]; worst = max(worst, r["max_excess"])
            if r["violations"] or r["missed"]:
                print(f"{str(idx):>16} {a_id:>4} {r['stored']:>7} {r['reached']:>8} "
                      f"{r['missed']:>7} {r['violations']:>6} {r['max_excess']:>11.2e} "
                      f"{str(r['feasible']):>9}")
    print("-" * 80)
    print(f"TOTAL violations: {total_viol}   worst excess: {worst:.2e}")
    print("PASS — abstraction is per-transition sound." if total_viol == 0
          else "FAIL — containment violated; abstraction is NOT sound.")


if __name__ == "__main__":
    run()
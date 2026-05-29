"""
Per-target dump for one row. Ranks targets by epsilon to confirm
whether K is dominated by sink, by a few targets, or by a plateau.
"""
import itertools
import numpy as np
from src.abstraction.algorithms.discretizer import (
    fast_norm_cdf, compute_range_epsilon_ij,
)


def dump_row(src_id, act_id, grid, plant_wrapper, label=""):
    n_g, n_v, n_vl = grid.shape
    i_g = src_id // (n_v * n_vl)
    rem = src_id % (n_v * n_vl)
    i_v, i_vl = rem // n_vl, rem % n_vl
    idx = (i_g, i_v, i_vl)
    bins = grid.bins

    centre    = np.array([(bins[d][idx[d]] + bins[d][idx[d]+1])/2.0 for d in range(3)])
    half_cell = [(bins[d][idx[d]+1] - bins[d][idx[d]])/2.0 for d in range(3)]

    act_val = plant_wrapper.get_action_space()[act_id]
    next_c, sigma = plant_wrapper.get_next_state_distribution(centre, act_val)

    print(f"\n=== {label}  state={src_id}  idx={idx}  centre={centre.tolist()} ===")
    print(f"action={act_id} ({act_val} m/s²) | next_c={next_c.tolist()} | "
          f"sigma={sigma.tolist()} | half_cell={half_cell}")

    if next_c[0] <= 0.0:
        print("[crash early return]")
        return

    stoch = [d for d in range(3) if sigma[d] > 0]
    nrad  = 4.0 * max(sigma)
    ranges = []
    for d in range(3):
        if sigma[d] > 0:
            s = np.digitize(next_c[d] - nrad, bins[d]) - 1
            e = np.digitize(next_c[d] + nrad, bins[d]) - 1
        else:
            s = e = np.digitize(next_c[d], bins[d]) - 1
        ranges.append(range(max(0, s), min(grid.shape[d]-1, e) + 1))

    rows, total_in = [], 0.0
    for tgt in itertools.product(*ranges):
        prob = 1.0
        for d in range(3):
            lo, hi = bins[d][tgt[d]], bins[d][tgt[d]+1]
            if sigma[d] > 0:
                prob *= fast_norm_cdf(hi, next_c[d], sigma[d]) \
                      - fast_norm_cdf(lo, next_c[d], sigma[d])
            else:
                prob *= 1.0 if lo <= next_c[d] < hi else 0.0
        if prob <= 0.0:
            continue
        eps = 0.0
        for d in stoch:
            eps += compute_range_epsilon_ij(
                next_c[d] - half_cell[d], next_c[d] + half_cell[d],
                bins[d][tgt[d]], bins[d][tgt[d]+1], sigma[d])
        total_in += prob
        off_vl = (bins[2][tgt[2]] + bins[2][tgt[2]+1])/2.0 - next_c[2]
        rows.append((tgt, off_vl, prob, eps))

    p_sink = max(0.0, 1.0 - total_in)
    eps_sink = sum(
        compute_range_epsilon_ij(next_c[d] - half_cell[d], next_c[d] + half_cell[d],
                                  bins[d][0], bins[d][-1], sigma[d])
        for d in stoch)

    rows.sort(key=lambda r: -r[3])
    print(f"  {'target idx':<14} {'off_vl':>8} {'prob':>10} {'epsilon':>10}")
    for tgt, off, prob, eps in rows[:12]:
        print(f"  {str(tgt):<14} {off:>8.3f} {prob:>10.5f} {eps:>10.5f}")

    K_in   = sum(r[3] for r in rows)
    K_full = K_in + (eps_sink if p_sink > 1e-6 else 0.0)
    print(f"  n_targets={len(rows)} | total_in={total_in:.6f} | "
          f"p_sink={p_sink:.6f} | eps_sink={eps_sink:.6f}")
    print(f"  K(in-grid)={K_in:.6f}  K(with sink)={K_full:.6f}")